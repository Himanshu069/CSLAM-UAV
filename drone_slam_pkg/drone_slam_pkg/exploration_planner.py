#!/usr/bin/env python3
"""
FULL AUTONOMOUS EXPLORATION SYSTEM
- Frontier Detection: Finds unexplored areas
- RRT* Global Planner: Avoids local minima
- CBF based safety filter
- Potential Field Local Controller: Smooth reactive motion

"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, PoseArray, Pose, Point
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float32, Int32, Bool
from scipy.ndimage import binary_dilation
import numpy as np
import time
import math
import random
from scipy import ndimage
from dataclasses import dataclass
from typing import Optional

@dataclass
class CBFConstraint:
    """One linear CBF constraint: g^T u >= b"""
    g: np.ndarray   # gradient of SDF (unit vector by Eikonal)
    h: float        # barrier value  h_i(x) = tanh(a*(phi-d))
    b: float        # RHS = -(gamma/2a)*sinh(2a*(phi-d))
    label: str = ""
 
@dataclass
class CBFResult:
    u_safe:        np.ndarray
    u_des:         np.ndarray
    h1:            float          # obstacle barrier value in (-1, 1)
    h2:            float          # frontier barrier value in (-1, 1)
    phi1:          float          # raw SDF distance to obstacle (m)
    phi2:          float          # raw SDF distance to frontier (m)
    slack:         float          # ||u_safe - u_des|| (m/s)
    cbf1_active:   bool           # obstacle constraint fired
    cbf2_active:   bool           # frontier constraint fired
    speed_clipped: bool           # v_max clipped the result
    qp_infeasible: bool           # soft-CBF fallback was used
    delta:         float          # soft-CBF slack magnitude
 
 
def build_sdf_obstacle(map_data: np.ndarray, resolution: float) -> np.ndarray:
    """
    Signed Distance Function to obstacles.
      phi > 0  free space (distance to nearest obstacle surface)
      phi < 0  inside obstacle
      phi = 0  on obstacle surface
    Satisfies Eikonal ||grad phi|| = 1 a.e. (Rademacher's theorem).
    """
    obstacle_mask = (map_data >= 50)
    d_out = ndimage.distance_transform_edt(~obstacle_mask) * resolution
    d_in  = ndimage.distance_transform_edt( obstacle_mask) * resolution
    return np.where(obstacle_mask, -d_in, d_out)

def build_sdf_frontier(map_data: np.ndarray,
                        resolution: float,
                        min_cluster_cells: int = 5) -> Optional[np.ndarray]:
    """
    Signed Distance Function to significant unknown-cell clusters.
 
    Filters SLAM noise by keeping only connected unknown regions
    with area >= min_cluster_cells (8-connectivity).
 
    Returns None when no significant frontier exists (map fully explored).
    The returned field satisfies the Eikonal equation a.e., so the same
    T(.) smoothing proof (Raja et al. 2024, Lemma 1) applies as Lemma 2.
    """
    unknown_mask = (map_data == -1)
    if not unknown_mask.any():
        return None
 
    struct = ndimage.generate_binary_structure(2, 2)   # 8-connected
    labeled, n = ndimage.label(unknown_mask, structure=struct)
 
    filtered = np.zeros_like(unknown_mask, dtype=bool)
    for lid in range(1, n + 1):
        if np.sum(labeled == lid) >= min_cluster_cells:
            filtered |= (labeled == lid)
 
    if not filtered.any():
        return None
 
    d_out = ndimage.distance_transform_edt(~filtered) * resolution
    d_in  = ndimage.distance_transform_edt( filtered) * resolution
    return np.where(filtered, -d_in, d_out)

class TanhDualCBF:
    def __init__(self,
                d_safe:           float = 0.35,
                d_stop:           float = 0.50,
                a1:               float = 2.0,
                a2:               float = 1.5,
                gamma1:           float = 1.5,
                gamma2:           float = 1.0,
                v_max:            float = 0.20,
                min_cluster_cells: int  = 25):
        self.d_safe            = d_safe
        self.d_stop            = d_stop
        self.a1                = a1
        self.a2                = a2
        self.gamma1            = gamma1
        self.gamma2            = gamma2
        self.v_max             = v_max
        self.min_cluster_cells = min_cluster_cells

    def _h(self, phi: float, d: float, a: float) -> float:
        """h = tanh(a*(phi-d))  in (-1, 1)"""
        return math.tanh(a * (phi - d))
    
    def _b(self, phi: float, d: float, a: float, gamma: float) -> float:
        #b in inequality
        s = a * (phi - d)
        return -(gamma / (2.0 * a)) * math.sinh(2.0 * s)

    def _grad(self, sdf: np.ndarray,
            gx: int, gy: int,
            res: float) -> np.ndarray:
        H, W = sdf.shape
        def v(r, c):
            return float(sdf[max(0, min(H-1, r)), max(0, min(W-1, c))])
        gx_ = (v(gy, gx+1) - v(gy, gx-1)) / (2.0 * res)
        gy_ = (v(gy+1, gx) - v(gy-1, gx)) / (2.0 * res)
        g   = np.array([gx_, gy_])
        if np.linalg.norm(g) < 1e-8:
            return np.array([0.0, 0.0])   # flat SDF region, _project_single handles via g_sq < 1e-10
        return g    
     
    def _make_constraint(self, sdf, gx, gy, res, d, a, gamma, label):
        phi = float(sdf[gy, gx])
        return CBFConstraint(
            g     = self._grad(sdf, gx, gy, res),
            h     = self._h(phi, d, a),
            b     = self._b(phi, d, a, gamma),
            label = label,
        )
    
    def _project_single(self,
                         u: np.ndarray,
                         c: CBFConstraint):
        """
        Closed-form projection onto halfspace g^T u >= b.
 
        u* = u_des + lambda * g
        lambda* = (b - g^T u) / ||g||^2    (only when constraint violated)
 
        Returns (u*, lambda*)
        """
        gTu = float(c.g @ u)
        if gTu >= c.b:
            return u.copy(), 0.0
        g_sq = float(c.g @ c.g)
        if g_sq < 1e-10:
            return u.copy(), 0.0
        lam = (c.b - gTu) / g_sq
        return u + lam * c.g, lam
    
    def _project_dual(self,
                       u_des: np.ndarray,
                       c1: CBFConstraint,
                       c2: CBFConstraint):
        """
        Closed-form projection onto intersection of two halfspaces.
 
            min  0.5*||u - u_des||^2
            s.t. g1^T u >= b1
                 g2^T u >= b2
 
        Four cases via KKT active-set enumeration.
 
        Case 4 (both active) closed form:
            [g1.g1  g1.g2] [l1]   [b1 - g1.u_des]
            [g2.g1  g2.g2] [l2] = [b2 - g2.u_des]
 
        det = ||g1||^2*||g2||^2 - (g1.g2)^2 = sin^2(theta)*||g1||^2*||g2||^2
        Zero iff g1 || g2 (obstacle and frontier in same direction).
 
        Returns (u*, qp_infeasible_flag)
        """
        v1 = float(c1.g @ u_des) < c1.b
        v2 = float(c2.g @ u_des) < c2.b
 
        # Case 1: neither violated
        if not v1 and not v2:
            return u_des.copy(), False
 
        # Case 2: only c1 violated — project, verify c2
        if v1 and not v2:
            u_c, lam = self._project_single(u_des, c1)
            if lam >= -1e-8 and float(c2.g @ u_c) >= c2.b:
                return u_c, False
 
        # Case 3: only c2 violated — project, verify c1
        if not v1 and v2:
            u_c, lam = self._project_single(u_des, c2)
            if lam >= -1e-8 and float(c1.g @ u_c) >= c1.b:
                return u_c, False
 
        # Case 4: both active — closed-form 2x2 Gram inverse
        g11 = float(c1.g @ c1.g)
        g22 = float(c2.g @ c2.g)
        g12 = float(c1.g @ c2.g)
        det = g11 * g22 - g12 * g12
 
        if abs(det) < 1e-10:
            # g1 || g2: project onto more critical constraint
            dom = c1 if c1.h < c2.h else c2
            return self._project_single(u_des, dom)[0], False
 
        r1   = c1.b - float(c1.g @ u_des)
        r2   = c2.b - float(c2.g @ u_des)
        lam1 = ( g22 * r1 - g12 * r2) / det
        lam2 = (-g12 * r1 + g11 * r2) / det
 
        # KKT dual feasibility: both multipliers must be >= 0
        if lam1 >= -1e-8 and lam2 >= -1e-8:
            return u_des + lam1 * c1.g + lam2 * c2.g, False
 
        # One multiplier negative: only the other constraint is truly active
        if lam1 < -1e-8:
            return self._project_single(u_des, c2)[0], False
        return self._project_single(u_des, c1)[0], False
 
    def _soft_qp(self,
                  u_des: np.ndarray,
                  c1: CBFConstraint,
                  c2: CBFConstraint):
        """
        Soft-CBF fallback — always feasible.
 
            min  0.5*||u-u_des||^2 + (p/2)*delta^2
            s.t. g1^T u >= b1 - delta
                 g2^T u >= b2 - delta,  delta >= 0
 
        Solved by bisection on delta in [0, delta_max].
        Provides ISSf bound (Romdlony & Jayawardhana 2016):
            h_i(t) >= h_i(0)*exp(-gamma_i*t)
                     - integral_0^t exp(-gamma_i*(t-s)) * delta*(s) ds
 
        Returns (u*, delta*)
        """
        lo, hi = 0.0, max(abs(c1.b), abs(c2.b), 0.1) * 2.0
        for _ in range(20):
            mid  = (lo + hi) / 2.0
            cs1  = CBFConstraint(g=c1.g, h=c1.h, b=c1.b - mid, label=c1.label)
            cs2  = CBFConstraint(g=c2.g, h=c2.h, b=c2.b - mid, label=c2.label)
            u_c, _ = self._project_dual(u_des, cs1, cs2)
            ok = (float(c1.g @ u_c) >= c1.b - mid - 1e-6 and
                  float(c2.g @ u_c) >= c2.b - mid - 1e-6)
            if ok: hi = mid
            else:  lo = mid
        delta = hi
        cs1 = CBFConstraint(g=c1.g, h=c1.h, b=c1.b - delta, label=c1.label)
        cs2 = CBFConstraint(g=c2.g, h=c2.h, b=c2.b - delta, label=c2.label)
        u_s, _ = self._project_dual(u_des, cs1, cs2)
        return u_s, delta
    

    def filter(self,
               u_des:   np.ndarray,
               sdf_obs: np.ndarray,
               sdf_unk: Optional[np.ndarray],
               gx: int, gy: int,
               res: float) -> CBFResult:
        """
        Apply dual CBF filter to a desired velocity command.
 
        Parameters
        ----------
        u_des   : APF desired velocity [vx, vy] (m/s)
        sdf_obs : obstacle SDF (metres)           built in map_cb
        sdf_unk : frontier SDF (metres) or None   built in map_cb
        gx, gy  : drone's current grid cell indices
        res     : map resolution (m/cell)
 
        Returns CBFResult with u_safe and full diagnostics.
        """
        u_des = np.asarray(u_des, dtype=float)
 
        # Constraint 1: obstacle
        c1 = self._make_constraint(
            sdf_obs, gx, gy, res,
            self.d_safe, self.a1, self.gamma1, "obs")
 
        # Constraint 2: frontier (may be inactive if map fully explored)
        frontier_active = sdf_unk is not None
        if not frontier_active:
            u_cbf, _ = self._project_single(u_des, c1)
            spd   = float(np.linalg.norm(u_cbf))
            clip  = spd > self.v_max
            u_safe = u_cbf / spd * self.v_max if clip and spd > 1e-8 else u_cbf.copy()
            return CBFResult(
                u_safe=u_safe, u_des=u_des,
                h1=c1.h, h2=float('inf'),
                phi1=float(sdf_obs[gy, gx]), phi2=float('inf'),
                slack=float(np.linalg.norm(u_safe - u_des)),
                cbf1_active=float(c1.g @ u_safe) < c1.b + 1e-4,
                cbf2_active=False, speed_clipped=clip,
                qp_infeasible=False, delta=0.0)
 
        c2 = self._make_constraint(
            sdf_unk, gx, gy, res,
            self.d_stop, self.a2, self.gamma2, "frontier")
 
        # Attempt hard dual projection
        u_cbf, infeas = self._project_dual(u_des, c1, c2)
 
        # Verify — fall back to soft QP if still infeasible
        delta = 0.0
        if (float(c1.g @ u_cbf) < c1.b - 1e-4 or
                float(c2.g @ u_cbf) < c2.b - 1e-4):
            u_cbf, delta = self._soft_qp(u_des, c1, c2)
            infeas = True
 
        # Speed ceiling (ball projection)
        spd  = float(np.linalg.norm(u_cbf))
        clip = spd > self.v_max
        u_safe = u_cbf / spd * self.v_max if clip and spd > 1e-8 else u_cbf.copy()
 
        return CBFResult(
            u_safe=u_safe, u_des=u_des,
            h1=c1.h, h2=c2.h,
            phi1=float(sdf_obs[gy, gx]), phi2=float(sdf_unk[gy, gx]),
            slack=float(np.linalg.norm(u_safe - u_des)),
            cbf1_active=float(c1.g @ u_safe) < c1.b + 1e-4,
            cbf2_active=float(c2.g @ u_safe) < c2.b + 1e-4,
            speed_clipped=clip, qp_infeasible=infeas, delta=delta)
 

class RRTNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__("autonomous_explorer")
        
        # --- FRONTIER DETECTION PARAMETERS ---
        self.declare_parameters("", [
            ("min_frontier_size", 1),        # Min cells in frontier cluster
            ("frontier_search_rate", 1.0),   # Hz
            ("exploration_complete_threshold", 0.95), # 95% map known
            ("frontier_weight_approach", 2.0),  
            ("map_frame", "map"),
            
            # --- RRT* PARAMETERS ---
            ("rrt_max_iter", 3000),
            ("rrt_step_size", 1.0),
            ("rrt_goal_sample_rate", 0.15),
            ("rrt_search_radius", 3.0),
            ("rrt_replan_rate", 0.5),        # Hz
            
            # --- POTENTIAL FIELD PARAMETERS ---
            ("k_att", 5.0),
            ("k_yaw", 1.5),
            ("k_rep", 0.5),
            ("influence_radius", 0.75),
            ("pf_update_rate", 10.0),        # Hz
            ("max_speed", 0.2),
            ("waypoint_threshold", 0.5),

            ("stuck_time_threshold", 4.0),   # seconds - time to detect astuck
            ("stuck_distance_threshold", 0.15),  # meters - min movement expected
                    
            # --- EXPLORATION STRATEGY ---
            ("frontier_weight_size", 1.0),   # Prefer larger frontiers
            ("frontier_weight_distance", 0.5), # Prefer closer frontiers

            #-----COSTMAP/ DRONE SAFETY---------
            ("drone_radius", 0.25),

            #-------MULTI-DRONE AVOIDANCE------------
            ("other_drone_pose_topic", "/x500_drone_1/localization_pose"),
            ("other_drone_safety_radius", 3),
            ("k_rep_drone", 200.0),
            ("other_drone_init_x", 0.0),
            ("other_drone_init_y", 0.0),
            ("frontier_search_radii", [ 5.0, 10.0, -1.0]),

            ("cbf_d_safe",            0.35),   # obstacle standoff (m)
            ("cbf_d_stop",            0.50),   # frontier standoff (m)
            ("cbf_a1",                2.0),    # tanh sharpness — obstacle
            ("cbf_a2",                1.5),    # tanh sharpness — frontier
            ("cbf_gamma1",            1.5),    # CBF gain — obstacle
            ("cbf_gamma2",            1.0),    # CBF gain — frontier
            ("cbf_min_cluster_cells", 25),     # min unknown cluster size
        ])
        
        self.map_frame = self.get_parameter("map_frame").value
        #Altitude
        self.takeoff_altitude = 1.3  # meters
        self.altitude_ready = False
        self.current_z = 0.0
        # Frontier params
        self.min_frontier_size = self.get_parameter("min_frontier_size").value
        self.frontier_rate = self.get_parameter("frontier_search_rate").value
        self.exploration_threshold = self.get_parameter("exploration_complete_threshold").value
        self.w_approach = self.get_parameter("frontier_weight_approach").value
        
        # RRT* params
        self.rrt_max_iter = self.get_parameter("rrt_max_iter").value
        self.rrt_step_size = self.get_parameter("rrt_step_size").value
        self.rrt_goal_rate = self.get_parameter("rrt_goal_sample_rate").value
        self.rrt_radius = self.get_parameter("rrt_search_radius").value
        self.rrt_replan_rate = self.get_parameter("rrt_replan_rate").value
        self._last_replan_time = None
        
        # PF params
        self.k_att = self.get_parameter("k_att").value
        self.k_rep = self.get_parameter("k_rep").value
        self.influence_radius = self.get_parameter("influence_radius").value
        self.pf_rate = self.get_parameter("pf_update_rate").value
        self.max_speed = self.get_parameter("max_speed").value
        self.wp_threshold = self.get_parameter("waypoint_threshold").value
        
        # Exploration strategy
        self.stuck_time_threshold = self.get_parameter("stuck_time_threshold").value
        self.stuck_distance_threshold = self.get_parameter("stuck_distance_threshold").value
        self.w_size = self.get_parameter("frontier_weight_size").value
        self.w_dist = self.get_parameter("frontier_weight_distance").value
        self._last_apf_forces = {
            "att": (0.0, 0.0),
            "rep_obs": (0.0, 0.0),
            "rep_drone": (0.0, 0.0),
            "total": (0.0, 0.0),
        }
        
        #Safety/costmap
        self.drone_radius = self.get_parameter("drone_radius").value
        
        # Multi-drone avoidance
        self.other_drone_pose_topic = self.get_parameter("other_drone_pose_topic").value
        self.other_drone_safety_radius = self.get_parameter("other_drone_safety_radius").value
        self.k_rep_drone = self.get_parameter("k_rep_drone").value
        self.other_drone_x = None
        self.other_drone_y = None
        self.other_drone_init_x = self.get_parameter("other_drone_init_x").value
        self.other_drone_init_y = self.get_parameter("other_drone_init_y").value

        raw_radii = self.get_parameter("frontier_search_radii").value
        self.frontier_search_radii = [float(r) for r in raw_radii]
        
        self.cbf = TanhDualCBF(
            d_safe            = self.get_parameter("cbf_d_safe").value,
            d_stop            = self.get_parameter("cbf_d_stop").value,
            a1                = self.get_parameter("cbf_a1").value,
            a2                = self.get_parameter("cbf_a2").value,
            gamma1            = self.get_parameter("cbf_gamma1").value,
            gamma2            = self.get_parameter("cbf_gamma2").value,
            v_max             = self.max_speed,
            min_cluster_cells = self.get_parameter("cbf_min_cluster_cells").value,
        )
        self._cbf_infeasible_count = 0
         
        self.inflated_map = None
        self.map_data = None
        self.map_info = None
        self.distance_field  = None
        self.sdf_obs         = None 
        self.sdf_unk         = None 

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_x = None
        self.goal_y = None
        self.autonomous_mode = True  # Auto-exploration enabled
        # self.motion_state = "FORWARD"  # "FORWARD" or "ALIGNING"
        self.last_position = (0.0, 0.0)
        self.last_stuck_check_time = None
        # self.alignment_start_time = None
        # self.max_alignment_time = 3.0  # seconds

        # Path following
        self.waypoints = []
        self.current_waypoint_idx = 0
        
        # Frontier tracking
        self.visited_frontiers = []  # Avoid revisiting
        self.exploration_complete = False

        self.stuck_check_position = (0.0, 0.0)
        self._free_cells_cache = None
        
        self.local_min_count = 0
        self.local_min_threshold = 5
        self.escape_cycles_remaining = 0
        self.escape_vec = (0.0, 0.0)
        self._smooth_rep = (0.0, 0.0)
        self.k_yaw = 0.3
        self.MAX_YAW_RATE = 0.5 

        self._total_path_length = 0.0
        self._exploration_start_time = self.get_clock().now()
        self._last_explored_area = 0.0
        self._last_coverage_time = self.get_clock().now()
        self._running_min_clearance = float('inf')

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1, 
        )
        
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_path = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, "map", self.map_cb, qos_map)
        self.create_subscription(VehicleLocalPosition, "fmu/out/vehicle_local_position", 
                                self.pos_cb, qos_px4)
        self.create_subscription(VehicleAttitude, "fmu/out/vehicle_attitude", 
                                self.attitude_cb, qos_px4)
        self.create_subscription(PoseStamped, "goal_pose", self.goal_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.other_drone_pose_topic,
                                self.other_drone_pose_cb, qos_map)

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "global_path", qos_path)
        self.frontier_pub = self.create_publisher(MarkerArray, "frontiers", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "current_goal", 10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, "inflated_map", qos_map)
        self.apf_path_pub = self.create_publisher(Path, "apf_path", qos_path)
        self.waypoints_pub = self.create_publisher(PoseArray, "rrt_waypoints", qos_path)
        self.apf_markers_pub = self.create_publisher(MarkerArray, "apf_forces", 10)

        #DATA logging
        # Coverage
        self.metrics_coverage_pub = self.create_publisher(Float32, "metrics/coverage_percent", 10)
        self.metrics_coverage_rate_pub = self.create_publisher(Float32, "metrics/coverage_rate", 10)
        self.metrics_area_pub = self.create_publisher(Float32, "metrics/explored_area_m2", 10)

        # Path efficiency  
        self.metrics_path_length_pub = self.create_publisher(Float32, "metrics/path_length_m", 10)
        self.metrics_path_efficiency_pub = self.create_publisher(Float32, "metrics/path_efficiency", 10)
        self.metrics_speed_pub = self.create_publisher(Float32, "metrics/avg_speed", 10)

        # Planning
        self.metrics_rrt_time_pub = self.create_publisher(Float32, "metrics/rrt_solve_time_ms", 10)
        self.metrics_rrt_iters_pub = self.create_publisher(Int32, "metrics/rrt_iterations_used", 10)
        self.metrics_rrt_fail_pub = self.create_publisher(Bool, "metrics/rrt_failed", 10)
        self.metrics_frontier_count_pub = self.create_publisher(Int32, "metrics/frontier_count", 10)

        # Safety
        self.metrics_clearance_pub = self.create_publisher(Float32, "metrics/obstacle_clearance_m", 10)
        self.metrics_min_clearance_pub = self.create_publisher(Float32, "metrics/min_clearance_running", 10)
        self.metrics_force_ratio_pub = self.create_publisher(Float32, "metrics/apf_force_ratio", 10)

        # Internal events 
        self.metrics_stuck_pub = self.create_publisher(Bool, "metrics/stuck_event", 10)
        self.metrics_local_min_pub = self.create_publisher(Int32, "metrics/local_min_count", 10)
        self.metrics_escape_pub = self.create_publisher(Bool, "metrics/escape_triggered", 10)
        self.metrics_replan_pub = self.create_publisher(Bool, "metrics/rrt_replan_triggered", 10)
        self.metrics_tracking_error_pub = self.create_publisher(Float32, "metrics/path_tracking_error_m", 10)

        self.metrics_cbf_h1_pub         = self.create_publisher(Float32, "metrics/cbf_h1",               10)
        self.metrics_cbf_h2_pub         = self.create_publisher(Float32, "metrics/cbf_h2",               10)
        self.metrics_cbf_phi1_pub       = self.create_publisher(Float32, "metrics/cbf_phi1_m",           10)
        self.metrics_cbf_phi2_pub       = self.create_publisher(Float32, "metrics/cbf_phi2_m",           10)
        self.metrics_cbf_slack_pub      = self.create_publisher(Float32, "metrics/cbf_slack",            10)
        self.metrics_cbf1_active_pub    = self.create_publisher(Bool,    "metrics/cbf1_obstacle_active", 10)
        self.metrics_cbf2_active_pub    = self.create_publisher(Bool,    "metrics/cbf2_frontier_active", 10)
        self.metrics_cbf_infeasible_pub = self.create_publisher(Int32,   "metrics/cbf_infeasible_count", 10)
        self.metrics_cbf_delta_pub      = self.create_publisher(Float32, "metrics/cbf_soft_delta",       10)
        self.metrics_cbf_speed_clip_pub = self.create_publisher(Bool,    "metrics/cbf_speed_clipped",    10)
        self.cbf_viz_pub = self.create_publisher(MarkerArray, "cbf_viz", 10)
        
        self.frontier_timer = self.create_timer(1.0 / self.frontier_rate, self.frontier_search_loop)
        self.rrt_timer = self.create_timer(1.0 / self.rrt_replan_rate, self.rrt_replan)
        self.pf_timer = self.create_timer(1.0 / self.pf_rate, self.potential_field_control)
        
        self.get_logger().info(" AUTONOMOUS EXPLORER ONLINE!")
        self.get_logger().info("   Frontier Detection: ON")
        self.get_logger().info("   RRT* Planning: ON")
        self.get_logger().info("   Potential Field Control: ON")
        self.get_logger().info(f"   Drone radius (inflation): {self.drone_radius:.2f} m")
        self.get_logger().info(f"   Other drone pose topic:  {self.other_drone_pose_topic}")
        self.get_logger().info(f"   Drone repulsion gain:    {self.k_rep_drone}")
        self.get_logger().info(
            f"   Other drone init pose:   ({self.other_drone_init_x:.2f}, {self.other_drone_init_y:.2f})"
        )
        self.get_logger().info(f"   APF viz topic: ~/apf_forces  (MarkerArray)")
    
    def pos_cb(self, msg):
        self.current_x = msg.y
        self.current_y = msg.x
        self.current_z = msg.z
        
        if not self.altitude_ready:
            self.get_logger().info(
                "Waiting for takeoff... current altitude: %.2f m" % -self.current_z,
                throttle_duration_sec=2.0
            )
            if (-self.current_z) >= self.takeoff_altitude *0.95:
                self.altitude_ready = True
                self.last_position = (self.current_x, self.current_y)  
                self._exploration_start_time = self.get_clock().now() 
                self.get_logger().info(
                    f"Takeoff altitude reached: {(-self.current_z):.2f} m"
            )
            else:
                return  
            
        dx = self.current_x - self.last_position[0]
        dy = self.current_y - self.last_position[1]
        step = math.hypot(dx, dy)
        if step < 0.5:  # ignore teleports/jumps
            self._total_path_length += step

        self.last_position = (self.current_x, self.current_y)

        elapsed = (self.get_clock().now() - self._exploration_start_time).nanoseconds / 1e9
        avg_speed = self._total_path_length / max(elapsed, 0.001)

        msg_f = Float32(); msg_f.data = self._total_path_length
        self.metrics_path_length_pub.publish(msg_f)

        msg_f = Float32(); msg_f.data = float(avg_speed)
        self.metrics_speed_pub.publish(msg_f)
    
    def attitude_cb(self, msg):
        # PX4 Quaternions are typically [w, x, y, z]
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw_ned = math.atan2(siny_cosp, cosy_cosp)     
        self.current_yaw = self.normalize_angle(math.pi / 2.0 - yaw_ned)    
    
    def _build_distance_field(self):
        """
        Compute a float32 array (same shape as map) where every cell holds the
        Euclidean distance (metres) to the nearest obstacle cell.
        Called inside map_cb after _inflate_map.
        """
        if self.map_data is None:
            return
        obstacle_mask = (self.map_data >= 50)
        # EDT returns distance in *cells*; multiply by resolution for metres
        dist_cells = ndimage.distance_transform_edt(~obstacle_mask)
        self.distance_field = dist_cells * self.map_info.resolution  

    def map_cb(self, msg):
        self.map_info = msg.info
        raw = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        self.map_data = raw

        self.inflated_map = self._inflate_map(raw, msg.info.resolution)
        self._free_cells_cache = np.argwhere(self.inflated_map == 0)

        unique, counts = np.unique(self.map_data, return_counts=True)
        self.get_logger().info(
        f"Map values: {dict(zip(unique.tolist(), counts.tolist()))}",
        throttle_duration_sec=5.0  
        )
        self._build_distance_field()

        self.sdf_obs = build_sdf_obstacle(raw, msg.info.resolution)
        self.sdf_unk = build_sdf_frontier(raw, msg.info.resolution,
                                           self.cbf.min_cluster_cells)
        
        self._publish_inflated_map(msg)
    
    def goal_cb(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.current_waypoint_idx = 0
        self.autonomous_mode = False # Disable auto-exploration
        # self.motion_state = "FORWARD"
        self.get_logger().info(f"Manual Goal Set: ({self.goal_x:.1f}, {self.goal_y:.1f})")
        self.get_logger().info(" Autonomous exploration PAUSED")
        self.rrt_replan()
    
    def other_drone_pose_cb(self, msg: PoseWithCovarianceStamped):
        self.other_drone_x = msg.pose.pose.position.x + self.other_drone_init_x
        self.other_drone_y = msg.pose.pose.position.y + self.other_drone_init_y
        self.get_logger().debug(
            f"Other drone at ({self.other_drone_x:.2f}, {self.other_drone_y:.2f}) "
            f"[raw=({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}) "
            f"+ init=({self.other_drone_init_x:.2f}, {self.other_drone_init_y:.2f})]"
        )

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi: 
            angle += 2.0 * math.pi
        return angle
    
    def _inflate_map(self, raw: np.ndarray, resolution: float) -> np.ndarray:
        """
        Binary costmap:
        - Obstacles (>=50): 100
        - Within drone_radius of obstacle: 99 (lethal)
        - Within drone_radius of unknown (-1) border: 99 (lethal)
        - Everything else: passthrough (0 or -1)
        """
        obstacle_mask = (raw >= 50)
        # unknown_mask = (raw == -1)
        # free_mask = (raw == 0)

        # Inflate obstacles
        inflate_cells = max(1, int(math.ceil(self.drone_radius / resolution)))
        struct = np.ones((2 * inflate_cells + 1, 2 * inflate_cells + 1), dtype=bool)

        inflated_obstacle = binary_dilation(obstacle_mask, structure=struct)

        inflated = raw.copy().astype(np.int8)
        
        # Apply lethal zones (don't overwrite actual obstacle cells)
        inflated[inflated_obstacle & ~obstacle_mask] = 99
        # inflated[unknown_border_mask] = 99
        inflated[obstacle_mask] = 100

        self.get_logger().debug(
            f"Binary costmap: obstacles={int(obstacle_mask.sum())}, "
            f"lethal_obs={int((inflated_obstacle & ~obstacle_mask).sum())}, "
            f"(inflate_radius={self.drone_radius:.2f}m = {inflate_cells} cells)"
        )
        return inflated
    
    def _publish_inflated_map(self, original_msg: OccupancyGrid):
        if self.inflated_map is None:
            return
        out = OccupancyGrid()
        out.header = original_msg.header
        out.info = original_msg.info
        out.data = self.inflated_map.flatten().tolist()
        self.inflated_map_pub.publish(out)
        self.get_logger().info("Published inflated map", throttle_duration_sec=5.0)  
    
        
    def frontier_search_loop(self):
        if self.map_data is None:
            return
        
        if not self.altitude_ready:
            return
        # Check exploration progress
        total_cells = self.map_data.size
        unknown_cells = np.sum(self.map_data == -1)
        explored_ratio = 1.0 - (unknown_cells / total_cells)

        resolution = self.map_info.resolution
        explored_area = float(np.sum(self.map_data == 0)) * (resolution ** 2)
        
        msg_f = Float32(); msg_f.data = float(explored_ratio * 100.0)
        self.metrics_coverage_pub.publish(msg_f)

        msg_f = Float32(); msg_f.data = explored_area
        self.metrics_area_pub.publish(msg_f)
        
        now = self.get_clock().now()
        dt = (now - self._last_coverage_time).nanoseconds / 1e9
        if dt > 0.5:
            rate = (explored_area - self._last_explored_area) / dt
            msg_f = Float32(); msg_f.data = float(rate)
            self.metrics_coverage_rate_pub.publish(msg_f)
            self._last_explored_area = explored_area
            self._last_coverage_time = now

        if self._total_path_length > 0.1:
            efficiency = explored_area / self._total_path_length
            msg_f = Float32(); msg_f.data = float(efficiency)
            self.metrics_path_efficiency_pub.publish(msg_f)

        if explored_ratio > self.exploration_threshold:
            if not self.exploration_complete:
                self.get_logger().info(f"EXPLORATION COMPLETE! ({explored_ratio*100:.1f}% mapped)")
                self.exploration_complete = True
                self.goal_x = None
                self.publish_zero_velocity()
            return
        
        # Only search for new frontier if no current goal (or in autonomous mode)
        if self.goal_x is not None and not self.autonomous_mode:
            return
        
        # If current goal exists and we're far from it, keep going
        if self.goal_x is not None:
            dist = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
            if dist > 1.0:  # Still navigating to current frontier
                return
        
        # Find frontiers
        frontiers = []
        for radius in self.frontier_search_radii:
            frontiers = self.find_frontiers(search_radius=radius if radius > 0 else None)

            if frontiers:
                self.get_logger().info(
                    f"Frontiers found at radius: "
                    f"{'full map' if radius < 0 else f'{radius:.1f} m'}"
                )
                break        

        msg_i = Int32(); msg_i.data = len(frontiers)
        self.metrics_frontier_count_pub.publish(msg_i)
        if not frontiers:
            self.get_logger().warn("No frontiers found!")
            return
        
        # Select best frontier
        best_frontier = self.select_best_frontier(frontiers)
        
        if best_frontier:
            self.goal_x, self.goal_y = best_frontier
            self.autonomous_mode = True
            self.current_waypoint_idx = 0
            # self.motion_state = "FORWARD" 
            self.get_logger().info(f"New Exploration Goal: ({self.goal_x:.1f}, {self.goal_y:.1f})")
            
            # Publish goal marker
            self.publish_goal_marker(self.goal_x, self.goal_y)
            
            # Visualize frontiers
            self.publish_frontier_markers(frontiers, best_frontier)
            
            # Trigger immediate replan
            self.rrt_replan()
    
    def find_frontiers(self, search_radius=None):
        """
        Find frontier cells (boundaries between known-free and unknown space)
        Returns list of frontier clusters: [[(x1,y1), (x2,y2), ...], [...], ...]
        """
        if self.map_data is None:
            return []
        
        height, width = self.map_data.shape
        if search_radius is not None and search_radius > 0:
            cx, cy = self.world_to_grid(self.current_x, self.current_y)
            if cx is None:                    # ← add this guard
                return []  
            r_cells = int(math.ceil(search_radius / self.map_info.resolution))
            row_min = max(0,      cy - r_cells)
            row_max = min(height, cy + r_cells)
            col_min = max(0,      cx - r_cells)
            col_max = min(width,  cx + r_cells)
        else:
            row_min, row_max = 0, height
            col_min, col_max = 0, width
        # 1. Create binary maps
        map_slice = self.map_data[row_min:row_max, col_min:col_max]
        
        free_cells = (map_slice == 0)  # Known free
        unknown_cells = (map_slice == -1)  # Unknown
        
        # Dilate free cells by 1 pixel
        dilated_unknown = binary_dilation(unknown_cells, structure=np.ones((3,3)))
        
        # Frontier = dilated free cells that overlap with unknown
        frontier_mask = free_cells & dilated_unknown
        
        # 3. Label connected frontier regions
        labeled, num_features = ndimage.label(frontier_mask)
        
        # 4. Extract frontier clusters
        frontiers = []
        
        for label_id in range(1, num_features + 1):
            cluster = np.argwhere(labeled == label_id)
            
            # Filter small clusters
            if len(cluster) < self.min_frontier_size:
                continue
            
            # Convert to world coordinates and get centroid
            world_points = []
            for gy, gx in cluster:  # Note: argwhere returns (row, col) = (y, x)
                wx, wy = self.grid_to_world(gx + col_min, gy + row_min)
                world_points.append((wx, wy))
            
            # Store cluster
            frontiers.append(world_points)
        
        self.get_logger().info(f"Found {len(frontiers)} frontier clusters")
        return frontiers
    
    def select_best_frontier(self, frontiers):
        """
        Select best frontier based on:
        - Size (information gain)
        - Distance (travel cost)
        - Novelty (not visited before)
        """
        if not frontiers:
            return None
        
        best_score = -float('inf')
        best_centroid = None
        
        for cluster in frontiers:
            # Compute centroid
            centroid_x = sum(p[0] for p in cluster) / len(cluster)
            centroid_y = sum(p[1] for p in cluster) / len(cluster)
            
            gx, gy = self.world_to_grid(centroid_x, centroid_y)
            if gx is None:
                continue
            if self.inflated_map[gy, gx] >= 50:
                continue
       
            if self.is_visited((centroid_x, centroid_y)):
                continue
            
            # Size score (larger = more info gain)
            size_score = len(cluster)
            
            # Distance score (closer = better)
            distance = math.hypot(centroid_x - self.current_x, centroid_y - self.current_y)
            distance_score = 1.0 / (distance + 0.1)  # Avoid division by zero
            
            # Combined score
            approach_score = self.score_approach_corridor(self.current_x, self.current_y, centroid_x, centroid_y)
            score = (self.w_size * size_score) + (self.w_dist * distance_score * 100) + (self.w_approach * approach_score * 100)            
            if score > best_score:
                best_score = score
                best_centroid = (centroid_x, centroid_y)
        
        # Mark as visited
        if best_centroid:
            self.visited_frontiers.append(best_centroid)
        if best_centroid is None:
            self.get_logger().warn("No valid frontiers found")
            return None
            # return self._select_best_frontier_unconstrained(frontiers)
        return best_centroid
    
    def score_approach_corridor(self, x1, y1, x2, y2, sample_res=0.1):
        if self.map_data is None:
            return 0.0
        dist  = math.hypot(x2 - x1, y2 - y1)
        steps = max(int(dist / sample_res), 1)
        known_free  = 0
        total_steps = 0
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            gx, gy = self.world_to_grid(x, y)
            if gx is None:
                total_steps += 1
                continue
            if self.map_data[gy, gx] == 0:
                known_free += 1
            total_steps += 1
        return known_free / max(total_steps, 1)
    
    def is_visited(self, frontier_pos, threshold=2.0):
        """Check if frontier is too close to already visited frontiers"""
        for visited in self.visited_frontiers:
            dist = math.hypot(frontier_pos[0] - visited[0], frontier_pos[1] - visited[1])
            if dist < threshold:
                return True
        return False


    # ==================== RRT* GLOBAL PLANNER ====================
    
    def rrt_replan(self):
        """Run RRT* to generate global waypoint path"""
        if self.inflated_map is None or self.goal_x is None:
            return
        
        if not self.altitude_ready:
            return
        
        if self._last_replan_time is not None:
            elapsed = (self.get_clock().now() - self._last_replan_time).nanoseconds / 1e9
            if elapsed < 3.0:  # minimum replan interval
                return
        
        dist = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
        if dist < 0.5:
            self.get_logger().info("Goal Reached!")
            self.goal_x = None
            self.waypoints = []
            self.publish_zero_velocity()
            return
        
        self.get_logger().info("Running RRT*...")

        msg_b = Bool(); msg_b.data = True
        self.metrics_replan_pub.publish(msg_b)
        
        start = (self.current_x, self.current_y)
        goal = (self.goal_x, self.goal_y)
        
        # Run RRT*
        path = self.rrt_star(start, goal)
        
        if path:
            # safe_path = self._truncate_at_unknown(path)
            # self.waypoints = safe_path
            self.waypoints = path
            self.current_waypoint_idx = 1
            self._last_replan_time = self.get_clock().now()
            self.last_stuck_check_time = None
            self.stuck_check_position = (self.current_x, self.current_y)
            self.publish_path_viz(path)  
            self.get_logger().info(
                f"Path found: {len(path)} waypoints, "
                # f"safe portion: {len(safe_path)} waypoints"
            )
        else:
            self.get_logger().warn(" RRT* failed - will try again")
            msg_b = Bool(); msg_b.data = True
            self.metrics_rrt_fail_pub.publish(msg_b)
    
    # def _truncate_at_unknown(self, path, sample_res=0.1):
    #     if self.map_data is None:
    #         return path
    #     safe = [path[0]]
    #     for i in range(1, len(path)):
    #         x1, y1 = path[i - 1]
    #         x2, y2 = path[i]
    #         if self._segment_has_unknown(x1, y1, x2, y2, sample_res):
    #             self.get_logger().info(
    #                 f"Path truncated at segment {i}/{len(path)} "
    #                 f"— holding until map fills in"
    #             )
    #             break  # don't append — hold before unknown space
    #         safe.append((x2, y2))
    #     if len(safe) <= 1:
    #         self.get_logger().warn(
    #             "No safe waypoints available — holding position"
    #         )
    #         return []
    #     return safe
    
    # def _segment_has_unknown(self, x1, y1, x2, y2, sample_res=0.1):
    #     dist  = math.hypot(x2 - x1, y2 - y1)
    #     steps = max(int(dist / sample_res), 1)
    #     for i in range(steps + 1):
    #         t  = i / steps
    #         x  = x1 + t * (x2 - x1)
    #         y  = y1 + t * (y2 - y1)
    #         gx, gy = self.world_to_grid(x, y)
    #         if gx is None:
    #             return True
    #         if self.map_data[gy, gx] == -1:
    #             return True
    #     return False

    def rrt_star(self, start, goal):
        """RRT* algorithm - returns list of (x,y) waypoints"""
        _t_start = time.monotonic()

        start_node = RRTNode(start[0], start[1])
        nodes = [start_node]
        
        for i in range(self.rrt_max_iter):
            # Sample random point
            if random.random() < self.rrt_goal_rate:
                rand_x, rand_y = goal
            else:
                rand_x, rand_y = self.sample_free()
            
            # Find nearest node
            nearest = min(nodes, key=lambda n: math.hypot(n.x - rand_x, n.y - rand_y))
            
            # Steer toward sample
            new_x, new_y = self.steer(nearest.x, nearest.y, rand_x, rand_y)
            
            # Check collision
            if not self.is_collision_free(nearest.x, nearest.y, new_x, new_y):
                continue
            
            # Create new node
            new_node = RRTNode(new_x, new_y)
            
            # Find nearby nodes
            nearby = [n for n in nodes if math.hypot(n.x - new_x, n.y - new_y) < self.rrt_radius]
            
            # Choose best parent
            min_cost = nearest.cost + math.hypot(new_x - nearest.x, new_y - nearest.y)
            best_parent = nearest
            
            for near_node in nearby:
                new_cost = near_node.cost + math.hypot(new_x - near_node.x, new_y - near_node.y)
                if new_cost < min_cost and self.is_collision_free(near_node.x, near_node.y, new_x, new_y):
                    min_cost = new_cost
                    best_parent = near_node
            
            new_node.parent = best_parent
            new_node.cost = min_cost
            nodes.append(new_node)
            
            # Rewire nearby nodes
            for near_node in nearby:
                new_cost = new_node.cost + math.hypot(near_node.x - new_x, near_node.y - new_y)
                if new_cost < near_node.cost and self.is_collision_free(new_x, new_y, near_node.x, near_node.y):
                    near_node.parent = new_node
                    near_node.cost = new_cost
            
            # Check if near goal
            if math.hypot(new_x - goal[0], new_y - goal[1]) < self.rrt_step_size:
                goal_node = RRTNode(goal[0], goal[1])
                goal_node.parent = new_node
                goal_node.cost = new_node.cost + math.hypot(goal[0] - new_x, goal[1] - new_y)
                
                # Extract path
                path = []
                current = goal_node
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                path.reverse()

                solve_ms = (time.monotonic() - _t_start) * 1000.0
                msg_f = Float32(); msg_f.data = float(solve_ms)
                self.metrics_rrt_time_pub.publish(msg_f)

                msg_i = Int32(); msg_i.data = int(i)
                self.metrics_rrt_iters_pub.publish(msg_i)
                return path
        
        return None
    
    def sample_free(self):
        if self._free_cells_cache is None or len(self._free_cells_cache) == 0:
            return (self.current_x, self.current_y)
        idx = random.randint(0, len(self._free_cells_cache) - 1)
        gy, gx = self._free_cells_cache[idx]
        return self.grid_to_world(gx, gy)
    
    def steer(self, from_x, from_y, to_x, to_y):
        """Extend from current point toward target by step_size"""
        dx = to_x - from_x
        dy = to_y - from_y
        dist = math.hypot(dx, dy)
        
        if dist < self.rrt_step_size:
            return (to_x, to_y)
        
        ratio = self.rrt_step_size / dist
        new_x = from_x + dx * ratio
        new_y = from_y + dy * ratio
        return (new_x, new_y)
    
    def is_collision_free(self, x1, y1, x2, y2):
        """Check if line segment is collision-free"""
        if self.inflated_map is None:
            return True
        
        dist = math.hypot(x2 - x1, y2 - y1)
        steps = int(dist / (self.map_info.resolution * 0.5)) + 1
        
        for i in range(steps + 1):
            t = i / max(steps, 1)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            gx, gy = self.world_to_grid(x, y)
            if gx is None:
                return False
            
            # Treat unknown as traversable (soptimistic)
            if self.inflated_map[gy, gx] >= 50:
                return False
        
        return True
    
    # ==================== POTENTIAL FIELD LOCAL CONTROLLER ====================
    
    def potential_field_control(self):
        """Execute motion toward current waypoint using potential fields"""
        if not self.waypoints or self.inflated_map is None:
            return
        
        if not self.altitude_ready:
            self.publish_zero_velocity()
            return
        if self.current_waypoint_idx >= len(self.waypoints):
            self.publish_zero_velocity()
            return
        
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        
        dist = math.hypot(target_x - self.current_x, target_y - self.current_y)
        if dist < self.wp_threshold:
            self.current_waypoint_idx += 1
            # self.motion_state = "FORWARD" 
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info(" Reached final waypoint")
                self.publish_zero_velocity()
                return
            target_x, target_y = self.waypoints[self.current_waypoint_idx]
            dist = math.hypot(target_x - self.current_x, target_y - self.current_y)
        
        lookahead_weights = [1.0, 0.1, 0.1]
        f_att = (0.0, 0.0)

        for i, w in enumerate(lookahead_weights):
            idx = self.current_waypoint_idx + i
            if idx >= len(self.waypoints):
                break
            wp_x, wp_y = self.waypoints[idx]
            fa = self.attractive_force(wp_x, wp_y)
            f_att = (f_att[0] + w * fa[0], f_att[1] + w * fa[1])

        f_rep = self.repulsive_force()
        
        alpha = 0.8
        self._smooth_rep = (
            alpha * f_rep[0] + (1 - alpha) * self._smooth_rep[0],
            alpha * f_rep[1] + (1 - alpha) * self._smooth_rep[1],
        )
        f_rep = self._smooth_rep

        f_rep_drone = self.repulsive_force_other_drone()

        fx = f_att[0] + f_rep[0] + f_rep_drone[0]
        fy = f_att[1] + f_rep[1] + f_rep_drone[1]

        att_mag = math.hypot(*f_att)
        rep_mag = math.hypot(*f_rep)
        if att_mag > 1e-4:
            msg_f = Float32(); msg_f.data = float(rep_mag / att_mag)
            self.metrics_force_ratio_pub.publish(msg_f)

        self.get_logger().info(
            f"F_att={math.hypot(*f_att):.1f} | "
            f"F_obs={math.hypot(*f_rep):.1f} | "
            f"F_drone={math.hypot(*f_rep_drone):.1f} |"
            f"F_total={math.hypot(fx,fy):.1f}",
            throttle_duration_sec=0.5
        )
        self._last_apf_forces = {
            "att":       f_att,
            "rep_obs":   f_rep,
            "rep_drone": f_rep_drone,
            "total":     (fx, fy),
        }
            
        force_mag = math.hypot(fx, fy)

        if force_mag < 0.15 and dist > self.wp_threshold:
            self.local_min_count += 1
        else:
            self.local_min_count = max(0, self.local_min_count - 1)

        if self.local_min_count >= self.local_min_threshold:
            self.local_min_count = 0
            self.escape_cycles_remaining = 4
            att_angle = math.atan2(f_att[1], f_att[0])
            perp = att_angle + random.choice([math.pi / 2, -math.pi / 2])
            self.escape_vec = (
                self.max_speed * math.cos(perp),
                self.max_speed * math.sin(perp),
            )
            self.get_logger().warn("Local minimum detected — injecting escape perturbation")
            msg_b = Bool(); msg_b.data = True
            self.metrics_escape_pub.publish(msg_b)
        
        msg_i = Int32(); msg_i.data = int(self.local_min_count)
        self.metrics_local_min_pub.publish(msg_i)
        
        if self.escape_cycles_remaining > 0:
            self.escape_cycles_remaining -= 1
            fx += self.escape_vec[0]
            fy += self.escape_vec[1]
            force_mag = math.hypot(fx, fy)  
        
        if force_mag < 1e-4:
            self.publish_zero_velocity()
            return
        
        desired_speed = self.max_speed * math.tanh(force_mag / self.k_att)

        # yaw = self.current_yaw

        # vx_body =  (math.cos(yaw) * fx/force_mag + math.sin(yaw) * fy/force_mag)*desired_speed
        # vy_body = (-math.sin(yaw) * fx/force_mag + math.cos(yaw) * fy/force_mag)*desired_speed
        vx = (fx / force_mag) * desired_speed
        vy = (fy / force_mag) * desired_speed
        # self._last_body_vel = (vx_body, vy_body)
        gx_cell, gy_cell = self.world_to_grid(self.current_x, self.current_y)

        if gx_cell is not None and self.sdf_obs is not None:
            cbf_result = self.cbf.filter(
                u_des   = np.array([vx, vy]),
                sdf_obs = self.sdf_obs,
                sdf_unk = self.sdf_unk,
                gx      = gx_cell,
                gy      = gy_cell,
                res     = self.map_info.resolution,
            )
 
            vx = float(cbf_result.u_safe[0])
            vy = float(cbf_result.u_safe[1])
 
            if cbf_result.qp_infeasible:
                self._cbf_infeasible_count += 1
 
            msg_f = Float32(); msg_f.data = float(cbf_result.h1)
            self.metrics_cbf_h1_pub.publish(msg_f)
 
            msg_f = Float32()
            msg_f.data = float(cbf_result.h2) if cbf_result.h2 != float('inf') else 1.0
            self.metrics_cbf_h2_pub.publish(msg_f)
 
            msg_f = Float32(); msg_f.data = float(cbf_result.phi1)
            self.metrics_cbf_phi1_pub.publish(msg_f)
 
            msg_f = Float32()
            msg_f.data = float(cbf_result.phi2) if cbf_result.phi2 != float('inf') else 99.0
            self.metrics_cbf_phi2_pub.publish(msg_f)
 
            msg_f = Float32(); msg_f.data = float(cbf_result.slack)
            self.metrics_cbf_slack_pub.publish(msg_f)
 
            msg_b = Bool(); msg_b.data = bool(cbf_result.cbf1_active)
            self.metrics_cbf1_active_pub.publish(msg_b)
 
            msg_b = Bool(); msg_b.data = bool(cbf_result.cbf2_active)
            self.metrics_cbf2_active_pub.publish(msg_b)
 
            msg_i = Int32(); msg_i.data = int(self._cbf_infeasible_count)
            self.metrics_cbf_infeasible_pub.publish(msg_i)
 
            msg_f = Float32(); msg_f.data = float(cbf_result.delta)
            self.metrics_cbf_delta_pub.publish(msg_f)
 
            msg_b = Bool(); msg_b.data = bool(cbf_result.speed_clipped)
            self.metrics_cbf_speed_clip_pub.publish(msg_b)
 
            self.get_logger().info(
                f"CBF | h1={cbf_result.h1:.3f} phi1={cbf_result.phi1:.2f}m "
                f"h2={cbf_result.h2:.3f} phi2={cbf_result.phi2:.2f}m "
                f"slack={cbf_result.slack:.3f} "
                f"{'[C1]' if cbf_result.cbf1_active else '    '}"
                f"{'[C2]' if cbf_result.cbf2_active else '    '}"
                f"{'[INFEAS]' if cbf_result.qp_infeasible else ''}",
                throttle_duration_sec=0.5)
            
            self._publish_cbf_viz(cbf_result)

        desired_yaw = math.atan2(vy, vx)
        yaw_error = desired_yaw - self.current_yaw
        # wrap to [-π, π]
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
        yaw_rate = 0.0
        if abs(yaw_error) > math.radians(5.0):
            yaw_rate = float(np.clip(self.k_yaw * yaw_error, -self.MAX_YAW_RATE, self.MAX_YAW_RATE))

        if self.check_if_stuck():
            self.get_logger().warn("Stuck , clearing waypoints to force replan")
            self.waypoints = []
            self.publish_zero_velocity()
            return
        
        self._publish_apf_markers()

        if gx_cell is not None and self.distance_field is not None:
            d = float(self.distance_field[gy_cell, gx_cell])
            msg_f = Float32(); msg_f.data = d
            self.metrics_clearance_pub.publish(msg_f)
            if d < self._running_min_clearance:
                self._running_min_clearance = d
            msg_f = Float32(); msg_f.data = float(self._running_min_clearance)
            self.metrics_min_clearance_pub.publish(msg_f)
 
        tracking_err = self._compute_tracking_error()
        msg_f = Float32(); msg_f.data = float(tracking_err)
        self.metrics_tracking_error_pub.publish(msg_f)
        self.publish_velocity(vx, vy, yaw_rate)
    
    def check_if_stuck(self):
        current_time = self.get_clock().now()
        if self._last_replan_time is not None:
            elapsed = (self.get_clock().now() - self._last_replan_time).nanoseconds / 1e9
            if elapsed < 3.0:
                return False

        if self.last_stuck_check_time is None:
            self.last_stuck_check_time = current_time
            self.stuck_check_position = (self.current_x, self.current_y)  
            return False
        
        time_elapsed = (current_time - self.last_stuck_check_time).nanoseconds / 1e9
        
        if time_elapsed >= self.stuck_time_threshold:
            distance_traveled = math.hypot(
                self.current_x - self.stuck_check_position[0],  
                self.current_y - self.stuck_check_position[1]  
            )
            self.last_stuck_check_time = current_time
            self.stuck_check_position = (self.current_x, self.current_y)  
            
            if distance_traveled < self.stuck_distance_threshold:
                msg_b = Bool(); msg_b.data = True
                self.metrics_stuck_pub.publish(msg_b)
                return True
        
        return False
    def attractive_force(self, target_x, target_y):
        """Pull toward target waypoint"""
        d_switch = 1.5

        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist = math.hypot(dx, dy)
        
        if dist < 0.001:
            return (0.0, 0.0)
        
        d_hat_x = dx / dist
        d_hat_y = dy / dist
        
        if dist <= d_switch:
            fx = self.k_att * dx
            fy = self.k_att * dy
        else:
            fx = self.k_att * d_switch * d_hat_x
            fy = self.k_att * d_switch * d_hat_y
        return (fx, fy)
    
    def repulsive_force(self):
        """Push away from obstacles"""

        if self.distance_field is None:
            return (0.0,0.0)
        
        d0 = self.influence_radius
        d_min = 0.05
        gx, gy = self.world_to_grid(self.current_x, self.current_y)
        if gx is None:
            return (0.0,0.0)
        
        d = float(self.distance_field[gy, gx])
        d = max(d, d_min)

        if d>= d0:
            return (0.0,0.0)
        
        scalar = self.k_rep * (1.0/d -1.0/d0) / (d **2)

        h = self.map_info.resolution
        H, W = self.distance_field.shape

        def _d(r, c):
            r = max(0, min(H - 1, r))
            c = max(0, min(W - 1, c))
            return float(self.distance_field[r, c])
        
        grad_x = (_d(gy, gx + 1) - _d(gy, gx - 1)) / (2 * h)
        grad_y = (_d(gy + 1, gx) - _d(gy - 1, gx)) / (2 * h)

        grad_mag = math.hypot(grad_x, grad_y)
        if grad_mag < 1e-6:
            # no repulsion needed
            return (0.0, 0.0)

        # Force = scalar * gradientd  
        fx = scalar * (grad_x / grad_mag)
        fy = scalar * (grad_y / grad_mag)

        return(fx, fy)

    
    def repulsive_force_other_drone(self):

        if self.other_drone_x is None or self.other_drone_y is None:
            return (0.0, 0.0)
        
        dx = self.current_x - self.other_drone_x
        dy = self.current_y - self.other_drone_y
        dist = max(math.hypot(dx, dy), 0.05)
        
        if dist >= self.other_drone_safety_radius:
            return (0.0, 0.0)
        
        d0 = self.other_drone_safety_radius
        scalar = self.k_rep_drone * (1.0 / dist - 1.0 / d0) / (dist ** 2)
        
        fx = scalar * (dx / dist)
        fy = scalar * (dy / dist)
        
        cap = max(math.hypot(*self.attractive_force(
            self.goal_x or self.current_x,
            self.goal_y or self.current_y)) * 2.0, 5.0)
        
        mag = math.hypot(fx, fy)
        if mag > cap:
            fx = fx / mag * cap
            fy = fy / mag * cap
        self.get_logger().warn(
            f"[DRONE PROXIMITY] dist={dist:.2f} m  repulsion=({fx:.2f}, {fy:.2f})",
            throttle_duration_sec=1.0,
        )
        return (fx, fy)
        # ==================== VISUALIZATION ====================
    
    def publish_frontier_markers(self, frontiers, best_frontier):
        """Visualize all frontiers and highlight the selected one"""
        markers = MarkerArray()
        
        for i, cluster in enumerate(frontiers):
            centroid_x = sum(p[0] for p in cluster) / len(cluster)
            centroid_y = sum(p[1] for p in cluster) / len(cluster)
            
            is_best = (best_frontier and 
                    abs(centroid_x - best_frontier[0]) < 0.1 and 
                    abs(centroid_y - best_frontier[1]) < 0.1)
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "frontiers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = centroid_x
            marker.pose.position.y = centroid_y
            marker.pose.position.z = 2.0
            
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            if is_best:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
            else:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5)  # Yellow
            
            markers.markers.append(marker)
        
        self.frontier_pub.publish(markers)
    
    def publish_goal_marker(self, x, y):
        """Publish current exploration goal"""
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 2.0
        self.goal_pub.publish(msg)
    
    # ==================== HELPER FUNCTIONS ====================
    
    def get_nearby_cells(self, radius):
        if self.map_info is None:
            return []
        
        gx, gy = self.world_to_grid(self.current_x, self.current_y)
        if gx is None:
            return []
        
        cell_radius = int(radius / self.map_info.resolution)
        cells = []
        
        for dy in range(-cell_radius, cell_radius + 1):
            for dx in range(-cell_radius, cell_radius + 1):
                nx, ny = gx + dx, gy + dy
                if (0 <= nx < self.map_info.width and 
                    0 <= ny < self.map_info.height):
                    cells.append((nx, ny))
        
        return cells
    
    def world_to_grid(self, wx, wy):
        if self.map_info is None:
            return None, None
        
        gx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        
        if (0 <= gx < self.map_info.width and 0 <= gy < self.map_info.height):
            return gx, gy
        return None, None
    
    def grid_to_world(self, gx, gy):
        if self.map_info is None:
            return 0.0, 0.0
        wx = (gx * self.map_info.resolution) + self.map_info.origin.position.x
        wy = (gy * self.map_info.resolution) + self.map_info.origin.position.y
        return wx, wy
    
    def publish_velocity(self, vx, vy, yaw_rate =0.0):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = yaw_rate
        self.vel_pub.publish(msg)
    
    def publish_zero_velocity(self):
        self.publish_velocity(0.0, 0.0)
    
    def _compute_tracking_error(self) -> float:
        """
        Perpendicular distance from current position to the
        nearest segment of the current RRT* waypoint path.
        """
        if len(self.waypoints) < 2 or self.current_waypoint_idx >= len(self.waypoints):
            return 0.0

        px, py = self.current_x, self.current_y
        min_dist = float('inf')

        # check segment from previous waypoint to current target
        start_idx = max(0, self.current_waypoint_idx - 1)
        end_idx   = min(self.current_waypoint_idx + 2, len(self.waypoints))

        for i in range(start_idx, end_idx - 1):
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[i + 1]

            seg_len = math.hypot(bx - ax, by - ay)
            if seg_len < 1e-6:
                dist = math.hypot(px - ax, py - ay)
            else:
                # project point onto segment, clamp to [0,1]
                t = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / (seg_len ** 2)
                t = max(0.0, min(1.0, t))
                closest_x = ax + t * (bx - ax)
                closest_y = ay + t * (by - ay)
                dist = math.hypot(px - closest_x, py - closest_y)

            min_dist = min(min_dist, dist)

        return min_dist if min_dist != float('inf') else 0.0

    def publish_path_viz(self, path):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()      
        waypoints_msg = PoseArray()
        waypoints_msg.header.frame_id = self.map_frame
        waypoints_msg.header.stamp = msg.header.stamp
        
        for (x, y) in path:
            p = PoseStamped()
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            msg.poses.append(p)

            pose = Pose()
            pose.position.x = x 
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            waypoints_msg.poses.append(pose)
        
        self.path_pub.publish(msg)
        self.waypoints_pub.publish(waypoints_msg)

    def _publish_apf_markers(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        forces = {
            0: ("att",       self._last_apf_forces["att"],       (0.0, 1.0, 0.0)),  # green
            1: ("rep_obs",   self._last_apf_forces["rep_obs"],   (1.0, 0.0, 0.0)),  # red
            2: ("rep_drone", self._last_apf_forces["rep_drone"], (1.0, 0.5, 0.0)),  # orange
            3: ("total",     self._last_apf_forces["total"],     (0.0, 0.0, 1.0)),  # blue
        }
        
        scale = 2.0  
        
        for id_, (label, (fx, fy), color) in forces.items():
            mag = math.hypot(fx, fy)
            if mag < 1e-4:
                continue
            
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now
            m.ns = "apf_forces"
            m.id = id_
            m.type = Marker.ARROW
            m.action = Marker.ADD
            
            start = Point()
            start.x = self.current_x
            start.y = self.current_y
            start.z = 0.5  
            
            end = Point()
            end.x = self.current_x + (fx / mag) * scale * math.tanh(mag)
            end.y = self.current_y + (fy / mag) * scale * math.tanh(mag)
            end.z = 0.5
            
            m.points = [start, end]
            
            m.scale.x = 0.05  
            m.scale.y = 0.1   
            m.scale.z = 0.1   
            
            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.color.a = 1.0
            
            markers.markers.append(m)
        
        self.apf_markers_pub.publish(markers)
    
    def _publish_cbf_viz(self, cbf_result: CBFResult):
            """
            Publish RViz markers for the CBF safety filter:
            - u_des  arrow  (yellow)
            - u_safe arrow  (green if untouched, red if modified)
            - d_safe circle (red ring around drone)
            - d_stop circle (cyan ring around drone)
            - constraint‑gradient arrows (magenta / cyan)
            - text overlay with h₁, h₂, φ₁, φ₂
            """
            markers = MarkerArray()
            now = self.get_clock().now().to_msg()
            px, py, pz = self.current_x, self.current_y, 0.6  # slightly above ground

            arrow_scale = 3.0  # visual length multiplier

            # ── 0  Delete‑all (keeps old IDs from lingering) ──
            delete = Marker()
            delete.header.frame_id = self.map_frame
            delete.header.stamp = now
            delete.ns = "cbf"
            delete.action = Marker.DELETEALL
            markers.markers.append(delete)

            # ── 1  u_des arrow (yellow) ──
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now
            m.ns = "cbf"
            m.id = 1
            m.type = Marker.ARROW
            m.action = Marker.ADD
            mag_des = float(np.linalg.norm(cbf_result.u_des))
            if mag_des > 1e-4:
                dx, dy = cbf_result.u_des / mag_des
                length = arrow_scale * mag_des
                m.points = [
                    Point(x=px, y=py, z=pz),
                    Point(x=px + dx * length, y=py + dy * length, z=pz),
                ]
            else:
                m.points = [Point(x=px, y=py, z=pz), Point(x=px, y=py, z=pz)]
            m.scale.x = 0.06  # shaft diameter
            m.scale.y = 0.12  # head diameter
            m.scale.z = 0.10
            m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)  # yellow
            markers.markers.append(m)

            # ── 2  u_safe arrow (green = unmodified, red = CBF intervened) ──
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now
            m.ns = "cbf"
            m.id = 2
            m.type = Marker.ARROW
            m.action = Marker.ADD
            mag_safe = float(np.linalg.norm(cbf_result.u_safe))
            if mag_safe > 1e-4:
                dx, dy = cbf_result.u_safe / mag_safe
                length = arrow_scale * mag_safe
                m.points = [
                    Point(x=px, y=py, z=pz),
                    Point(x=px + dx * length, y=py + dy * length, z=pz),
                ]
            else:
                m.points = [Point(x=px, y=py, z=pz), Point(x=px, y=py, z=pz)]
            m.scale.x = 0.07
            m.scale.y = 0.14
            m.scale.z = 0.12
            modified = cbf_result.cbf1_active or cbf_result.cbf2_active
            if modified:
                m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # red
            else:
                m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # green
            markers.markers.append(m)

            # ── 3  d_safe circle (obstacle standoff – red ring) ──
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now
            m.ns = "cbf"
            m.id = 3
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = 0.02
            d_safe = self.cbf.d_safe
            m.scale.x = d_safe * 2.0
            m.scale.y = d_safe * 2.0
            m.scale.z = 0.01  # flat disk
            alpha = 0.7 if cbf_result.cbf1_active else 0.25
            m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=alpha)
            markers.markers.append(m)

            # ── 4  d_stop circle (frontier standoff – cyan ring) ──
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now
            m.ns = "cbf"
            m.id = 4
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = 0.01
            d_stop = self.cbf.d_stop
            m.scale.x = d_stop * 2.0
            m.scale.y = d_stop * 2.0
            m.scale.z = 0.01
            alpha = 0.7 if cbf_result.cbf2_active else 0.15
            m.color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=alpha)
            markers.markers.append(m)

            # ── 5  Constraint‑1 gradient (obstacle, magenta arrow) ──
            gx_cell, gy_cell = self.world_to_grid(self.current_x, self.current_y)
            if gx_cell is not None and self.sdf_obs is not None:
                g1 = self.cbf._grad(self.sdf_obs, gx_cell, gy_cell,
                                    self.map_info.resolution)
                gn = float(np.linalg.norm(g1))
                if gn > 1e-6:
                    m = Marker()
                    m.header.frame_id = self.map_frame
                    m.header.stamp = now
                    m.ns = "cbf"
                    m.id = 5
                    m.type = Marker.ARROW
                    m.action = Marker.ADD
                    g1n = g1 / gn
                    glen = 0.6  # fixed visual length
                    m.points = [
                        Point(x=px, y=py, z=pz + 0.15),
                        Point(x=px + g1n[0] * glen,
                            y=py + g1n[1] * glen,
                            z=pz + 0.15),
                    ]
                    m.scale.x = 0.04
                    m.scale.y = 0.08
                    m.scale.z = 0.06
                    m.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)  # magenta
                    markers.markers.append(m)

            # ── 6  Constraint‑2 gradient (frontier, cyan arrow) ──
            if gx_cell is not None and self.sdf_unk is not None:
                g2 = self.cbf._grad(self.sdf_unk, gx_cell, gy_cell,
                                    self.map_info.resolution)
                gn = float(np.linalg.norm(g2))
                if gn > 1e-6:
                    m = Marker()
                    m.header.frame_id = self.map_frame
                    m.header.stamp = now
                    m.ns = "cbf"
                    m.id = 6
                    m.type = Marker.ARROW
                    m.action = Marker.ADD
                    g2n = g2 / gn
                    glen = 0.6
                    m.points = [
                        Point(x=px, y=py, z=pz + 0.25),
                        Point(x=px + g2n[0] * glen,
                            y=py + g2n[1] * glen,
                            z=pz + 0.25),
                    ]
                    m.scale.x = 0.04
                    m.scale.y = 0.08
                    m.scale.z = 0.06
                    m.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8)  # cyan
                    markers.markers.append(m)

            # ── 7  Text overlay ──
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = now
            m.ns = "cbf"
            m.id = 7
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = px
            m.pose.position.y = py
            m.pose.position.z = pz + 0.8
            m.scale.z = 0.18  # text height
            h2_str = f"{cbf_result.h2:.2f}" if cbf_result.h2 != float('inf') else "N/A"
            phi2_str = f"{cbf_result.phi2:.2f}" if cbf_result.phi2 != float('inf') else "N/A"
            flags = ""
            if cbf_result.cbf1_active:
                flags += " [OBS]"
            if cbf_result.cbf2_active:
                flags += " [FRO]"
            if cbf_result.qp_infeasible:
                flags += " [SOFT]"
            if cbf_result.speed_clipped:
                flags += " [CLIP]"
            m.text = (
                f"h1={cbf_result.h1:.2f}  phi1={cbf_result.phi1:.2f}m\n"
                f"h2={h2_str}  phi2={phi2_str}m\n"
                f"slack={cbf_result.slack:.3f}{flags}"
            )
            m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            markers.markers.append(m)

            # ── 8  "Intervention flash" – a sphere that pops when CBF fires ──
            if cbf_result.cbf1_active or cbf_result.cbf2_active:
                m = Marker()
                m.header.frame_id = self.map_frame
                m.header.stamp = now
                m.ns = "cbf"
                m.id = 8
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = px
                m.pose.position.y = py
                m.pose.position.z = pz
                r = 0.15
                m.scale.x = r * 2
                m.scale.y = r * 2
                m.scale.z = r * 2
                m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.6)
                m.lifetime.sec = 0
                m.lifetime.nanosec = 300_000_000  # flash for 0.3 s
                markers.markers.append(m)

            self.cbf_viz_pub.publish(markers)
        
def main():

    rclpy.init()
    node = AutonomousExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
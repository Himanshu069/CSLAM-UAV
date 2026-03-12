#!/usr/bin/env python3
"""
FULL AUTONOMOUS EXPLORATION SYSTEM
- Frontier Detection: Finds unexplored areas
- RRT* Global Planner: Avoids local minima
- Potential Field Local Controller: Smooth reactive motion

"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, PoseArray, Pose, Point
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, DistanceSensor
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from scipy.ndimage import binary_dilation
import numpy as np
import math
import random
from scipy import ndimage
from collections import deque

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
            ("k_att", 2.0),
            ("k_yaw", 1.5),
            ("k_rep", 40.0),
            ("influence_radius", 2.5),
            ("pf_update_rate", 10.0),        # Hz
            ("max_speed", 0.2),
            ("waypoint_threshold", 1.0),

            ("heading_tolerance", 30.0),     # degrees - move forward if within this
            ("stuck_time_threshold", 4.0),   # seconds - time to detect astuck
            ("stuck_distance_threshold", 0.15),  # meters - min movement expected
            ("alignment_yaw_gain", 1.0),     # yaw control gain during alignment
            ("forward_yaw_gain", 0.5),       # yaw control gain during forward motion
                      
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
            ("other_drone_init_y", 0.0)
        ])
        
        self.map_frame = self.get_parameter("map_frame").value
        #Altitude
        self.takeoff_altitude = 1.3  # meters
        self.altitude_ready = False
        self.current_z = 0.0
        self.current_range = 0.0
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
        self.heading_tolerance = math.radians(self.get_parameter("heading_tolerance").value)
        self.stuck_time_threshold = self.get_parameter("stuck_time_threshold").value
        self.stuck_distance_threshold = self.get_parameter("stuck_distance_threshold").value
        self.alignment_yaw_gain = self.get_parameter("alignment_yaw_gain").value
        self.forward_yaw_gain = self.get_parameter("forward_yaw_gain").value
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
        
        # Inflated map 
        self.inflated_map = None
        
        # State
        self.map_data = None
        self.map_info = None
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
        
        self.distance_field = None
        self.local_min_count = 0
        self.local_min_threshold = 5
        self.escape_cycles_remaining = 0
        self.escape_vec = (0.0, 0.0)
        
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
        self.create_subscription(DistanceSensor, "fmu/in/distance_sensor",
                                self.range_cb, qos_px4)

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "global_path", qos_path)
        self.frontier_pub = self.create_publisher(MarkerArray, "frontiers", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "current_goal", 10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, "inflated_map", qos_map)
        self.apf_path_pub = self.create_publisher(Path, "apf_path", qos_path)
        self.waypoints_pub = self.create_publisher(PoseArray, "rrt_waypoints", qos_path)
        self.apf_markers_pub = self.create_publisher(MarkerArray, "apf_forces", 10)

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
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        
        if not self.altitude_ready:
            self.get_logger().info(
                "Waiting for takeoff... current altitude: %.2f m" % -self.current_z,
                throttle_duration_sec=2.0
            )
            if (-self.current_z) >= self.takeoff_altitude:
                self.altitude_ready = True
                self.get_logger().info(
                    f"Takeoff altitude reached: {(-self.current_z):.2f} m"
            )
            else:
                return  # still no
    
    def range_cb(self, msg):
        self.current_range = msg.current_distance

        if not self.altitude_ready:
            self.get_logger().info(
                f"Waiting for takeoff... rangefinder: {self.current_range:.2f}m "
                f"/ {self.takeoff_altitude:.2f}m",
                throttle_duration_sec=1.0
                )
            
            if self.current_range >= (self.takeoff_altitude - 0.15):
                self.altitude_ready = True
                self.get_logger().info(
                    f"Takeoff altitude reached! Rangefinder: {self.current_range:.2f}m "
                    f"— starting exploration"
                    )
        else:
            #Safety monitor while flying
            if self.current_range < 0.4:
                self.get_logger().warn(
                    f"OBSTACLE CLOSE BELOW: {self.current_range:.2f}m — "
                    f"PX4 terrain follow should be rising!",
                    throttle_duration_sec=1.0
                    )

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
        
        unique, counts = np.unique(self.map_data, return_counts=True)
        self.get_logger().info(
        f"Map values: {dict(zip(unique.tolist(), counts.tolist()))}"
        )
        self._build_distance_field()
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
        free_mask = (raw == 0)

        # Inflate obstacles
        inflate_cells = max(1, int(math.ceil(self.drone_radius / resolution)))
        struct = np.ones((2 * inflate_cells + 1, 2 * inflate_cells + 1), dtype=bool)

        inflated_obstacle = binary_dilation(obstacle_mask, structure=struct)
        
        # Inflate unknown borders — only the free cells adjacent to unknown get marked
        # inflated_unknown = binary_dilation(unknown_mask, structure=struct)
        # unknown_border_mask = inflated_unknown & free_mask  # don't mark unknown itself as lethal

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
        self.get_logger().info("Published inflated map", throttle_duration_sec=5.0)  # add this
    
    # def fake_pose(self):
    #     # Slowly move in a circle
    #     t = self.get_clock().now().nanoseconds * 1e-9
    #     self.current_x = 5.0 * math.cos(0.2 * t)
    #     self.current_y = 5.0 * math.sin(0.2 * t)

    
    # ==================== FRONTIER DETECTION ====================
    
    def frontier_search_loop(self):
        if self.map_data is None:
            return
        
        if not self.altitude_ready:
            return
        # Check exploration progress
        total_cells = self.map_data.size
        unknown_cells = np.sum(self.map_data == -1)
        explored_ratio = 1.0 - (unknown_cells / total_cells)
        
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
        frontiers = self.find_frontiers()
        
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
    
    def find_frontiers(self):
        """
        Find frontier cells (boundaries between known-free and unknown space)
        Returns list of frontier clusters: [[(x1,y1), (x2,y2), ...], [...], ...]
        """
        if self.map_data is None:
            return []
        
        height, width = self.map_data.shape
        
        # 1. Create binary maps
        free_cells = (self.map_data == 0)  # Known free
        unknown_cells = (self.map_data == -1)  # Unknown
        
        # 2. Find edges of free space adjacent to unknown
        # Use morphological dilation to find neighbors
        
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
                wx, wy = self.grid_to_world(gx, gy)
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
        # Check if reached goal
        dist = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
        if dist < 0.5:
            self.get_logger().info("Goal Reached!")
            self.goal_x = None
            self.waypoints = []
            self.publish_zero_velocity()
            return
        
        self.get_logger().info("Running RRT*...")
        
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
            self.last_position = (self.current_x, self.current_y)
            self.current_waypoint_idx = 0
            self.publish_path_viz(path)  
            self.get_logger().info(
                f"Path found: {len(path)} waypoints, "
                # f"safe portion: {len(safe_path)} waypoints"
            )
        else:
            self.get_logger().warn(" RRT* failed - will try again")
    
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
                return path
        
        return None
    
    def sample_free(self):
        """Sample random point in map bounds"""
        if self.map_info is None:
            return (0.0, 0.0)
        
        x = random.uniform(
            self.map_info.origin.position.x,
            self.map_info.origin.position.x + self.map_info.width * self.map_info.resolution
        )
        y = random.uniform(
            self.map_info.origin.position.y,
            self.map_info.origin.position.y + self.map_info.height * self.map_info.resolution
        )
        return (x, y)
    
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
        
        lookahead_weights = [1.0, 0.5, 0.25]
        f_att = (0.0, 0.0)

        for i, w in enumerate(lookahead_weights):
            idx = self.current_waypoint_idx + i
            if idx >= len(self.waypoints):
                break
            wp_x, wp_y = self.waypoints[idx]
            fa = self.attractive_force(wp_x, wp_y)
            f_att = (f_att[0] + w * fa[0], f_att[1] + w * fa[1])
        
        f_rep = self.repulsive_force()
        f_rep_drone = self.repulsive_force_other_drone()

        fx = f_att[0] + f_rep[0] + f_rep_drone[0]
        fy = f_att[1] + f_rep[1] + f_rep_drone[1]
        
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
            self.escape_cycles_remaining = 8
            att_angle = math.atan2(f_att[1], f_att[0])
            perp = att_angle + random.choice([math.pi / 2, -math.pi / 2])
            self.escape_vec = (
                self.k_att * math.cos(perp),
                self.k_att * math.sin(perp),
            )
            self.get_logger().warn("Local minimum detected — injecting escape perturbation")
        
        if self.escape_cycles_remaining > 0:
            self.escape_cycles_remaining -= 1
            fx += self.escape_vec[0]
            fy += self.escape_vec[1]
            force_mag = math.hypot(fx, fy)

        if force_mag < 1e-4:
            self.publish_zero_velocity()
            return
        
        force_mag = math.hypot(fx, fy)
        desired_speed = self.max_speed * math.tanh(force_mag / self.k_att)

        alpha = math.atan2(fy, fx)
        yaw = self.current_yaw

        vx_body =  (math.cos(yaw) * fx/force_mag + math.sin(yaw) * fy/force_mag)*desired_speed
        vy_body = (-math.sin(yaw) * fx/force_mag + math.cos(yaw) * fy/force_mag)*desired_speed
        
        yaw_error = self.normalize_angle(alpha - self.current_yaw)

        MAX_YAW_RATE = 1.0  
        yaw_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, self.alignment_yaw_gain * yaw_error))

        vx_body *= max(0.0, math.cos(yaw_error))

        if self.check_if_stuck():
            self.get_logger().warn("Stuck — clearing waypoints to force replan")
            self.waypoints = []
            self.publish_zero_velocity()
            return
        
        self._publish_apf_viz()   
        self.publish_velocity(vx_body, vy_body, yaw_rate)
    
    def check_if_stuck(self):
        """Detect if robot is stuck (not making progress)"""
        current_time = self.get_clock().now()
        if self._last_replan_time is not None:
            elapsed = (self.get_clock().now() - self._last_replan_time).nanoseconds / 1e9
            if elapsed < 3.0:
                return False
        # Initialize on first call
        if self.last_stuck_check_time is None:
            self.last_stuck_check_time = current_time
            self.last_position = (self.current_x, self.current_y)
            return False
        
        # Check periodically
        time_elapsed = (current_time - self.last_stuck_check_time).nanoseconds / 1e9
        
        if time_elapsed >= self.stuck_time_threshold:
            # Calculate distance traveled
            distance_traveled = math.hypot(
                self.current_x - self.last_position[0],
                self.current_y - self.last_position[1]
            )
            
            # Update for next check
            self.last_stuck_check_time = current_time
            self.last_position = (self.current_x, self.current_y)
            
            # Check if stuck
            if distance_traveled < self.stuck_distance_threshold:
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
            msg.poses.append(p)

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            waypoints_msg.poses.append(pose)
        
        self.path_pub.publish(msg)
        self.waypoints_pub.publish(waypoints_msg)
    
    def _publish_apf_viz(self):
        VIZ_SCALE = 0.5
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        force_list = [
            (0, "att",       0.1, 0.9, 0.1),
            (1, "rep_obs",   0.9, 0.1, 0.1),
            (2, "rep_drone", 1.0, 0.5, 0.0),
            (3, "total",     1.0, 1.0, 1.0),
        ]

        for mid, key, r, g, b in force_list:
            dx, dy = self._last_apf_forces[key]
            mag = math.hypot(dx, dy)
            if mag < 1e-4:
                continue

            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp    = stamp
            m.ns              = "apf_forces"
            m.id              = mid
            m.type            = Marker.ARROW
            m.action          = Marker.ADD
            m.lifetime.nanosec = int(0.5e9)

            tail = Point(x=self.current_x, y=self.current_y, z=0.05)
            tip  = Point(
                x=self.current_x + (dx / mag) * mag * VIZ_SCALE,
                y=self.current_y + (dy / mag) * mag * VIZ_SCALE,
                z=0.05
            )
            m.points  = [tail, tip]
            m.scale.x = 0.05
            m.scale.y = 0.12
            m.scale.z = 0.0
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 1.0
            markers.markers.append(m)

        self.apf_markers_pub.publish(markers)

def main():
    rclpy.init()
    node = AutonomousExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
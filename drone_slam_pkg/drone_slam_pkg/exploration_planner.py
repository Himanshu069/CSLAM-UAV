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
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
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
            ("exploration_complete_threshold", 0.95),  # 95% map known
            
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
            ("influence_radius", 1.5),
            ("pf_update_rate", 10.0),        # Hz
            ("max_speed", 1.0),
            ("waypoint_threshold", 0.5),

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
        
        #Altitude
        self.takeoff_altitude = 1.3  # meters
        self.altitude_ready = False
        self.current_z = 0.0

        # Frontier params
        self.min_frontier_size = self.get_parameter("min_frontier_size").value
        self.frontier_rate = self.get_parameter("frontier_search_rate").value
        self.exploration_threshold = self.get_parameter("exploration_complete_threshold").value
        
        # RRT* params
        self.rrt_max_iter = self.get_parameter("rrt_max_iter").value
        self.rrt_step_size = self.get_parameter("rrt_step_size").value
        self.rrt_goal_rate = self.get_parameter("rrt_goal_sample_rate").value
        self.rrt_radius = self.get_parameter("rrt_search_radius").value
        self.rrt_replan_rate = self.get_parameter("rrt_replan_rate").value
        
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
        self.motion_state = "FORWARD"  # "FORWARD" or "ALIGNING"
        self.last_position = (0.0, 0.0)
        self.last_stuck_check_time = None
        self.alignment_start_time = None
        self.max_alignment_time = 3.0  # seconds

        # Path following
        self.waypoints = []
        self.current_waypoint_idx = 0
        
        # Frontier tracking
        self.visited_frontiers = []  # Avoid revisiting
        self.exploration_complete = False
        
        # QoS
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
        
        # Subscribers
        self.create_subscription(OccupancyGrid, "map", self.map_cb, qos_map)
        self.create_subscription(VehicleLocalPosition, "fmu/out/vehicle_local_position", 
                                 self.pos_cb, qos_px4)
        self.create_subscription(VehicleAttitude, "fmu/out/vehicle_attitude", 
                                self.attitude_cb, qos_px4)
        self.create_subscription(PoseStamped, "goal_pose", self.goal_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.other_drone_pose_topic,
                                 self.other_drone_pose_cb, qos_map)
        # Publishers
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "global_path", 10)
        self.frontier_pub = self.create_publisher(MarkerArray, "frontiers", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "current_goal", 10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, "inflated_map", qos_map)

        # Timers
        self.frontier_timer = self.create_timer(1.0 / self.frontier_rate, self.frontier_search_loop)
        self.rrt_timer = self.create_timer(1.0 / self.rrt_replan_rate, self.rrt_replan)
        self.pf_timer = self.create_timer(1.0 / self.pf_rate, self.potential_field_control)
        # self.test_pose_timer = self.create_timer(0.1, self.fake_pose)
        
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
    
    def pos_cb(self, msg):
        self.current_x = msg.y
        self.current_y = msg.x
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
    
    def attitude_cb(self, msg):
    # PX4 Quaternions are typically [w, x, y, z]
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw_ned = math.atan2(siny_cosp, cosy_cosp)     
        self.current_yaw = self.normalize_angle(math.pi / 2.0 - yaw_ned)    
    
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
        self._publish_inflated_map(msg)
    
    def goal_cb(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.current_waypoint_idx = 0
        self.autonomous_mode = False # Disable auto-exploration
        self.motion_state = "FORWARD"
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
        cell_radius = math.ceil(self.drone_radius / resolution)

        y_idx, x_idx = np.ogrid[-cell_radius:cell_radius + 1,
                                 -cell_radius:cell_radius + 1]
        struct = (x_idx ** 2 + y_idx ** 2) <= cell_radius ** 2

        obstacle_mask = (raw >= 50)
        inflated_mask = binary_dilation(obstacle_mask, structure=struct)

        inflated = raw.copy()
        newly_occupied = inflated_mask & ~obstacle_mask
        inflated[newly_occupied] = 100

        self.get_logger().debug(
            f"Map inflated: {int(np.sum(newly_occupied))} cells added "
            f"(drone_radius={self.drone_radius:.2f} m, kernel={cell_radius} px)"
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
            self.motion_state = "FORWARD" 
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
            
            # Check if already visited
            if self.is_visited((centroid_x, centroid_y)):
                continue
            
            # Size score (larger = more info gain)
            size_score = len(cluster)
            
            # Distance score (closer = better)
            distance = math.hypot(centroid_x - self.current_x, centroid_y - self.current_y)
            distance_score = 1.0 / (distance + 0.1)  # Avoid division by zero
            
            # Combined score
            score = (self.w_size * size_score) + (self.w_dist * distance_score * 100)
            
            if score > best_score:
                best_score = score
                best_centroid = (centroid_x, centroid_y)
        
        # Mark as visited
        if best_centroid:
            self.visited_frontiers.append(best_centroid)
        
        return best_centroid
    
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
            self.waypoints = path
            self.current_waypoint_idx = 0
            self.get_logger().info(f"Path found: {len(path)} waypoints")
            self.publish_path_viz(path)
        else:
            self.get_logger().warn(" RRT* failed - will try again")
    
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
        
        # Check if reached current waypoint
        dist = math.hypot(target_x - self.current_x, target_y - self.current_y)
        if dist < self.wp_threshold:
            self.current_waypoint_idx += 1
            self.motion_state = "FORWARD" 
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info(" Reached final waypoint")
                self.publish_zero_velocity()
                return
            target_x, target_y = self.waypoints[self.current_waypoint_idx]
        
        # Compute forces
        f_att = self.attractive_force(target_x, target_y)
        f_rep = self.repulsive_force()
        f_rep_drone = self.repulsive_force_other_drone()
        # Combine
        fx = f_att[0] + f_rep[0] + f_rep_drone[0]
        fy = f_att[1] + f_rep[1] + f_rep_drone[1]

        desired_yaw = math.atan2(fy, fx)
        yaw_error = self.normalize_angle(desired_yaw - self.current_yaw)

        vx, vy, yaw_rate = 0.0, 0.0, 0.0
        
        ENTER_FORWARD_THRESHOLD = math.radians(20.0)   # Start moving once within 20°
        EXIT_FORWARD_THRESHOLD  = math.radians(45.0)
        MAX_YAW_RATE = 1.0  # rad/s cap to prevent oscillation
        
        if self.motion_state == "ALIGNING":
            if abs(yaw_error) < ENTER_FORWARD_THRESHOLD:
                self.motion_state = "FORWARD"
        elif self.motion_state == "FORWARD":
            if abs(yaw_error) > EXIT_FORWARD_THRESHOLD:
                self.motion_state = "ALIGNING"

        if self.motion_state == "ALIGNING":
            vx = 0.0
            vy = 0.0
            # Cap yaw rate to avoid overshoot
            yaw_rate = max(-MAX_YAW_RATE, min(MAX_YAW_RATE,
                        self.alignment_yaw_gain * yaw_error))
            self.get_logger().info(
                f"ALIGNING: yaw_error={math.degrees(yaw_error):.1f}°",
                throttle_duration_sec=1.0)
        else:
            # FORWARD: move and maintain heading
            speed = min(math.hypot(fx, fy), self.max_speed)
            vx = speed * math.cos(self.current_yaw)
            vy = speed * math.sin(self.current_yaw)
            yaw_rate = max(-MAX_YAW_RATE * 0.5, min(MAX_YAW_RATE * 0.5,
                        self.forward_yaw_gain * yaw_error))

        # 5. Final Stuck Check / Safety
        if self.check_if_stuck():
            if self.motion_state == "ALIGNING":
                self.last_stuck_check_time = self.get_clock().now()
                self.last_position = (self.current_x, self.current_y)
                return False
            
            self.get_logger().warn("Robot stuck! Clearing waypoints to force replan.")
            self.waypoints = []
            self.publish_zero_velocity()
            return
        
        self.publish_velocity(vx, vy, yaw_rate)
    
    def check_if_stuck(self):
        """Detect if robot is stuck (not making progress)"""
        current_time = self.get_clock().now()
        
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
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist = math.hypot(dx, dy)
        
        if dist < 0.01:
            return (0.0, 0.0)
        
        fx = self.k_att * (dx / dist)
        fy = self.k_att * (dy / dist)
        return (fx, fy)
    
    def repulsive_force(self):
        """Push away from obstacles"""
        fx_total, fy_total = 0.0, 0.0
        
        cells = self.get_nearby_cells(self.influence_radius)
        
        for (gx, gy) in cells:
            if self.inflated_map[gy, gx] >= 50:
                wx, wy = self.grid_to_world(gx, gy)
                
                dx = self.current_x - wx
                dy = self.current_y - wy
                dist = math.hypot(dx, dy)
                
                if dist < 0.01:
                    dist = 0.01
                
                if dist < self.influence_radius:
                    magnitude = self.k_rep * (1.0/dist - 1.0/self.influence_radius) / (dist**2)
                    fx_total += magnitude * (dx / dist)
                    fy_total += magnitude * (dy / dist)
        
        return (fx_total, fy_total)
    
    def repulsive_force_other_drone(self):
        if self.other_drone_x is None or self.other_drone_y is None:
            return (0.0, 0.0)

        dx = self.current_x - self.other_drone_x
        dy = self.current_y - self.other_drone_y
        dist = max(math.hypot(dx, dy), 0.01)

        if dist >= self.other_drone_safety_radius:
            return (0.0, 0.0)

        mag = (
            self.k_rep_drone
            * (1.0 / dist - 1.0 / self.other_drone_safety_radius)
            / (dist ** 2)
        )
        fx = mag * (dx / dist)
        fy = mag * (dy / dist)

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
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for (x, y) in path:
            p = PoseStamped()
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 2.0
            msg.poses.append(p)
        
        self.path_pub.publish(msg)

def main():
    rclpy.init()
    node = AutonomousExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
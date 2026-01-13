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
from geometry_msgs.msg import PoseStamped, Twist
from px4_msgs.msg import VehicleLocalPosition
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from scipy.ndimage import binary_dilation
import numpy as np
import math
import random
from scipy import ndimage
from collections import deque

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__("autonomous_explorer")
        
        # --- FRONTIER DETECTION PARAMETERS ---
        self.declare_parameters("", [
            ("min_frontier_size", 5),        # Min cells in frontier cluster
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
            ("k_rep", 40.0),
            ("influence_radius", 2.5),
            ("pf_update_rate", 10.0),        # Hz
            ("max_speed", 2.5),
            ("waypoint_threshold", 1.0),
            
            # --- EXPLORATION STRATEGY ---
            ("frontier_weight_size", 1.0),   # Prefer larger frontiers
            ("frontier_weight_distance", 0.5), # Prefer closer frontiers
        ])
        
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
        self.w_size = self.get_parameter("frontier_weight_size").value
        self.w_dist = self.get_parameter("frontier_weight_distance").value
        
        # State
        self.map_data = None
        self.map_info = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.goal_x = None
        self.goal_y = None
        self.autonomous_mode = True  # Auto-exploration enabled
        
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
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, qos_map)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", 
                                 self.pos_cb, qos_px4)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        
        # Publishers
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/global_path", 10)
        self.frontier_pub = self.create_publisher(MarkerArray, "/frontiers", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/current_goal", 10)
        
        # Timers
        self.frontier_timer = self.create_timer(1.0 / self.frontier_rate, self.frontier_search_loop)
        self.rrt_timer = self.create_timer(1.0 / self.rrt_replan_rate, self.rrt_replan)
        self.pf_timer = self.create_timer(1.0 / self.pf_rate, self.potential_field_control)
        self.test_pose_timer = self.create_timer(0.1, self.fake_pose)
        
        self.get_logger().info(" AUTONOMOUS EXPLORER ONLINE!")
        self.get_logger().info("   Frontier Detection: ON")
        self.get_logger().info("   RRT* Planning: ON")
        self.get_logger().info("   Potential Field Control: ON")
    
    def pos_cb(self, msg):
        self.current_x = msg.y
        self.current_y = msg.x
    
    def map_cb(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
    
    def goal_cb(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.current_waypoint_idx = 0
        self.autonomous_mode = False  # Disable auto-exploration
        self.get_logger().info(f"Manual Goal Set: ({self.goal_x:.1f}, {self.goal_y:.1f})")
        self.get_logger().info(" Autonomous exploration PAUSED")
        self.rrt_replan()

    def fake_pose(self):
        # Slowly move in a circle
        t = self.get_clock().now().nanoseconds * 1e-9
        self.current_x = 5.0 * math.cos(0.2 * t)
        self.current_y = 5.0 * math.sin(0.2 * t)

    
    # ==================== FRONTIER DETECTION ====================
    
    def frontier_search_loop(self):
        if self.map_data is None:
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
        dilated_free = binary_dilation(free_cells, structure=np.ones((3,3)))
        
        # Frontier = dilated free cells that overlap with unknown
        frontier_mask = dilated_free & unknown_cells
        
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
        if self.map_data is None or self.goal_x is None:
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
        class Node:
            def __init__(self, x, y):
                self.x = x
                self.y = y
                self.parent = None
                self.cost = 0.0
        
        start_node = Node(start[0], start[1])
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
            new_node = Node(new_x, new_y)
            
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
                goal_node = Node(goal[0], goal[1])
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
        if self.map_data is None:
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
            
            # Treat unknown as traversable (optimistic)
            if self.map_data[gy, gx] >= 50:
                return False
        
        return True
    
    # ==================== POTENTIAL FIELD LOCAL CONTROLLER ====================
    
    def potential_field_control(self):
        """Execute motion toward current waypoint using potential fields"""
        if not self.waypoints or self.map_data is None:
            return
        
        if self.current_waypoint_idx >= len(self.waypoints):
            self.publish_zero_velocity()
            return
        
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        
        # Check if reached current waypoint
        dist = math.hypot(target_x - self.current_x, target_y - self.current_y)
        if dist < self.wp_threshold:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info(" Reached final waypoint")
                self.publish_zero_velocity()
                return
            target_x, target_y = self.waypoints[self.current_waypoint_idx]
        
        # Compute forces
        f_att = self.attractive_force(target_x, target_y)
        f_rep = self.repulsive_force()
        
        # Combine
        fx = f_att[0] + f_rep[0]
        fy = f_att[1] + f_rep[1]
        
        # Limit speed
        speed = math.hypot(fx, fy)
        if speed > self.max_speed:
            fx = (fx / speed) * self.max_speed
            fy = (fy / speed) * self.max_speed
        
        self.publish_velocity(fx, fy)
    
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
            if self.map_data[gy, gx] >= 50:
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
        wx = (gx * self.map_info.resolution) + self.map_info.origin.position.x
        wy = (gy * self.map_info.resolution) + self.map_info.origin.position.y
        return wx, wy
    
    def publish_velocity(self, vx, vy):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
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
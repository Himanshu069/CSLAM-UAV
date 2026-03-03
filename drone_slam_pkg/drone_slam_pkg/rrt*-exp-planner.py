#!/usr/bin/env python3
"""
RRT* Global Planner Node — Nav2 + Costmap Aware
================================================
Standalone node that:
  1. Reads the inflated global costmap from Nav2
  2. Runs RRT* on it (obstacle + inflation aware)
  3. Publishes the path to Nav2's controller server via FollowPath action
  4. Accepts goals from /goal_pose or NavigateToPose action

Costmap value reference:
  0         = free
  1–252     = inflated (cost gradient, traversable but costly)
  253       = inscribed radius (avoid — drone center would touch obstacle)
  254       = lethal obstacle
  255       = unknown (treated as obstacle for safety)

ROS2 Topics:
  SUB  /global_costmap/costmap        (nav2_msgs/Costmap or OccupancyGrid)
  SUB  /goal_pose                     (geometry_msgs/PoseStamped)
  SUB  /fmu/out/vehicle_local_position (px4_msgs/VehicleLocalPosition)
  PUB  /rrt_star/path                 (nav_msgs/Path)         — viz
  PUB  /rrt_star/tree                 (visualization_msgs/MarkerArray) — viz
  ACT  /follow_path                   (nav2_msgs/FollowPath)  — sends to controller
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped, Point
from px4_msgs.msg import VehicleLocalPosition
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration

import numpy as np
import math
import random
import time
from typing import Optional, List, Tuple


# Costmap thresholds
COSTMAP_FREE       = 0
COSTMAP_UNKNOWN    = 255
COSTMAP_INSCRIBED  = 253  
COSTMAP_LETHAL     = 254
COSTMAP_SAFE_MAX   = 252   


class RRTNode:
    __slots__ = ("x", "y", "parent", "cost")

    def __init__(self, x: float, y: float):
        self.x: float = x
        self.y: float = y
        self.parent: Optional["RRTNode"] = None
        self.cost: float = 0.0


class RRTStarPlannerNode(Node):
    def __init__(self):
        super().__init__("rrt_star_planner")

        # Parameters
        self.declare_parameters("", [
            # RRT* core
            ("max_iterations",      5000),
            ("step_size",           0.5),    # metres per extension step
            ("goal_sample_rate",    0.10),   # fraction of samples biased to goal
            ("search_radius",       2.0),    # metres — RRT* rewire radius
            ("goal_tolerance",      0.4),    # metres — close enough to goal
            ("safe_cost_threshold", 252),    # costmap cells <= this are traversable

            # Costmap collision check
            ("collision_check_resolution", 0.1),  # metres between collision checks

            # Path smoothing
            ("smooth_path",         True),
            ("smooth_iterations",   50),

            # Replanning
            ("replan_on_new_goal",  True),

            # Viz
            ("publish_tree",        True),   # publish RRT tree markers (expensive)
        ])

        self.max_iter       = self.get_parameter("max_iterations").value
        self.step_size      = self.get_parameter("step_size").value
        self.goal_rate      = self.get_parameter("goal_sample_rate").value
        self.search_radius  = self.get_parameter("search_radius").value
        self.goal_tol       = self.get_parameter("goal_tolerance").value
        self.safe_thresh    = self.get_parameter("safe_cost_threshold").value
        self.cc_res         = self.get_parameter("collision_check_resolution").value
        self.smooth         = self.get_parameter("smooth_path").value
        self.smooth_iter    = self.get_parameter("smooth_iterations").value
        self.pub_tree       = self.get_parameter("publish_tree").value

        # State
        self.costmap_data: Optional[np.ndarray] = None
        self.costmap_info = None           
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.planning    = False

        # QoS profiles
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
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.costmap_cb,
            qos_map,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "fmu/out/vehicle_local_position",
            self.pos_cb,
            qos_px4,
        )
        self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_cb,
            10,
        )

        # Publishers
        self.path_pub = self.create_publisher(Path,       "/rrt_star/path", 10)
        self.tree_pub = self.create_publisher(MarkerArray, "/rrt_star/tree", 10)

        self._follow_path_client = ActionClient(self, FollowPath, "/follow_path")

        self.get_logger().info("RRT* Planner Node online.")
        self.get_logger().info("  Waiting for /global_costmap/costmap ...")

    # Callbacks
    def pos_cb(self, msg: VehicleLocalPosition):
        self.current_x = msg.y
        self.current_y = msg.x

    def costmap_cb(self, msg: OccupancyGrid):

        self.costmap_info = msg.info
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        self.costmap_data = raw.reshape((msg.info.height, msg.info.width))

        if not hasattr(self, "_costmap_logged"):
            unique, counts = np.unique(self.costmap_data, return_counts=True)
            self.get_logger().info(
                f"Costmap received: {msg.info.width}x{msg.info.height} "
                f"@ {msg.info.resolution:.2f}m/cell"
            )
            self.get_logger().info(
                f"  Value distribution: {dict(zip(unique[:6].tolist(), counts[:6].tolist()))}"
            )
            self._costmap_logged = True

    def goal_cb(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(f"Goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})")
        self._run_planner()

    # Planning entry point

    def _run_planner(self):
        if self.costmap_data is None:
            self.get_logger().warn("No costmap yet, cannot plan.")
            return
        if self.goal_x is None:
            return
        if self.planning:
            self.get_logger().warn("Already planning , ignoring new request.")
            return

        self.planning = True
        start = (self.current_x, self.current_y)
        goal  = (self.goal_x, self.goal_y)

        self.get_logger().info(
            f"RRT* planning: {start} to {goal}  (max_iter={self.max_iter})"
        )

        t0   = time.time()
        path = self.rrt_star(start, goal)
        dt   = time.time() - t0

        if path is None:
            self.get_logger().error(
                f"RRT* failed after {self.max_iter} iterations ({dt:.2f}s)"
            )
            self.planning = False
            return

        self.get_logger().info(
            f"RRT* found path: {len(path)} waypoints in {dt:.2f}s"
        )

        if self.smooth:
            path = self.smooth_path(path)
            self.get_logger().info(f"  After smoothing: {len(path)} waypoints")

        self._publish_path_viz(path)
        self._send_path_to_controller(path)
        self.planning = False

    # RRT* 

    def rrt_star(
        self,
        start: Tuple[float, float],
        goal:  Tuple[float, float],
    ) -> Optional[List[Tuple[float, float]]]:

        # Validate start and goal on costmap
        if not self._is_free_world(start[0], start[1]):
            self.get_logger().error("Start position is in collision on costmap!")
            return None
        if not self._is_free_world(goal[0], goal[1]):
            self.get_logger().error("Goal position is in collision on costmap!")
            return None

        start_node = RRTNode(start[0], start[1])
        nodes: List[RRTNode] = [start_node]

        best_goal_node: Optional[RRTNode] = None

        for i in range(self.max_iter):

            # --- Sample ---
            if random.random() < self.goal_rate:
                rx, ry = goal
            else:
                rx, ry = self._sample_free()

            # --- Nearest ---
            nearest = min(nodes, key=lambda n: math.hypot(n.x - rx, n.y - ry))

            # --- Steer ---
            nx, ny = self._steer(nearest.x, nearest.y, rx, ry)

            # --- Collision check ---
            if not self._is_segment_free(nearest.x, nearest.y, nx, ny):
                continue

            new_node = RRTNode(nx, ny)

            # --- Near neighbours ---
            near_nodes = [
                n for n in nodes
                if math.hypot(n.x - nx, n.y - ny) < self.search_radius
            ]

            # --- Choose best parent ---
            best_cost   = nearest.cost + math.hypot(nx - nearest.x, ny - nearest.y)
            best_parent = nearest

            for near in near_nodes:
                c = near.cost + math.hypot(nx - near.x, ny - near.y)
                if c < best_cost and self._is_segment_free(near.x, near.y, nx, ny):
                    best_cost   = c
                    best_parent = near

            new_node.parent = best_parent
            new_node.cost   = best_cost
            nodes.append(new_node)

            # --- Rewire ---
            for near in near_nodes:
                c = new_node.cost + math.hypot(near.x - nx, near.y - ny)
                if c < near.cost and self._is_segment_free(nx, ny, near.x, near.y):
                    near.parent = new_node
                    near.cost   = c

            # --- Goal check ---
            if math.hypot(nx - goal[0], ny - goal[1]) < self.goal_tol:
                if best_goal_node is None or new_node.cost < best_goal_node.cost:
                    goal_node = RRTNode(goal[0], goal[1])
                    goal_node.parent = new_node
                    goal_node.cost   = new_node.cost + math.hypot(
                        goal[0] - nx, goal[1] - ny
                    )
                    best_goal_node = goal_node
                    self.get_logger().info(
                        f"  Path found at iter {i}, cost={goal_node.cost:.2f}"
                    )

        if self.pub_tree:
            self._publish_tree_viz(nodes)

        if best_goal_node is None:
            return None

        # Extract path
        path = []
        node = best_goal_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path

    # -----------------------------------------------------------------------
    # Path Smoothing — greedy shortcutting
    # -----------------------------------------------------------------------

    def smooth_path(
        self, path: List[Tuple[float, float]]
    ) -> List[Tuple[float, float]]:
        """
        Greedy shortcut smoothing:
        Try to connect non-adjacent waypoints directly.
        Keeps the path collision-free on the costmap.
        """
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        i = 0

        while i < len(path) - 1:
            # Try to reach as far ahead as possible
            j = len(path) - 1
            while j > i + 1:
                if self._is_segment_free(
                    path[i][0], path[i][1],
                    path[j][0], path[j][1],
                ):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j

        return smoothed

    # -----------------------------------------------------------------------
    # Costmap helpers
    # -----------------------------------------------------------------------

    def _is_free_world(self, wx: float, wy: float) -> bool:
        gx, gy = self._world_to_grid(wx, wy)
        if gx is None:
            return False
        cost = int(self.costmap_data[gy, gx])
        return cost <= self.safe_thresh

    def _is_segment_free(
        self, x1: float, y1: float, x2: float, y2: float
    ) -> bool:
        """Walk along segment at self.cc_res intervals and check costmap."""
        dist  = math.hypot(x2 - x1, y2 - y1)
        steps = max(int(dist / self.cc_res), 1)

        for k in range(steps + 1):
            t  = k / steps
            wx = x1 + t * (x2 - x1)
            wy = y1 + t * (y2 - y1)
            if not self._is_free_world(wx, wy):
                return False
        return True

    def _world_to_grid(
        self, wx: float, wy: float
    ) -> Tuple[Optional[int], Optional[int]]:
        if self.costmap_info is None:
            return None, None
        gx = int((wx - self.costmap_info.origin.position.x) / self.costmap_info.resolution)
        gy = int((wy - self.costmap_info.origin.position.y) / self.costmap_info.resolution)
        if 0 <= gx < self.costmap_info.width and 0 <= gy < self.costmap_info.height:
            return gx, gy
        return None, None

    def _grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        wx = gx * self.costmap_info.resolution + self.costmap_info.origin.position.x
        wy = gy * self.costmap_info.resolution + self.costmap_info.origin.position.y
        return wx, wy

    def _sample_free(self) -> Tuple[float, float]:
        """Uniform random sample from costmap bounds, biased to free cells."""
        if self.costmap_info is None:
            return (0.0, 0.0)

        # Try random sampling, reject if not free (rejection sampling)
        for _ in range(50):
            wx = random.uniform(
                self.costmap_info.origin.position.x,
                self.costmap_info.origin.position.x
                + self.costmap_info.width * self.costmap_info.resolution,
            )
            wy = random.uniform(
                self.costmap_info.origin.position.y,
                self.costmap_info.origin.position.y
                + self.costmap_info.height * self.costmap_info.resolution,
            )
            if self._is_free_world(wx, wy):
                return (wx, wy)

        # Fallback: return random point regardless (RRT* will reject via collision check)
        return (wx, wy)

    def _steer(
        self, fx: float, fy: float, tx: float, ty: float
    ) -> Tuple[float, float]:
        dx   = tx - fx
        dy   = ty - fy
        dist = math.hypot(dx, dy)
        if dist < self.step_size:
            return (tx, ty)
        ratio = self.step_size / dist
        return (fx + dx * ratio, fy + dy * ratio)

    # -----------------------------------------------------------------------
    # Send path to Nav2 controller server
    # -----------------------------------------------------------------------

    def _send_path_to_controller(self, path: List[Tuple[float, float]]):
        """
        Send computed path to Nav2's /follow_path action server.
        Nav2's controller (DWB/TEB) will execute it safely on the local costmap.
        """
        if not self._follow_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                "/follow_path action server not available — "
                "is nav2_controller_server running?"
            )
            return

        nav_path = self._build_nav_path(path)

        goal_msg = FollowPath.Goal()
        goal_msg.path       = nav_path
        goal_msg.controller_id = ""  # use default controller

        self.get_logger().info("Sending path to Nav2 controller server ...")

        future = self._follow_path_client.send_goal_async(
            goal_msg,
            feedback_callback=self._follow_feedback_cb,
        )
        future.add_done_callback(self._follow_response_cb)

    def _follow_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("FollowPath goal rejected by controller server!")
            return
        self.get_logger().info("Controller server accepted path.")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._follow_result_cb)

    def _follow_result_cb(self, future):
        result = future.result()
        self.get_logger().info(f"FollowPath finished, status={result.status}")

    def _follow_feedback_cb(self, feedback_msg):
        # feedback_msg.feedback.speed, .distance_to_goal, etc.
        pass

    # -----------------------------------------------------------------------
    # Visualization
    # -----------------------------------------------------------------------

    def _build_nav_path(self, path: List[Tuple[float, float]]) -> Path:
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp    = self.get_clock().now().to_msg()
        for (x, y) in path:
            ps = PoseStamped()
            ps.header          = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def _publish_path_viz(self, path: List[Tuple[float, float]]):
        msg = self._build_nav_path(path)
        self.path_pub.publish(msg)

    def _publish_tree_viz(self, nodes: List[RRTNode]):
        """Publish RRT tree as line list marker for RViz."""
        ma = MarkerArray()

        # Delete old markers first
        delete = Marker()
        delete.action = Marker.DELETEALL
        ma.markers.append(delete)

        if len(nodes) < 2:
            self.tree_pub.publish(ma)
            return

        edges = Marker()
        edges.header.frame_id = "map"
        edges.header.stamp    = self.get_clock().now().to_msg()
        edges.ns              = "rrt_tree"
        edges.id              = 1
        edges.type            = Marker.LINE_LIST
        edges.action          = Marker.ADD
        edges.scale.x         = 0.02   # line width metres
        edges.color           = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.5)

        for node in nodes:
            if node.parent is not None:
                p1 = Point(x=node.parent.x, y=node.parent.y, z=0.5)
                p2 = Point(x=node.x,        y=node.y,        z=0.5)
                edges.points.extend([p1, p2])

        ma.markers.append(edges)

        # Path waypoints as spheres
        waypoints_marker = Marker()
        waypoints_marker.header       = edges.header
        waypoints_marker.ns           = "rrt_nodes"
        waypoints_marker.id           = 2
        waypoints_marker.type         = Marker.SPHERE_LIST
        waypoints_marker.action       = Marker.ADD
        waypoints_marker.scale.x      = 0.08
        waypoints_marker.scale.y      = 0.08
        waypoints_marker.scale.z      = 0.08
        waypoints_marker.color        = ColorRGBA(r=0.0, g=0.8, b=0.4, a=0.7)

        for node in nodes:
            waypoints_marker.points.append(
                Point(x=node.x, y=node.y, z=0.5)
            )

        ma.markers.append(waypoints_marker)
        self.tree_pub.publish(ma)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = RRTStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
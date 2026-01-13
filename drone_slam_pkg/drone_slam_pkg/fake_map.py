#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class FakeMap(Node):
    def __init__(self):
        super().__init__("fake_map")
        self.pub = self.create_publisher(OccupancyGrid, "/map", 1)
        self.timer = self.create_timer(1.0, self.publish_map)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"

        msg.info.resolution = 0.2
        msg.info.width = 100
        msg.info.height = 100
        msg.info.origin.position.x = -10.0
        msg.info.origin.position.y = -10.0

        grid = -1 * np.ones((100, 100), dtype=np.int8)

        # Known free space
        grid[20:80, 20:80] = 0

        # Obstacles
        grid[45:55, 30:70] = 100

        msg.data = grid.flatten().tolist()
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(FakeMap())
    rclpy.shutdown()

if __name__ == "__main__":
    main()

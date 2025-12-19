#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_drone_tf')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.declare_parameter('px4_ns', '')
        self.declare_parameter('vehicle_ns', 'x500_drone_0')

        px4_ns = self.get_parameter('px4_ns').get_parameter_value().string_value
        vehicle_ns = self.get_parameter('vehicle_ns').get_parameter_value().string_value

        qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
        )

        px4_topic = f'/{px4_ns}/fmu/out/vehicle_odometry' if px4_ns else '/fmu/out/vehicle_odometry'

        self.subscription = self.create_subscription(
            VehicleOdometry,
            px4_topic,
            self.odom_callback,
            qos_profile
        )

        self.odom_frame = f'{vehicle_ns}/odom'
        self.base_frame = f'{vehicle_ns}/base_link'

        self.get_logger().info(f"Subscribed to: {px4_topic}")
        self.get_logger().info(f"Odom frame: {self.odom_frame}")
        self.get_logger().info(f"Base frame: {self.base_frame}")

    def odom_callback(self, msg: VehicleOdometry):
        t = TransformStamped()
         # t.header.stamp = int(msg.timestamp // 1_000_000)
         # t.header.stamp.nanosec = int((msg.timestamp % 1_000_000)*1000)
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        # Position
        t.transform.translation.x = float(msg.position[0])
        t.transform.translation.y = float(msg.position[1])
        t.transform.translation.z = float(msg.position[2])

        # Orientation
        quat = [float(q) for q in msg.q]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


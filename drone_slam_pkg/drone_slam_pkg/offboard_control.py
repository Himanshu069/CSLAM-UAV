#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleStatus,
)

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from builtin_interfaces.msg import Time


class OffboardControl(Node):

    class ControlState:
        POSITION = 0
        VELOCITY = 1

    def __init__(self):
        super().__init__("offboard_control")

        self.use_sim_time_ = False
        self.declare_parameter("use_sim_time", False)
        self.use_sim_time_ = self.get_parameter("use_sim_time").value

        qos_best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )

        # Subscriptions
        self.twist_sub = self.create_subscription(
            Twist, "/cmd_vel", self.twist_callback, 10
        )

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.local_pos_callback,
            qos_best_effort,
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self.vehicle_status_callback,
            qos_best_effort,
        )

        self.ack_sub = self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self.vehicle_cmd_ack_callback,
            qos_best_effort,
        )

        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10
        )

        # State variables
        self.offboard_setpoint_counter = 0

        self.control_state = OffboardControl.ControlState.POSITION
        self.velocity2d = True

        self.twist = Twist()
        self.twist_stamp = self.get_clock().now()
        self.arming_stamp = self.get_clock().now()
        self.last_request = self.get_clock().now()

        self.local_pose = VehicleLocalPosition()
        self.current_goal = VehicleLocalPosition()

        self.current_goal.x = 0.0
        self.current_goal.y = 0.0
        self.current_goal.z = -1.3
        self.current_goal.heading = 0.0

        self.vehicle_status = VehicleStatus()

        # Timer (100 ms)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Offboard control node started")

    # ---------------- TIMER ---------------- #

    def timer_callback(self):

        if self.offboard_setpoint_counter == 10:
            self.get_logger().info("Setting offboard mode...")
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0
            )

        elif self.offboard_setpoint_counter == 20:
            if (
                not self.vehicle_status.pre_flight_checks_pass
                or self.vehicle_status.nav_state
                != VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self.offboard_setpoint_counter = 0
                self.get_logger().warn(
                    "Offboard not enabled, retrying..."
                )
            else:
                self.arm()

        if self.offboard_setpoint_counter >= 21:
            self.update_state()
        else:
            self.offboard_setpoint_counter += 1

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

    # ---------------- STATE MACHINE ---------------- #

    def update_state(self):
        now = self.get_clock().now()

        if (now - self.last_request).nanoseconds > 1e9:

            if (
                not self.vehicle_status.pre_flight_checks_pass
                or self.vehicle_status.nav_state
                != VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self.get_logger().error(
                    "Preflight checks failed or offboard disabled"
                )
                rclpy.shutdown()
                return

            if (now - self.twist_stamp).nanoseconds < 1e9:
                if (
                    self.vehicle_status.arming_state
                    == VehicleStatus.ARMING_STATE_DISARMED
                    and self.twist.linear.z < -0.4
                    and self.twist.angular.z < -0.4
                ):
                    self.arm()

                elif (
                    self.vehicle_status.arming_state
                    == VehicleStatus.ARMING_STATE_ARMED
                    and self.twist.linear.z < -0.4
                    and self.twist.angular.z > 0.4
                ):
                    self.disarm()

            self.last_request = now

    # ---------------- COMMANDS ---------------- #

    def arm(self):
        self.arming_stamp = self.get_clock().now()
        self.current_goal = self.local_pose
        self.current_goal.z -= 1.3

        self.get_logger().info("Arming vehicle")
        self.control_state = OffboardControl.ControlState.POSITION

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand.ARMING_ACTION_ARM,
        )

    def disarm(self):
        self.get_logger().info("Disarming vehicle")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand.ARMING_ACTION_DISARM,
        )

    # ---------------- PUBLISHERS ---------------- #

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):

        if (
            self.control_state != OffboardControl.ControlState.POSITION
            and abs(
                self.local_pose.timestamp * 1e-6
                - self.twist_stamp.nanoseconds * 1e-9
            )
            > 0.1
        ):
            self.control_state = OffboardControl.ControlState.POSITION
            self.current_goal = self.local_pose
            self.get_logger().info("Switching to position control")

        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        yaw = self.local_pose.heading
        cosy = math.cos(yaw)
        siny = math.sin(yaw)

        if self.control_state == OffboardControl.ControlState.VELOCITY:
            msg.velocity[0] = self.twist.linear.x * cosy + self.twist.linear.y * siny
            msg.velocity[1] = self.twist.linear.x * siny - self.twist.linear.y * cosy
            msg.velocity[2] = -self.twist.linear.z if not self.velocity2d else float("nan")
            msg.yawspeed = -self.twist.angular.z
        else:
            msg.velocity[:] = [float("nan")] * 3
            msg.yawspeed = float("nan")

        if self.control_state == OffboardControl.ControlState.POSITION:
            msg.position[0] = self.current_goal.x
            msg.position[1] = self.current_goal.y
            msg.position[2] = self.current_goal.z
            msg.yaw = self.current_goal.heading
        else:
            msg.position[:] = [float("nan")] * 3
            msg.yaw = float("nan")

        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_pub.publish(msg)

    # ---------------- CALLBACKS ---------------- #

    def twist_callback(self, msg):
        self.twist = msg
        self.twist_stamp = self.get_clock().now()

        if (
            (self.twist_stamp - self.arming_stamp).nanoseconds > 5e9
            and self.control_state == OffboardControl.ControlState.POSITION
        ):
            self.get_logger().info("Switching to velocity control")
            self.control_state = OffboardControl.ControlState.VELOCITY

    def local_pos_callback(self, msg):
        self.local_pose = msg
        if self.use_sim_time_:
            self.local_pose.timestamp = self.get_clock().now().nanoseconds // 1000

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def vehicle_cmd_ack_callback(self, msg):
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.get_logger().info("Command accepted")
        else:
            self.get_logger().error("Command rejected")

    def joy_callback(self, msg):
        self.velocity2d = not (msg.buttons[5] == 1)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

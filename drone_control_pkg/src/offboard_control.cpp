#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <Eigen/Eigen>

static const std::string kModeName = "Teleoperation";
static const bool kEnableDebugOutput = true;

static constexpr float kHeightToleranceM = 0.10f;

static constexpr float kTargetHeightM = 0.8f;

static constexpr float kClimbSpeedMps = 0.5f;

enum class FlightPhase {
  CLIMBING,
  VELOCITY_CONTROL
};

class VelocityControlMode : public px4_ros2::ModeBase
{
public:
  explicit VelocityControlMode(rclcpp::Node & node)
  : ModeBase(node, kModeName)
  , _node(node)
  , _phase(FlightPhase::CLIMBING)
  {
    RCLCPP_INFO(_node.get_logger(), "Mode '%s' registered with PX4!", kModeName.c_str());

    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_attitude    = std::make_shared<px4_ros2::OdometryAttitude>(*this);
    _vehicle_local_pos   = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _clock               = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    std::string ns = _node.get_namespace();
    if (ns == "/") ns = "";

    std::string cmd_vel_topic = ns.empty() ? "cmd_vel" : ns + "/cmd_vel";
    _cmd_vel_sub = node.create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Buffer commands always, but only act on them in VELOCITY_CONTROL phase
        _latest_cmd_vel = *msg;
        _last_cmd_time  = _clock->now();
        _has_cmd        = true;
        RCLCPP_INFO(_node.get_logger(),
          "Received Twist: linear=(%.2f, %.2f, %.2f), angular=(%.2f, %.2f, %.2f)",
          msg->linear.x, msg->linear.y, msg->linear.z,
          msg->angular.x, msg->angular.y, msg->angular.z);
      });
  }

  void onActivate() override
  {
    _takeoff_ned_z   = _vehicle_local_pos->positionNed().z();
    _target_ned_z    = _takeoff_ned_z - kTargetHeightM;   // more negative = higher
    _last_cmd_time   = _clock->now();
    _phase           = FlightPhase::CLIMBING;

    RCLCPP_INFO(node().get_logger(),
      "Activated. Climbing from NED Z=%.2f to NED Z=%.2f (%.1f m)",
      _takeoff_ned_z, _target_ned_z, kTargetHeightM);
  }

  void onDeactivate() override
  {
    _phase = FlightPhase::CLIMBING;   // reset for next activation
    RCLCPP_INFO(node().get_logger(), "Velocity control mode deactivated!");
  }

  void updateSetpoint(float dt_s) override
  {
    const float current_ned_z = _vehicle_local_pos->positionNed().z();

    if (_phase == FlightPhase::CLIMBING) {
      const float height_error = _target_ned_z - current_ned_z;  // negative means we need to go up

      if (std::abs(height_error) < kHeightToleranceM) {

        RCLCPP_INFO(node().get_logger(),
          "Target height reached (NED Z=%.2f). Switching to velocity control.", current_ned_z);
        _phase = FlightPhase::VELOCITY_CONTROL;

        _trajectory_setpoint->update(Eigen::Vector3f::Zero(), {}, {}, 0.0f);
        return;
      }

        float climb_vel_ned = std::max(-kClimbSpeedMps,
                              std::min(0.f, height_error / 1.0f));  // P gain = 1.0

        if (height_error < -kHeightToleranceM) {
        climb_vel_ned = -kClimbSpeedMps;
      }

      Eigen::Vector3f vel_ned(0.f, 0.f, climb_vel_ned);
      _trajectory_setpoint->update(vel_ned, {}, {}, 0.0f);  // hold yaw

      RCLCPP_INFO_THROTTLE(node().get_logger(), *_clock, 500,
        "Climbing... NED Z=%.2f, target=%.2f, climb_vel=%.2f",
        current_ned_z, _target_ned_z, climb_vel_ned);
      return;
    }

    //  velocity control
    if (!_has_cmd) {
      _trajectory_setpoint->update(Eigen::Vector3f::Zero(), {}, {}, std::nullopt);
      return;
    }

    const auto current_time = _clock->now();
    if ((current_time - _last_cmd_time).seconds() > 0.25) {
      // Stale command — hold
      _trajectory_setpoint->update(Eigen::Vector3f::Zero(), {}, {}, 0.0f);
      return;
    }

    Eigen::Vector3f vel_ned;
    vel_ned.x() =   _latest_cmd_vel.linear.y;
    vel_ned.y() =   _latest_cmd_vel.linear.x;
    vel_ned.z() = -vz_body;   // body-up → NED-down

    const float yaw_rate = -_latest_cmd_vel.angular.z;

    _trajectory_setpoint->update(vel_ned, {}, {}, yaw_rate);
  }

private:
  rclcpp::Node & _node;

  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  std::shared_ptr<px4_ros2::OdometryAttitude>        _vehicle_attitude;
  std::shared_ptr<px4_ros2::OdometryLocalPosition>   _vehicle_local_pos;
  std::shared_ptr<rclcpp::Clock>                     _clock;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
  geometry_msgs::msg::Twist _latest_cmd_vel;
  rclcpp::Time              _last_cmd_time;
  bool                      _has_cmd{false};

  FlightPhase _phase{FlightPhase::CLIMBING};
  float       _takeoff_ned_z{0.f};
  float       _target_ned_z{0.f};
};

using MyNodeWithMode = px4_ros2::NodeWithMode<VelocityControlMode>;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  static const std::string kNodeName = "velocity_control_node";
  auto node = std::make_shared<MyNodeWithMode>(kNodeName, kEnableDebugOutput);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
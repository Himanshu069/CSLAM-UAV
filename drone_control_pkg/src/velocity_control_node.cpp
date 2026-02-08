#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <Eigen/Eigen>

static const std::string kModeName = "VelocityControl";
static const bool kEnableDebug = true;

class VelocityControlMode : public px4_ros2::ModeBase
{
public:
  explicit VelocityControlMode(rclcpp::Node & node)
  : ModeBase(node, kModeName)
  , _node(node)
  {
    RCLCPP_INFO(_node.get_logger(), "Mode '%s' registered with PX4!", kModeName.c_str());
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
    _clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    // Subscribe to cmd_vel topicx
    std::string ns = _node.get_namespace();
    if (ns == "/") ns = "";  // root namespace

    std::string cmd_vel_topic =
        ns.empty() ? "cmd_vel" : ns + "/cmd_vel";
    _cmd_vel_sub = node.create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        _latest_cmd_vel = *msg;
        _last_cmd_time = _clock->now();
        _has_cmd = true;       
            RCLCPP_INFO(_node.get_logger(),
                    "Received Twist: linear=(%.2f, %.2f, %.2f), angular=(%.2f, %.2f, %.2f)",
                    msg->linear.x, msg->linear.y, msg->linear.z,
                    msg->angular.x, msg->angular.y, msg->angular.z);
      });
  }

  void onActivate() override
  {
    _last_cmd_time = _clock->now();
    RCLCPP_INFO(node().get_logger(), "Velocity control mode activated!");
  }

  void onDeactivate() override
  {
    RCLCPP_INFO(node().get_logger(), "Velocity control mode deactivated!");
  }

  void updateSetpoint(float) override
  {
    const auto current_time = _clock->now();
    Eigen::Vector3f velocity_body;
    std::optional<float> yaw_rate = std::nullopt;
    Eigen::Vector3f velocity_ned;
    if (!_has_cmd) {
        // No commands received yet, send zero velocity
        Eigen::Vector3f zero_velocity = Eigen::Vector3f::Zero();
        _trajectory_setpoint->update(
            zero_velocity,
            {},
            {},
            std::nullopt
        );
        return;
    }
    if ((current_time - _last_cmd_time).seconds()<=0.25) {
      // Convert Twist (body frame) to NED frame velocities
      // Assuming cmd_vel is in body frame :
      // linear.x = forward, linear.y = left, linear.z = up
      // angular.z = yaw rate
      float yaw = _vehicle_attitude->yaw(); 
      
      velocity_body.x() = _latest_cmd_vel.linear.x;  // forward
      velocity_body.y() = _latest_cmd_vel.linear.y;  // left
      velocity_body.z() = _latest_cmd_vel.linear.z; 
      
      // float yaw_rate = _latest_cmd_vel->angular.z;
      velocity_ned.x() = velocity_body.x() * std::cos(yaw) - velocity_body.y() * std::sin(yaw);
      velocity_ned.y() = velocity_body.x() * std::sin(yaw) + velocity_body.y() * std::cos(yaw);
      velocity_ned.z() = -velocity_body.z();  // body-up → NED-down
      // Send to PX4
      yaw_rate = -_latest_cmd_vel.angular.z;  
    } 
    else
    {
      _trajectory_setpoint->update(Eigen::Vector3f::Zero(), {}, {}, 0.0f);
      return;
    }
    _trajectory_setpoint->update(
      velocity_ned,
      {},  // No acceleration feedforward
      {},  // No yaw setpoint (use rate instead)
      yaw_rate
    );
    }
  

private:
  void loadParameters();
  rclcpp::Node &_node;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
  geometry_msgs::msg::Twist _latest_cmd_vel;
  std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
  std::shared_ptr<rclcpp::Clock> _clock;
  rclcpp::Time _last_cmd_time;
  bool _has_cmd = false;

};
// using MyNodeWithMode = px4_ros2::NodeWithMode<VelocityControlMode>;
// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);

//     static const std::string kNodeName = "velocity_control_node";
//     static const bool kEnableDebugOutput = true;
//     auto node = std::make_shared<MyNodeWithMode>(kNodeName, kEnableDebugOutput);
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
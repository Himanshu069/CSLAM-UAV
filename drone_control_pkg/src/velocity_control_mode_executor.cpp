#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include "velocity_control_node.cpp"  // include your existing VelocityControlMode header or cpp

class VelocityExecutor : public px4_ros2::ModeExecutorBase
{
public:
    VelocityExecutor(rclcpp::Node &node, px4_ros2::ModeBase &owned_mode)
    : ModeExecutorBase(node, Settings{}, owned_mode), _node(node) {}

    void onActivate() override {
        RCLCPP_INFO(_node.get_logger(), "VelocityExecutor activated");
        switchToState(State::Takeoff, px4_ros2::Result::Success);
    }

    void onDeactivate(DeactivateReason reason) override {
        const char *reason_str = (reason == DeactivateReason::FailsafeActivated) ? "failsafe" : "other";
        RCLCPP_INFO(_node.get_logger(), "VelocityExecutor deactivated: %s", reason_str);
    }

private:
    rclcpp::Node &_node;

    enum class State { Takeoff, VelocityControl, RTL, Land, WaitUntilDisarmed };
    State _state;

    void switchToState(State state, px4_ros2::Result previous_result) {
        _state = state;

        if (previous_result != px4_ros2::Result::Success) {
            RCLCPP_WARN(_node.get_logger(), "Previous state failed (%d), switching to state %d",
                        static_cast<int>(previous_result), static_cast<int>(state));
        }

        RCLCPP_INFO(_node.get_logger(), "Switched to state : %d", static_cast<int>(state));
        switch (state) {
            case State::Takeoff:
                RCLCPP_INFO(_node.get_logger(), "Takeoff to 1.3 m");
                takeoff([this](px4_ros2::Result r){ switchToState(State::VelocityControl, r); }, 2.3f);
                break;

            case State::VelocityControl:
                RCLCPP_INFO(_node.get_logger(), "Activating VelocityControlMode");
                scheduleMode(ownedMode().id(),
                             [this](px4_ros2::Result r){
                                 if (r == px4_ros2::Result::Success)
                                     switchToState(State::Land, r);
                                 else
                                     switchToState(State::RTL, r);
                             });
                break;

            case State::RTL:
                RCLCPP_INFO(_node.get_logger(), "Returning to Launch");
                rtl([this](px4_ros2::Result r){ switchToState(State::WaitUntilDisarmed, r); });
                break;

            case State::Land:
                RCLCPP_INFO(_node.get_logger(), "Landing");
                land([this](px4_ros2::Result r){ switchToState(State::WaitUntilDisarmed, r); });
                break;

            case State::WaitUntilDisarmed:
                RCLCPP_INFO(_node.get_logger(), "Waiting until disarmed");
                waitUntilDisarmed([this](px4_ros2::Result r){
                    RCLCPP_INFO(_node.get_logger(), "Mission complete (%s)", resultToString(r));
                });
                break;
        }
    }
};

// ------------------- Main -------------------
using VelocityNodeWithExecutor = px4_ros2::NodeWithModeExecutor<VelocityExecutor, VelocityControlMode>;
static const std::string kNodeName = "Velocity_control_node";
static const bool kEnableDebugOutput = true;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_with_executor = std::make_shared<VelocityNodeWithExecutor>(kNodeName, kEnableDebugOutput);
    rclcpp::spin(node_with_executor);
    rclcpp::shutdown();
    return 0;
}

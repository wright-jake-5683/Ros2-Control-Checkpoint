#ifndef rb1_lift_controller__rb1_lift_controller_HPP_
#define rb1_lift_controller__rb1_lift_controller_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace rb1_lift_controller 
{
    struct Command
    {
        std::vector<double> displacements;
        std::vector<double> velocities;
        uint64_t seq{0};
    };

    enum class Mode { POSITION, VELOCITY };

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class RB1Controller : public controller_interface::ControllerInterface 
    {
        public:
        RB1Controller();

        CallbackReturn 
        on_init() override;

        controller_interface::InterfaceConfiguration
        command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration
        state_interface_configuration() const override;

        CallbackReturn
        on_configure(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::return_type
        update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        protected:
        std::vector<std::string> joint_names_;
        std::string interface_name_;

        // Command subscribers and Controller State publisher
        using ControllerCommandMsg = control_msgs::msg::JointJog;

        rclcpp::Subscription<ControllerCommandMsg>::SharedPtr command_subscriber_ = nullptr;
        //realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandMsg>> input_command_;

        using ControllerStateMsg = control_msgs::msg::JointControllerState;
        using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

        rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
        std::unique_ptr<ControllerStatePublisher> state_publisher_;

        realtime_tools::RealtimeBuffer<Command> input_command_;   // was a buffer of shared_ptr<msg>
        Mode mode_{Mode::POSITION};
        std::vector<double> target_position_;
        uint64_t last_seq_{0};
        uint64_t next_seq_{0};            // only touched in the subscriber callback
        rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
        double command_timeout_{0.25};
        double position_min_{0.0};
        double position_max_{0.034};
    };

} // namespace rb1_lift_controller

#endif // rb1_lift_controller__rb1_lift_controller_HPP_
 
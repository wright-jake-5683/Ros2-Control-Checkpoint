#include "rb1_ros2_description/rb1_lift_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace rb1_lift_controller
{
    RB1Controller::RB1Controller() : controller_interface::ControllerInterface() 
    {}
        
    CallbackReturn RB1Controller::on_init() 
    {
        try 
        {
            auto_declare("joints", std::vector<std::string>());
            auto_declare("interface_name", std::string());
            auto_declare("command_timeout", 0.25);
            auto_declare("position_min", 0.0);
            auto_declare("position_max", 0.034);
        } 
        catch (const std::exception &e) 
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }
    

    CallbackReturn RB1Controller::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) 
    {
        auto error_if_empty = [&](const auto &parameter, const char *parameter_name) 
        {
            if (parameter.empty()) 
            {
                RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty", parameter_name);
                return true;
            }
            return false;
        };

        auto error_if_nan = [&](const auto &parameter, const char *parameter_name)
        {
            if (std::isnan(parameter))
            {
                RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was NaN", parameter_name);
                return true;
            }
            return false;
        };

        auto get_string_array_param_and_error_if_empty = [&](std::vector<std::string> &parameter, const char *parameter_name) 
        {
                parameter = get_node()->get_parameter(parameter_name).as_string_array();
                return error_if_empty(parameter, parameter_name);
        };

        auto get_string_param_and_error_if_empty = [&](std::string &parameter, const char *parameter_name) 
        {
            parameter = get_node()->get_parameter(parameter_name).as_string();
            return error_if_empty(parameter, parameter_name);
        };

        auto get_double_param_and_error_if_nan = [&](double &parameter, const char *parameter_name)
        {
            parameter = get_node()->get_parameter(parameter_name).as_double();
            return error_if_nan(parameter, parameter_name);
        };

        if 
        (
            get_string_array_param_and_error_if_empty(joint_names_, "joints") ||
            get_string_param_and_error_if_empty(interface_name_, "interface_name") ||
            get_double_param_and_error_if_nan(command_timeout_, "command_timeout") ||
            get_double_param_and_error_if_nan(position_min_, "position_min") ||
            get_double_param_and_error_if_nan(position_max_, "position_max")
        ) 
            {
                return CallbackReturn::ERROR;
            }

        // Command Subscriber and callbacks
        auto callback_command = [&](const std::shared_ptr<ControllerCommandMsg> msg) -> void
        {
            const size_t number_of_joints = joint_names_.size();
            const auto &msg_content = (mode_ == Mode::POSITION) ? msg->displacements : msg->velocities;
            
            if (msg->joint_names.size() != number_of_joints || msg_content.size() != number_of_joints)
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                    "Command has wrong array sizes for mode '%s'. Ignoring message.",
                    interface_name_.c_str());
                return;
            }

            Command cmd;                                  // copy happens in the non-RT thread
            cmd.displacements = msg->displacements;
            cmd.velocities    = msg->velocities;
            cmd.seq           = ++next_seq_;
            input_command_.writeFromNonRT(cmd);
        };

        command_subscriber_ = get_node()->create_subscription<ControllerCommandMsg>(
            "~/lift_command", 
            rclcpp::SystemDefaultsQoS(), 
            callback_command
        );

        // State publisher
        s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
            "~/state", 
            rclcpp::SystemDefaultsQoS()
        );
        state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);

        state_publisher_->lock();
        state_publisher_->msg_.header.frame_id = joint_names_[0];
        state_publisher_->unlock();

        RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }



    controller_interface::InterfaceConfiguration RB1Controller::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        command_interfaces_config.names.reserve(joint_names_.size());
        for (const auto &joint : joint_names_) 
        {
            command_interfaces_config.names.push_back(joint + "/" + interface_name_);
        }

        return command_interfaces_config;
    }


    controller_interface::InterfaceConfiguration RB1Controller::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        state_interfaces_config.names.reserve(joint_names_.size());
        for (const auto &joint : joint_names_) 
        {
            state_interfaces_config.names.push_back(joint + "/" + interface_name_);
        }

        return state_interfaces_config;
    }


    CallbackReturn RB1Controller::on_activate(const rclcpp_lifecycle::State &)
    {
        input_command_.writeFromNonRT(Command{});     // seq 0, empty arrays
        last_seq_ = 0;
        last_cmd_time_ = get_node()->now();

        target_position_.assign(joint_names_.size(), 0.0);
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            auto pos = state_interfaces_[i].get_optional();
            if (!pos.has_value())
            {
                RCLCPP_ERROR(get_node()->get_logger(), "No position state for joint %zu", i);
                return CallbackReturn::ERROR;
            }
            target_position_[i] = pos.value();        // start the target at the current position
        }
        return CallbackReturn::SUCCESS;
    }


    CallbackReturn RB1Controller::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }


    controller_interface::return_type
    RB1Controller::update(const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
    {
        // 1. Read the latest command (never blocks)
        const Command &cmd = *input_command_.readFromRT();

        // 2. Detect whether a message arrived since the last cycle
        const bool is_new = (cmd.seq != last_seq_);
        if (is_new)
        {
            last_seq_ = cmd.seq;
            last_cmd_time_ = time;
        }

        // What we report on the state topic (joint 0 only)
        double set_point = std::numeric_limits<double>::quiet_NaN();

        if (mode_ == Mode::POSITION)
        {
            // 3a. Position mode: displacements are relative steps
            for (size_t i = 0; i < command_interfaces_.size(); ++i)
            {
                // Apply each displacement exactly once, on the cycle the message first appears
                if (is_new && i < cmd.displacements.size() && !std::isnan(cmd.displacements[i]))
                {
                    target_position_[i] = std::clamp(
                        target_position_[i] + cmd.displacements[i],
                        position_min_, position_max_);
                }

                // Hold the target every cycle
                if (!command_interfaces_[i].set_value(target_position_[i]))
                {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                        "Failed to set position command for joint %zu", i);
                }
            }
            set_point = target_position_[0];
        }
        else
        {
            // 3b. Velocity mode: velocities are applied only while messages keep arriving
            const bool fresh = (time - last_cmd_time_).seconds() <= command_timeout_;

            for (size_t i = 0; i < command_interfaces_.size(); ++i)
            {
                double v = 0.0;   // default: stop
                if (fresh && i < cmd.velocities.size() && !std::isnan(cmd.velocities[i]))
                {
                    v = cmd.velocities[i];
                }

                if (!command_interfaces_[i].set_value(v))
                {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                        "Failed to set velocity command for joint %zu", i);
                }
                if (i == 0)
                {
                    set_point = v;
                }
            }
        }

        // 4. Publish state without blocking
        if (state_publisher_ && state_publisher_->trylock())
        {
            state_publisher_->msg_.header.stamp = time;
            state_publisher_->msg_.set_point = set_point;
            state_publisher_->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }

}  // namespace rb1_lift_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rb1_lift_controller::RB1Controller, controller_interface::ControllerInterface)
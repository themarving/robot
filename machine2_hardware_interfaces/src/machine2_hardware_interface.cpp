#include "machine2_hardware_interfaces/machine2_hardware_interface.hpp"
#include <string>
#include <vector>

// to use this interface as a plugin in the ros2 control library
#include "pluginlib/class_list_macros.hpp"

namespace machine_hardware_interface
{
    // SystemInterface::on_init(info) call fills out the info object with specifics from the URDF
    CallbackReturn MachineSystem::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // robot has 3 joints and 2 interfaces
        // initializing vectors with 0 value
        joint_position_.assign(3, 0);
        joint_position_command_.assign(3, 0);

        prev_joint_position_command_.assign(3, 1);

        joint_velocities_.assign(3, 0);
        joint_velocities_command_.assign(3, 0);

        for (const auto &joint : info_.joints)
        {
            for (const auto &interface : joint.state_interfaces)
            {
                joint_interfaces[interface.name].push_back(joint.name);
            }
        }

        // create node to publish angles
        node_ = std::make_shared<rclcpp::Node>("machine_angle_publishing_node");

        // create a publisher for sending arm angles
        angles_publisher_ = node_->create_publisher<custom_interfaces::msg::MachineArmAngles>("machine_arm_angles", 10);

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> MachineSystem::export_state_interfaces()
    {
        // vector of state interfaces
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // for each joint add state interface for position
        int i = 0;
        for (const auto &joint_name : joint_interfaces["position"])
        {
            state_interfaces.emplace_back(joint_name, "position", &joint_position_[i++]);
        }

        // for each joint add state interface for position
        i = 0;
        for (const auto &joint_name : joint_interfaces["velocity"])
        {
            state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[i++]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MachineSystem::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // for each joint add command interface for position
        int i = 0;
        for (const auto &joint_name : joint_interfaces["position"])
        {
            command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[i++]);
        }

        // for each joint add command interface for velocity
        i = 0;
        for (const auto &joint_name : joint_interfaces["velocity"])
        {
            command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[i++]);
        }

        return command_interfaces;
    }

    return_type MachineSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
        {
            joint_velocities_[i] = joint_velocities_command_[i];
            joint_position_[i] += joint_velocities_command_[i] * period.seconds();
        }

        for (auto i = 0ul; i < joint_position_command_.size(); i++)
        {
            joint_position_[i] = joint_position_command_[i];
        }

        return return_type::OK;
    }

    return_type MachineSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if (joint_position_command_ == prev_joint_position_command_)
        {
            return hardware_interface::return_type::OK;
        }

        try
        {
            // ROS2 angles = radians, Servo angles = degree
            int shoulder1Angle = static_cast<int>((joint_position_command_.at(0) * 180) / M_PI);
            // shoulder1Angle += 90; // Shift the range from [-90, 90] to [0, 180]

            int shoulder2Angle = static_cast<int>((joint_position_command_.at(1) * 180) / M_PI);
            // shoulder2Angle += 90; // Shift the range from [-90, 90] to [0, 180]

            int kneeAngle = static_cast<int>((joint_position_command_.at(2) * 180) / M_PI);
            // kneeAngle += 90; // Shift the range from [-90, 90] to [0, 180]

            custom_interfaces::msg::MachineArmAngles machine_arm_angles_msg;

            machine_arm_angles_msg.shoulder_one_angle = shoulder1Angle;
            machine_arm_angles_msg.shoulder_two_angle = shoulder2Angle;
            machine_arm_angles_msg.knee_angle = kneeAngle;

            angles_publisher_->publish(machine_arm_angles_msg);
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("MachineSystem"), "Could not send servo angles to hardware in MachineSystem::write");
            return hardware_interface::return_type::ERROR;
        }

        // updating prev joint positions to current ones
        prev_joint_position_command_ = joint_position_command_;

        return return_type::OK;
    }

}  // namespace machine_hardware_interface

// c++ macro creates a plugin library using pluginlib
PLUGINLIB_EXPORT_CLASS(machine_hardware_interface::MachineSystem, hardware_interface::SystemInterface)
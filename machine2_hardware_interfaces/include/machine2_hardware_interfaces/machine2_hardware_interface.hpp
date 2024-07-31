// guards against this class being redefined multiple times
#ifndef MACHINE_HARDWARE_INTERFACE_H
#define MACHINE_HARDWARE_INTERFACE_H

#include "string"
#include "unordered_map"
#include "vector"

#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// custom msg type for sending angles
#include <custom_interfaces/msg/machine_arm_angles.hpp>

using hardware_interface::return_type;

namespace machine_hardware_interface
{
    // return type of the functions
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class HARDWARE_INTERFACE_PUBLIC MachineSystem : public hardware_interface::SystemInterface
    {
    public:
        // called once during ros2_control initialization
        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    protected:
        // The size of this vector is (standard_interfaces_.size() x nr_joints)
        std::vector<double> prev_joint_position_command_;
        std::vector<double> joint_position_command_;
        std::vector<double> joint_position_; 

        std::vector<double> joint_velocities_command_;
        std::vector<double> joint_velocities_;

        std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
            {"position", {}}, {"velocity", {}}
        };

        // node for publishing arm angles
        std::shared_ptr<rclcpp::Node> node_;

        // publisher for sending arm angles
        rclcpp::Publisher<custom_interfaces::msg::MachineArmAngles>::SharedPtr angles_publisher_;
    };

}   // machine_hardware_interface

#endif  // MACHINE_HARDWARE_INTERFACE_H

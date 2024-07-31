#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "leg");

  std::vector<double> leg_joint_goal;

  leg_joint_goal = {0.5, 0.8, 1.2};

  move_group_interface.setJointValueTarget(leg_joint_goal);

  moveit::planning_interface::MoveGroupInterface::Plan leg_plan;

  move_group_interface.plan(leg_plan);

  //move_group_interface.execute(plan);
  move_group_interface.move();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    moveit_config = (
        MoveItConfigsBuilder("machine2", package_name="machine2_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("machine2_description"),
            "urdf",
            "machine2.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path=os.path.join(
            get_package_share_directory("machine2_moveit"),
            "config",
            "machine.srdf"
        ))
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"machine2_moveit": "config/kinematics.yaml"},
            {"machine2_moveit": "config/initial_positions.yaml"},
            {"machine2_moveit": "config/joint_limits.yaml"},
            {"machine2_moveit": "config/pilz_cartesian_limits.yaml"},
        ],
        # remappings=[
        #     ("/follow_joint_trajectory", "/arm_controller/follow_joint_trajectory")
        #     # Add more remappings here if needed
        # ]
    )

    return LaunchDescription(
        [
            move_group_node, 
        ]
    )
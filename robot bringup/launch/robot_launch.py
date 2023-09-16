import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{severity}] ({name}) {message}'
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

    node_name = LaunchConfiguration('node_name')

    # Lifecycle manager configuration file
    lc_mgr_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'lifecycle_mgr.yaml'
    )

    # Launch arguments
    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='ldlidar_node',
        description='Name of the node'
    )

    # Lifecycle manager node
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            # YAML files
            lc_mgr_config_path  # Parameters
        ]
    )

    # Include LDLidar launch
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar.launch.py'
        ]),
        launch_arguments={
            'node_name': node_name
        }.items()
    )
    
    lidar_comms_node = Node(
        package = "robot_main_pkg",
        executable = "lidar_comms_node"
    )
    
    ultrasonic_sensors_node = Node(
        package = "robot_main_pkg",
        executable = "ultrasonic_sensors_node"
    )
    
    led_control_node = Node(
        package = "robot_main_pkg",
        executable = "led_control_node"
    )
    
    motor_control_node = Node(
        package = "robot_main_pkg",
        executable = "motor_control_node"
    )
    
    # Define LaunchDescription variable
    ld = LaunchDescription()
    
    ld.add_action(lidar_comms_node)
    ld.add_action(ultrasonic_sensors_node)
    ld.add_action(led_control_node)
    ld.add_action(motor_control_node)

    # Launch arguments
    ld.add_action(declare_node_name_cmd)

    # Launch Nav2 Lifecycle Manager
    ld.add_action(lc_mgr_node)

    # Call LDLidar launch
    ld.add_action(ldlidar_launch)

    return ld
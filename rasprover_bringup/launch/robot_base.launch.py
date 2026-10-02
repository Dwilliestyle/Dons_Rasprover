#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch the RaspRover robot base system
    - ESP32 bridge (hardware interface via serial, includes voltage + audio warnings)
    - OLED display (visual status display)
    - Encoder odometry (calculates pose from wheel encoders)
    """
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    robot_params_file = PathJoinSubstitution([
        FindPackageShare('rasprover_bringup'),
        'config',
        'robot_params.yaml'
    ])
    
    encoder_odom_params_file = PathJoinSubstitution([
        FindPackageShare('rasprover_controller'),
        'config',
        'encoder_odometry_params.yaml'
    ])

    robot_teleop = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("rasprover_bringup"),
                     "launch",
                     "teleop.launch.py"
        ),
    )

    
    esp32_bridge_node = Node(
        package='rasprover_bringup',
        executable='esp32_bridge',
        name='esp32_bridge',
        output='screen',
        parameters=[
            robot_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    oled_display_node = Node(
        package='rasprover_utils',
        executable='oled_display.py',
        name='oled_display',
        output='screen',
        parameters=[
            robot_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    encoder_odometry_node = Node(
        package='rasprover_controller',
        executable='encoder_odometry',
        name='encoder_odometry',
        output='screen',
        parameters=[
            encoder_odom_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_teleop,
        esp32_bridge_node,
        oled_display_node,
        encoder_odometry_node,
    ])
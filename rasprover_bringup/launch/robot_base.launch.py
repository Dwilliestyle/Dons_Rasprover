#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch the RaspRover robot base system
    - ESP32 bridge (hardware interface via serial)
    - Battery monitor (I2C battery sensor with audio warnings)
    - OLED display (visual status display)
    - Encoder odometry (calculates pose from wheel encoders)
    """
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Path to config files
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
    
    # ESP32 bridge node - communicates with ESP32 via serial
    # Handles motor commands, IMU, encoders, voltage publishing
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
    
    # Battery monitor node - reads INA219 via I2C
    # Subscribes to: /voltage (from ESP32 bridge)
    # Provides: audio warnings for low battery
    battery_monitor_node = Node(
        package='rasprover_utils',
        executable='battery_monitor.py',
        name='battery_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # OLED display node - updates OLED with status info
    # Subscribes to: /battery_voltage, /cmd_vel
    # Displays: IP, time, status, battery voltage
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
    
    # Encoder odometry node - calculates robot pose from wheel encoders
    # Subscribes to: /odom/odom_raw (encoder counts from ESP32)
    # Publishes: /odom (nav_msgs/Odometry) and TF (odom->base_footprint)
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
        esp32_bridge_node,
        battery_monitor_node,
        oled_display_node,  # Added this!
        encoder_odometry_node,
    ])
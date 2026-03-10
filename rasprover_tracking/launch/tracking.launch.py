#!/usr/bin/env python3
"""
tracking.launch.py
------------------
Launches both the color_tracker and follow_controller nodes
with parameters loaded from tracking_params.yaml.

Usage:
    ros2 launch rasprover_tracking tracking.launch.py

Optional overrides:
    ros2 launch rasprover_tracking tracking.launch.py camera_index:=1
    ros2 launch rasprover_tracking tracking.launch.py publish_debug_image:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir    = get_package_share_directory('rasprover_tracking')
    params_file = os.path.join(pkg_dir, 'config', 'tracking_params.yaml')

    return LaunchDescription([
        # Allow overriding camera index from command line
        DeclareLaunchArgument(
            'camera_index',
            default_value='0',
            description='USB camera device index'
        ),
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='true',
            description='Publish annotated debug image topic'
        ),

        # Vision node
        Node(
            package='rasprover_tracking',
            executable='color_tracker',
            name='color_tracker',
            parameters=[
                params_file,
                {
                    'camera_index':        LaunchConfiguration('camera_index'),
                    'publish_debug_image': LaunchConfiguration('publish_debug_image'),
                }
            ],
            output='screen',
        ),

        # Controller node
        Node(
            package='rasprover_tracking',
            executable='follow_controller',
            name='follow_controller',
            parameters=[params_file],
            output='screen',
        ),
    ])
#!/usr/bin/env python3
"""
tracking.launch.py
------------------
Launches the color_tracker and follow_controller nodes with parameters
loaded from tracking_params.yaml, plus a web video server so the
annotated camera view can be watched from any browser.

Usage:
    ros2 launch rasprover_tracking tracking.launch.py

    Then open in a browser on any machine on the network:
    http://rasprover:8080/stream_viewer?topic=/rasprover/tracker/debug_image

Optional overrides:
    ros2 launch rasprover_tracking tracking.launch.py publish_debug_image:=false
    ros2 launch rasprover_tracking tracking.launch.py video_server:=false

Requires:
    sudo apt install ros-jazzy-web-video-server
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_dir     = get_package_share_directory('rasprover_tracking')
    params_file = os.path.join(pkg_dir, 'config', 'tracking_params.yaml')

    video_server = LaunchConfiguration('video_server')

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_debug_image',
            default_value='true',
            description='Publish annotated debug image topic'
        ),
        DeclareLaunchArgument(
            'video_server',
            default_value='true',
            description='Serve the debug image to a web browser on port 8080'
        ),

        # Vision node
        Node(
            package='rasprover_tracking',
            executable='color_tracker',
            name='color_tracker',
            parameters=[
                params_file,
                {
                    'publish_debug_image': ParameterValue(
                        LaunchConfiguration('publish_debug_image'), value_type=bool
                    ),
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

        # Browser camera view — runs on the robot, so the raw image never
        # crosses Wi-Fi; the browser receives a compressed MJPEG stream.
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{'port': 8080}],
            output='screen',
            condition=IfCondition(video_server),
        ),
        LogInfo(
            msg='Camera view: http://rasprover:8080/stream_viewer'
                '?topic=/rasprover/tracker/debug_image',
            condition=IfCondition(video_server),
        ),
    ])
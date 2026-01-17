#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    """
    Launch teleoperation for Rasprover with choice of input method
    - joystick: Joy node + joy teleop (default)
    - keyboard: Keyboard teleop
    
    Usage:
      ros2 launch rasprover_bringup teleop.launch.py
      ros2 launch rasprover_bringup teleop.launch.py control_mode:=keyboard
    """
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='joystick',
        description='Control mode: joystick or keyboard'
    )
    
    # Path to joy config file (hardware settings)
    joy_params_file = PathJoinSubstitution([
        FindPackageShare('rasprover_controller'),
        'config',
        'joy_config.yaml'
    ])
    
    # Path to joy teleop params (speed limits, gears, etc)
    joy_teleop_params_file = PathJoinSubstitution([
        FindPackageShare('rasprover_controller'),
        'config',
        'joy_teleop.yaml'
    ])
    
    # Condition for joystick mode
    is_joystick = PythonExpression([
        "'", LaunchConfiguration('control_mode'), "' == 'joystick'"
    ])
    
    # Condition for keyboard mode
    is_keyboard = PythonExpression([
        "'", LaunchConfiguration('control_mode'), "' == 'keyboard'"
    ])
    
    # Joy node - reads joystick hardware and publishes /joy topic
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            joy_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(is_joystick)
    )
    
    # Joy teleop node - converts /joy to /cmd_vel
    joy_teleop_node = Node(
        package='rasprover_bringup',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
        parameters=[
            joy_teleop_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(is_joystick)
    )
    
    # Keyboard teleop node
    keyboard_teleop_node = Node(
        package='rasprover_tools',
        executable='keyboard_ctrl.py',
        name='keyboard_ctrl',
        output='screen',
        prefix='xterm -e',  # Opens in new terminal window
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(is_keyboard)
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        control_mode_arg,
        joy_node,
        joy_teleop_node,
        keyboard_teleop_node,
    ])
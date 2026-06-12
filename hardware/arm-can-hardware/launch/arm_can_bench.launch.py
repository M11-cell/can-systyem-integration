"""Bench launch for ArmCanInterface.

Starts ros2_control with the CAN arm plugin, a joint_state_broadcaster, and a
forward_command_controller so you can send velocity commands from the terminal:

    ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray \\
        "{data: [0.0, 0.0, 0.0, 0.0]}"

Usage::

    source install/setup.bash
    ros2 launch arm_can_hardware arm_can_bench.launch.py can_interface:=can0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('arm_can_hardware')
    urdf_path = os.path.join(pkg_share, 'urdf', 'test_arm_can.urdf.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'arm_can_controllers.yaml')

    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='SocketCAN interface passed into ArmCanInterface URDF.',
    )
    spawn_delay_arg = DeclareLaunchArgument(
        'spawn_controller_delay',
        default_value='8.0',
        description='Seconds after ros2_control_node starts before spawning controllers.',
    )

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' can_interface:=', LaunchConfiguration('can_interface'),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers_yaml,
            {'robot_description': robot_description},
        ],
        output='screen',
    )

    delayed_control_node = TimerAction(period=3.0, actions=[control_node])

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=LaunchConfiguration('spawn_controller_delay'),
                    actions=[jsb_spawner, arm_spawner],
                ),
            ],
        )
    )

    return LaunchDescription([
        can_interface_arg,
        spawn_delay_arg,
        robot_state_publisher,
        delayed_control_node,
        delayed_spawners,
    ])

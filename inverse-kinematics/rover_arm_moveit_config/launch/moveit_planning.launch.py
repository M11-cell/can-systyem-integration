"""MoveIt planning demo in RViz — no CAN hardware.

Starts everything needed to plan and visualize the rover arm in RViz:
  - static virtual joint TFs
  - robot_state_publisher (fake/mock ros2_control URDF, no SocketCAN)
  - move_group
  - RViz with Motion Planning plugin

By default this is planning-only: use the **Plan** button in RViz (not Plan and
Execute). Drag joints with joint_state_publisher_gui, or enable simulated
execution with enable_execution:=true.

Usage::

    source install/setup.bash
    ros2 launch rover_arm_moveit_config moveit_planning.launch.py

Simulated Plan and Execute in RViz (fake ros2_control, no real motors)::

    ros2 launch rover_arm_moveit_config moveit_planning.launch.py enable_execution:=true

Optional::

    ros2 launch rover_arm_moveit_config moveit_planning.launch.py use_rviz:=false
    ros2 launch rover_arm_moveit_config moveit_planning.launch.py use_joint_gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown as ShutdownEvent
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_arm_moveit_config')

    moveit_config = (
        MoveItConfigsBuilder('rover_arm', package_name='rover_arm_moveit_config')
        .robot_description(mappings={'hardware_backend': 'fake'})
        .to_moveit_configs()
    )

    planning_controllers_path = os.path.join(
        pkg_share, 'config', 'planning_ros2_controllers.yaml'
    )

    enable_execution_arg = DeclareLaunchArgument(
        'enable_execution',
        default_value='false',
        description=(
            'Start fake ros2_control + arm_controller so Plan and Execute works in RViz. '
            'When false, use the Plan button only.'
        ),
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz with the MoveIt Motion Planning UI.',
    )
    use_joint_gui_arg = DeclareLaunchArgument(
        'use_joint_gui',
        default_value='true',
        description=(
            'Launch joint_state_publisher_gui for interactive joint poses. '
            'Ignored when enable_execution:=true (joint_state_broadcaster is used instead).'
        ),
    )

    planning_hint = LogInfo(
        msg=(
            '[moveit_planning] Planning-only mode: click Plan in RViz, not Plan and Execute. '
            'For simulated execution: enable_execution:=true'
        ),
        condition=UnlessCondition(LaunchConfiguration('enable_execution')),
    )

    static_tfs = generate_static_virtual_joint_tfs_launch(moveit_config)
    rsp = generate_rsp_launch(moveit_config)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': LaunchConfiguration('enable_execution'),
                'capabilities': ParameterValue('', value_type=str),
                'disable_capabilities': ParameterValue('', value_type=str),
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'monitor_dynamics': False,
            },
        ],
        additional_env={'DISPLAY': os.environ.get('DISPLAY', '')},
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[planning_controllers_path, moveit_config.robot_description],
        output='screen',
        sigterm_timeout='3',
        sigkill_timeout='2',
        condition=IfCondition(LaunchConfiguration('enable_execution')),
    )

    controller_names = moveit_config.trajectory_execution.get(
        'moveit_simple_controller_manager', {}
    ).get('controller_names', [])
    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller],
            output='screen',
            sigterm_timeout='3',
            sigkill_timeout='2',
            condition=IfCondition(LaunchConfiguration('enable_execution')),
        )
        for controller in controller_names + ['joint_state_broadcaster']
    ]

    joint_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        sigterm_timeout='3',
        sigkill_timeout='2',
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('use_joint_gui'),
                "' == 'true' and '",
                LaunchConfiguration('enable_execution'),
                "' != 'true'",
            ])
        ),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', os.path.join(pkg_share, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        sigterm_timeout='3',
        sigkill_timeout='2',
    )

    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_node,
            on_exit=[EmitEvent(event=ShutdownEvent(reason='RViz closed'))],
        ),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    ld = LaunchDescription([
        enable_execution_arg,
        use_rviz_arg,
        use_joint_gui_arg,
        planning_hint,
        move_group_node,
        ros2_control_node,
        joint_gui,
        rviz_node,
        shutdown_on_rviz_exit,
        *controller_spawners,
    ])

    for child in (static_tfs, rsp):
        for entity in child.entities:
            ld.add_action(entity)

    return ld

"""Bring up ros2_control for the six-wheel CAN stack.

Passes robot_description into ros2_control_node (required). Without this,
controller_manager logs forever::

    Waiting for data on 'robot_description' topic to finish initialization

Local teleop (joystick on the same machine) starts joy_node, joy_mux_controller,
and skid_steer_mux by default. skid_steer_mux is the only publisher of
/velocity_controller/commands here; do NOT also run wheel_bench_node on the
same CAN interface (they are mutually exclusive wheel owners).

Usage::

    source install/setup.bash
    ros2 launch wheel_can_hardware ros2_control_wheels.launch.py can_interface:=can0

Joystick on another PC (same ROS_DOMAIN_ID), wheels on the Jetson::

    # PC: joy_node + joy_mux_controller (publishes /cmd_vel, /rover/drive_mode)
    ros2 run joy joy_node
    ros2 run joy_mux_controller_py joy_mux_controller

    # Jetson: ros2_control + skid_steer_mux only (no duplicate joy nodes)
    ros2 launch wheel_can_hardware ros2_control_wheels.launch.py \\
        can_interface:=can0 launch_teleop:=false

    # Bench / scripted control — disable skid_steer_mux so you own
    # /velocity_controller/commands directly:
    ros2 launch wheel_can_hardware ros2_control_wheels.launch.py \\
        can_interface:=can0 launch_teleop:=false launch_skid_steer_mux:=false
    ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray \\
        "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

With odometry node (needs /joint_states from joint_state_broadcaster)::

    ros2 launch wheel_can_hardware ros2_control_wheels.launch.py \\
        can_interface:=can0 launch_odometry:=true

Arm control lives in the arm / MoveIt launch (arm_hardware.launch.py with
arm_can_hardware plugin). Safety stop/resume for wheels is can_safety_node,
not part of ros2_control.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('wheel_can_hardware')
    urdf_path = os.path.join(pkg_share, 'urdf', 'test_rover_wheels.urdf.xacro')

    # control_mode selects both the controllers file and the spawned controller:
    # velocity -> velocity_controller, current -> effort_controller (bench).
    controllers_yaml_velocity = os.path.join(pkg_share, 'config', 'wheel_controllers.yaml')
    controllers_yaml_current = os.path.join(pkg_share, 'config', 'wheel_controllers_current.yaml')
    is_current = ["'", LaunchConfiguration('control_mode'), "' == 'current'"]
    controllers_yaml = PythonExpression(
        ["'", controllers_yaml_current, "' if (", *is_current, ") else '", controllers_yaml_velocity, "'"]
    )
    wheel_controller_name = PythonExpression(
        ["'effort_controller' if (", *is_current, ") else 'velocity_controller'"]
    )

    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='SocketCAN interface passed into WheelCanInterface URDF.',
    )
    launch_odom_arg = DeclareLaunchArgument(
        'launch_odometry',
        default_value='false',
        description='If true, also start wheel_odometry_node.',
    )
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.15',
        description='Wheel radius (m) for wheel_odometry_node; autonomy diff_drive uses 0.15.',
    )
    track_width_arg = DeclareLaunchArgument(
        'track_width',
        default_value='0.591',
        description='Track width (m) for skid_steer_mux and wheel_odometry_node.',
    )
    multiplier_arg = DeclareLaunchArgument(
        'multiplier',
        default_value='750.0',
        description='skid_steer_mux stick-to-motor-RPM gain.',
    )
    traction_mode_arg = DeclareLaunchArgument(
        'traction_mode',
        default_value='assist',
        description='WheelCanInterface traction mode: off | assist | aggressive.',
    )
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='velocity',
        description=(
            'WheelCanInterface command mode: velocity (velocity_controller) or '
            'current (effort_controller, bench). Use launch_teleop:=false for current.'
        ),
    )
    launch_teleop_arg = DeclareLaunchArgument(
        'launch_teleop',
        default_value='true',
        description='If true, start joy_node + joy_mux_controller (and skid_steer_mux).',
    )
    launch_skid_steer_mux_arg = DeclareLaunchArgument(
        'launch_skid_steer_mux',
        default_value='true',
        description=(
            'If true, start skid_steer_mux (/cmd_vel -> /velocity_controller/commands). '
            'Defaults on so launch_teleop:=false still drives wheels from a remote joy PC. '
            'Set false for bench when you publish /velocity_controller/commands directly.'
        ),
    )
    spawn_after_arg = DeclareLaunchArgument(
        'spawn_controller_delay',
        default_value='8.0',
        description=(
            'Seconds after ros2_control_node starts before spawning controllers '
            '(hardware plugins + CAN need time before list_controllers exists).'
        ),
    )
    log_telemetry_arg = DeclareLaunchArgument(
        'log_telemetry',
        default_value='true',
        description=(
            'If true, WheelCanInterface prints a wheel_bench_node-style telemetry '
            'table every print_period_ms.'
        ),
    )
    print_period_arg = DeclareLaunchArgument(
        'print_period_ms',
        default_value='1000',
        description='Telemetry table print interval in milliseconds.',
    )

    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                urdf_path,
                ' can_interface:=',
                LaunchConfiguration('can_interface'),
                ' traction_mode:=',
                LaunchConfiguration('traction_mode'),
                ' control_mode:=',
                LaunchConfiguration('control_mode'),
                ' log_telemetry:=',
                LaunchConfiguration('log_telemetry'),
                ' print_period_ms:=',
                LaunchConfiguration('print_period_ms'),
            ]
        ),
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

    # Give controller_manager time to load URDF plugins and open CAN before
    # list_controllers is advertised (matches arm_hardware.launch.py pattern).
    delayed_control_node = TimerAction(
        period=3.0,
        actions=[control_node],
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
        output='screen',
    )

    vel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            wheel_controller_name,
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
        output='screen',
    )

    delayed_spawners = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=LaunchConfiguration('spawn_controller_delay'),
                    actions=[jsb_spawner, vel_spawner],
                ),
            ],
        )
    )

    wheel_odometry_node = Node(
        package='wheel_can_hardware',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        output='screen',
        parameters=[
            {
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'track_width': LaunchConfiguration('track_width'),
                'publish_tf': True,
            }
        ],
        condition=IfCondition(LaunchConfiguration('launch_odometry')),
    )

    # Local joy: joystick -> joy_mux_controller -> /cmd_vel + /rover/drive_mode.
    # skid_steer_mux subscribes to those topics (local or from another host) and
    # publishes /velocity_controller/commands.
    teleop_condition = IfCondition(LaunchConfiguration('launch_teleop'))
    skid_steer_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration('launch_skid_steer_mux'),
                "' == 'true' or '",
                LaunchConfiguration('launch_teleop'),
                "' == 'true'",
            ]
        )
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=teleop_condition,
    )

    joy_mux_controller = Node(
        package='joy_mux_controller_py',
        executable='joy_mux_controller',
        name='joy_mux_controller',
        output='screen',
        condition=teleop_condition,
    )

    skid_steer_mux = Node(
        package='wheel_can_hardware',
        executable='skid_steer_mux',
        name='skid_steer_mux',
        output='screen',
        parameters=[
            {
                'track_width': LaunchConfiguration('track_width'),
                'multiplier': LaunchConfiguration('multiplier'),
            }
        ],
        condition=skid_steer_condition,
    )

    return LaunchDescription(
        [
            can_interface_arg,
            launch_odom_arg,
            wheel_radius_arg,
            track_width_arg,
            multiplier_arg,
            traction_mode_arg,
            control_mode_arg,
            launch_teleop_arg,
            launch_skid_steer_mux_arg,
            spawn_after_arg,
            log_telemetry_arg,
            print_period_arg,
            robot_state_publisher,
            delayed_control_node,
            delayed_spawners,
            wheel_odometry_node,
            joy_node,
            joy_mux_controller,
            skid_steer_mux,
        ]
    )

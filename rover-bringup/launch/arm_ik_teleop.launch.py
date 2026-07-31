"""Unified arm IK teleop bring-up: hardware/sim + MoveIt + VKB teleop.

Wires the whole arm teleop stack behind one launch so the same VKB controls
work in simulation and on the real CAN arm:

  robot_state_publisher
  ros2_control_node (+ joint_state_broadcaster, arm_servo_controller,
                     arm_trajectory_controller loaded inactive)
  move_group        (Plan & Execute / Poses mode)
  moveit_servo      (FK joint jog + Cartesian teleop)
  ik_mux_vkb        (B2 wheels/arm, F1 FK, F2 Servo, F3 Poses, D1 deadman)
  joy_node, rviz2   (optional)

Simulation first (default, no CAN required)::

    ros2 launch rover_bringup arm_ik_teleop.launch.py

Real hardware (after validating in sim)::

    ros2 launch rover_bringup arm_ik_teleop.launch.py \\
        hardware_backend:=can can_interface:=can0 \\
        launch_wheels:=true launch_safety:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def _spawner(controller, inactive=False):
    args = [controller, "--controller-manager", "/controller_manager"]
    if inactive:
        args.append("--inactive")
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=args,
        output="screen",
    )


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("rover_arm_moveit_config")

    backend = LaunchConfiguration("hardware_backend").perform(context)
    can_interface = LaunchConfiguration("can_interface").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context) == "true"
    launch_joy = LaunchConfiguration("launch_joy").perform(context) == "true"
    launch_wheels = LaunchConfiguration("launch_wheels").perform(context) == "true"
    launch_safety = LaunchConfiguration("launch_safety").perform(context) == "true"
    start_in_arm_mode = LaunchConfiguration("start_in_arm_mode").perform(context) == "true"

    # MoveItConfigsBuilder evaluates xacro at construction, so resolve the backend
    # here (inside OpaqueFunction) rather than passing a launch substitution.
    moveit_config = (
        MoveItConfigsBuilder("rover_arm", package_name="rover_arm_moveit_config")
        .robot_description(
            file_path="config/rover_arm.urdf.xacro",
            mappings={"hardware_backend": backend, "can_interface": can_interface},
        )
        # Default sensors_3d.yaml references a Kinect depth plugin we do not ship.
        .sensors_3d(file_path="config/sensors_3d_empty.yaml")
        .to_moveit_configs()
    )

    move_group_parameters = moveit_config.to_dict()
    for sensor_key in ("sensors", "default_sensor", "kinect_depthimage"):
        move_group_parameters.pop(sensor_key, None)

    # CheckStartStateBounds is initialized with the planning-pipeline namespace
    # (e.g. "ompl"), so it reads ompl.fix_start_state — NOT
    # default_request_adapter_parameters.fix_start_state. Without this, Servo
    # teleop that drives continuous joints past ±pi makes A2 / Plan&Execute
    # fail with START_STATE_INVALID.
    for pipeline in (
        "ompl",
        "chomp",
        "stomp",
        "pilz_industrial_motion_planner",
    ):
        move_group_parameters[f"{pipeline}.fix_start_state"] = True

    controllers_file = (
        "sim_ros2_controllers.yaml" if backend == "fake" else "hardware_ros2_controllers.yaml"
    )
    controllers_path = os.path.join(pkg_share, "config", controllers_file)

    servo_params = {
        "moveit_servo": ParameterBuilder("rover_arm_moveit_config")
        .yaml("config/servo_config.yaml")
        .to_dict()
    }

    static_tfs = generate_static_virtual_joint_tfs_launch(moveit_config)

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, controllers_path],
        output="screen",
    )
    # Sim boots mock hardware immediately; CAN hardware keeps a short settle delay.
    control_startup_delay_s = 0.0 if backend == "fake" else 3.0
    if control_startup_delay_s > 0.0:
        delayed_controller_manager = TimerAction(
            period=control_startup_delay_s, actions=[ros2_control_node]
        )
    else:
        delayed_controller_manager = ros2_control_node

    jsb_spawner = _spawner("joint_state_broadcaster")
    servo_ctrl_spawner = _spawner("arm_servo_controller")
    traj_ctrl_spawner = _spawner("arm_trajectory_controller", inactive=True)
    gripper_ctrl_spawner = _spawner("gripper_controller")
    start_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[jsb_spawner],
        )
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            move_group_parameters,
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "capabilities": ParameterValue("", value_type=str),
                "disable_capabilities": ParameterValue("", value_type=str),
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    ik_mux_node = Node(
        package="ik_mux_controller",
        executable="ik_mux_vkb",
        name="ik_mux_vkb",
        output="screen",
        parameters=[{"start_in_arm_mode": start_in_arm_mode}],
    )

    # Load controllers in a deterministic sequence and start the mux only after
    # all spawners finish. Otherwise an in-flight F1/F2/F3 mode switch can be
    # overwritten by a late controller spawner during startup.
    start_servo_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[servo_ctrl_spawner],
        )
    )
    start_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=servo_ctrl_spawner,
            on_exit=[traj_ctrl_spawner],
        )
    )
    start_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=traj_ctrl_spawner,
            on_exit=[gripper_ctrl_spawner],
        )
    )
    start_ik_mux = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_ctrl_spawner,
            on_exit=[ik_mux_node],
        )
    )

    # Turns MoveIt Servo's generic singularity halt into per-joint escape hints
    # on /arm/singularity_hint. Needs the robot model + SRDF to build the arm
    # Jacobian.
    singularity_advisor_node = Node(
        package="ik_mux_controller",
        executable="singularity_advisor",
        # Name comes from main(); do not also set name= here (double Node
        # registration → duplicate /singularity_advisor in ros2 node list).
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    actions = []
    for entity in static_tfs.entities:
        actions.append(entity)

    actions.extend([
        rsp,
        delayed_controller_manager,
        start_jsb,
        start_servo_controller,
        start_trajectory_controller,
        start_gripper_controller,
        start_ik_mux,
        move_group_node,
        servo_node,
        singularity_advisor_node,
    ])

    if launch_joy:
        actions.append(
            Node(package="joy", executable="joy_node", name="joy_node", output="screen")
        )

    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                arguments=["-d", os.path.join(pkg_share, "config", "moveit.rviz")],
                parameters=[
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
            )
        )

    if launch_wheels:
        # Wheel ros2_control + skid_steer_mux (/cmd_vel -> wheels). ik_mux_vkb
        # owns /joy, so disable the wheel launch's own joy teleop.
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("wheel_can_hardware"),
                        "launch",
                        "ros2_control_wheels.launch.py",
                    )
                ),
                launch_arguments={
                    "can_interface": can_interface,
                    "launch_teleop": "false",
                    "launch_skid_steer_mux": "true",
                }.items(),
            )
        )

    if launch_safety:
        # Remap safety off F1/F2 (26/27) since ik_mux_vkb uses those for arm modes.
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("can_safety_node"),
                        "launch",
                        "safety.launch.py",
                    )
                ),
                launch_arguments={
                    "can_interface": can_interface,
                    "wheel_force_stop_button": "0",
                    "wheel_resume_button": "1",
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_backend",
                default_value="fake",
                description="fake = simulation (mock hardware, no CAN); can = real ArmCanInterface.",
            ),
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface (only used when hardware_backend:=can).",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with the MoveIt Motion Planning UI.",
            ),
            DeclareLaunchArgument(
                "launch_joy",
                default_value="true",
                description="Launch joy_node for the VKB joystick.",
            ),
            DeclareLaunchArgument(
                "launch_wheels",
                default_value="false",
                description="Also start wheel ros2_control + skid_steer_mux for B2 wheels mode.",
            ),
            DeclareLaunchArgument(
                "launch_safety",
                default_value="false",
                description="Start can_safety_node with F1/F2 remapped off the arm-mode buttons.",
            ),
            DeclareLaunchArgument(
                "start_in_arm_mode",
                default_value="false",
                description="Boot ik_mux_vkb in arm/Servo mode instead of wheels.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

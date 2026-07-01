# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""
Unified bringup with RUNTIME position <-> force switching.

Loads, on a single controller_manager:
  * joint_state_broadcaster      (always active)
  * plan_group_controller (JTC)  (active at start = POSITION mode, MoveIt drives it)
  * cartesian_impedance_controller (loaded INACTIVE = FORCE mode, on demand)
  * mode_switcher                (coordinator: ~/to_position, ~/to_force)

Switch at runtime without relaunch:
    ros2 service call /mode_switcher/to_force    std_srvs/srv/Trigger
    ros2 service call /mode_switcher/to_position std_srvs/srv/Trigger

    # fake hardware (mock accepts position AND effort -> switch works in sim):
    ros2 launch armv7_bringup unified.launch.py use_fake_hardware:=true use_rt:=false

On REAL EtherCAT hardware the two control modes also need different CiA-402 drive
modes (CSP 8 / CST 10). That drive-mode handover is Step 2 (armv7_unified.ros2_control
xacro + cia402 mode command interface + mode_switcher manage_drive_mode:=true);
until then, real hardware should keep using arm.launch.py / impedance.launch.py.
"""
import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
)


def _ros2_control_xacro(use_fake_hardware: str) -> str:
    name = ('armv7_fake.ros2_control.xacro'
            if use_fake_hardware == 'true'
            else 'armv7_unified.ros2_control.xacro')
    return str(get_package_share_path('armv7_bringup') / 'urdf' / name)


def _build_moveit_config(ros2_control_xacro_path: str):
    urdf_xacro = str(get_package_share_path('armv7_description') / 'urdf' / 'armv7.urdf.xacro')
    return (
        MoveItConfigsBuilder('armv7', package_name='armv7_moveit_config')
        .robot_description(
            file_path=urdf_xacro,
            mappings={'ros2_control_xacro': ros2_control_xacro_path},
        )
        .to_moveit_configs()
    )


def _spawner(name, inactive=False):
    args = [name, '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30']
    if inactive:
        args.append('--inactive')
    return Node(package='controller_manager', executable='spawner',
                arguments=args, output='screen')


def _setup(context):
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    use_rt = LaunchConfiguration('use_rt').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    identified_params = LaunchConfiguration('identified_params').perform(context)

    moveit_config = _build_moveit_config(_ros2_control_xacro(use_fake_hardware))

    actions = []
    actions += generate_rsp_launch(moveit_config).entities
    moveit_share = Path(moveit_config.package_path)
    if (moveit_share / 'launch' / 'static_virtual_joint_tfs.launch.py').exists():
        actions += generate_static_virtual_joint_tfs_launch(moveit_config).entities
    actions += generate_move_group_launch(moveit_config).entities
    if use_rviz == 'true':
        actions += generate_moveit_rviz_launch(moveit_config).entities

    bringup_share = get_package_share_path('armv7_bringup')
    imp_share = get_package_share_path('armv7_impedance_moveit')
    controllers_yaml = str(bringup_share / 'config' / 'ros2_controllers.yaml')
    impedance_yaml = str(imp_share / 'config' / 'impedance.yaml')
    cm_params = [moveit_config.robot_description, controllers_yaml, impedance_yaml]

    if identified_params:
        resolved = os.path.abspath(os.path.expanduser(os.path.expandvars(identified_params)))
        fd, override_yaml = tempfile.mkstemp(suffix='.yaml', prefix='unified_imp_')
        os.close(fd)
        with open(override_yaml, 'w') as f:
            f.write('cartesian_impedance_controller:\n'
                    '  ros__parameters:\n'
                    f'    identified_params_file: {resolved}\n')
        cm_params.append(override_yaml)

    rt_prefix = ['chrt -f 99'] if use_rt == 'true' else []
    ros2_control_node = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=cm_params, prefix=rt_prefix, output='screen')
    actions.append(TimerAction(period=5.0, actions=[ros2_control_node]))

    # Startup mode. FORCE is the safe default: the impedance controller comes up
    # active but DISABLED (0 torque) so the arm just holds where it is. POSITION
    # would activate the JTC, whose command buffer is uninitialised at activation
    # and snaps the arm toward zero -- a real motion hazard at power-on.
    start_mode = LaunchConfiguration('start_mode').perform(context)
    force_at_start = start_mode != 'position'

    # Serial spawn chained via OnProcessExit to dodge the controller_manager
    # >= 2.54 parallel-spawn race. The selected start controller comes up active,
    # the other inactive. On real hardware mode_controller (always active) holds
    # the mode interface and is spawned right after jsb.
    real_hw = use_fake_hardware != 'true'
    jsb = _spawner('joint_state_broadcaster')
    jtc = _spawner('plan_group_controller', inactive=force_at_start)
    imp = _spawner('cartesian_impedance_controller', inactive=not force_at_start)
    # spawn the active controller first, then the inactive one
    first, second = (imp, jtc) if force_at_start else (jtc, imp)
    actions.append(TimerAction(period=7.0, actions=[jsb]))
    if real_hw:
        mode_ctrl = _spawner('mode_controller')
        # Always-active utility controller holding the reset_fault interface
        # (rqt 错误复位 drives it). Mock hardware has no reset_fault interface,
        # so it is real-hardware only.
        fault_ctrl = _spawner('fault_reset_controller')
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=jsb, on_exit=[mode_ctrl])))
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=mode_ctrl, on_exit=[fault_ctrl])))
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=fault_ctrl, on_exit=[first])))
    else:
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=jsb, on_exit=[first])))
    actions.append(RegisterEventHandler(OnProcessExit(target_action=first, on_exit=[second])))

    # Mode coordinator. manage_drive_mode only on real hardware (fake has no
    # mode_of_operation interface / no drive mode). It seeds current_mode from
    # start_mode so clients see the right state without an explicit switch.
    mode_switcher = Node(
        package='armv7_mode_manager', executable='mode_switcher',
        name='mode_switcher', output='screen',
        parameters=[{
            'position_controllers': ['plan_group_controller'],
            'force_controllers': ['cartesian_impedance_controller'],
            'manage_drive_mode': real_hw,
            'start_mode': 'force' if force_at_start else 'position',
        }])
    actions.append(TimerAction(period=9.0, actions=[mode_switcher]))

    # MoveIt Servo: real-time Cartesian jog for POSITION mode. Streams a
    # JointTrajectory to plan_group_controller (active only in position mode, so
    # its output is ignored in force mode). The rqt 笛卡尔 jog tab drives it via
    # ~/delta_twist_cmds and ~/start_servo / ~/stop_servo. Reads move_group's
    # planning scene for collision checking.
    if LaunchConfiguration('use_servo').perform(context) == 'true':
        servo_yaml = str(bringup_share / 'config' / 'servo.yaml')
        with open(servo_yaml) as f:
            servo_params = {'moveit_servo': yaml.safe_load(f)}
        servo_node = Node(
            package='moveit_servo', executable='servo_node_main',
            name='servo_node', output='screen',
            parameters=[
                servo_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ])
        actions.append(TimerAction(period=11.0, actions=[servo_node]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('use_rt', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_servo', default_value='true',
            description='Launch MoveIt Servo for Cartesian jog in position mode.'),
        DeclareLaunchArgument(
            'start_mode', default_value='force',
            description='Startup control mode: "force" (impedance active+disabled, '
                        'arm holds -- safe) or "position" (JTC active, may snap to '
                        'zero on activation).'),
        DeclareLaunchArgument(
            'identified_params',
            default_value='~/arm_ws/src/armv7_dyn_ident/config/identified_params.yaml',
            description='armv7_dyn_ident identified_params.yaml for the impedance side.'),
        OpaqueFunction(function=_setup),
    ])

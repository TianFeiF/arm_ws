# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Unified bringup for the armv7 arm.

Selects fake (mock_components) vs real EtherCAT hardware via `use_fake_hardware`.

    ros2 launch armv7_bringup arm.launch.py                       # real EtherCAT
    ros2 launch armv7_bringup arm.launch.py use_fake_hardware:=true
    ros2 launch armv7_bringup arm.launch.py use_rviz:=false       # headless
    ros2 launch armv7_bringup arm.launch.py use_rt:=false         # no chrt -f 99
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
    generate_warehouse_db_launch,
)


def _ros2_control_xacro(use_fake_hardware: str) -> str:
    name = (
        'armv7_fake.ros2_control.xacro'
        if use_fake_hardware == 'true'
        else 'armv7_ethercat.ros2_control.xacro'
    )
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


def _setup(context):
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    use_rt = LaunchConfiguration('use_rt').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    use_db = LaunchConfiguration('db').perform(context)
    use_safety = LaunchConfiguration('use_safety').perform(context)
    use_diagnostics = LaunchConfiguration('use_diagnostics').perform(context)

    moveit_config = _build_moveit_config(_ros2_control_xacro(use_fake_hardware))

    actions = []

    # robot_state_publisher + (optional) virtual-joint static TFs
    actions += generate_rsp_launch(moveit_config).entities
    moveit_share = Path(moveit_config.package_path)
    if (moveit_share / 'launch' / 'static_virtual_joint_tfs.launch.py').exists():
        actions += generate_static_virtual_joint_tfs_launch(moveit_config).entities

    # move_group
    actions += generate_move_group_launch(moveit_config).entities

    if use_rviz == 'true':
        actions += generate_moveit_rviz_launch(moveit_config).entities

    if use_db == 'true':
        actions += generate_warehouse_db_launch(moveit_config).entities

    # ros2_control_node — wait 5 s for MoveIt to settle. SCHED_FIFO 99 if use_rt=true.
    bringup_share = get_package_share_path('armv7_bringup')
    controllers_yaml = str(bringup_share / 'config' / 'ros2_controllers.yaml')
    rt_prefix = ['chrt -f 99'] if use_rt == 'true' else []
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[moveit_config.robot_description, controllers_yaml],
        prefix=rt_prefix,
    )
    actions.append(TimerAction(period=5.0, actions=[ros2_control_node]))

    # Chain the two spawners via OnProcessExit so plan_group_controller is
    # only spawned AFTER joint_state_broadcaster has fully finished spawning
    # and exited. This is the official ros2_control pattern that sidesteps
    # the controller_manager >= 2.54 race in which a parallel/serial spawner
    # call randomly fails with
    #     "Failed loading controller plan_group_controller"
    # right after joint_state_broadcaster is activated.
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )
    pgc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'plan_group_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    actions.append(TimerAction(period=7.0, actions=[jsb_spawner]))
    actions.append(RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[pgc_spawner])
    ))

    # Safety layer (workspace bbox + E-Stop) — start after controllers are up.
    if use_safety == 'true':
        safety_share = get_package_share_path('armv7_safety')
        actions.append(TimerAction(period=8.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(safety_share / 'launch' / 'safety.launch.py'))
        )]))

    # Diagnostics aggregator — same timing.
    if use_diagnostics == 'true':
        diag_share = get_package_share_path('armv7_diagnostics')
        actions.append(TimerAction(period=8.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(diag_share / 'launch' / 'diagnostics.launch.py'))
        )]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                              description='Use mock_components instead of EtherCAT.'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Start RViz with MoveIt motion-planning panel.'),
        DeclareLaunchArgument('db', default_value='false',
                              description='Start MoveIt warehouse database.'),
        DeclareLaunchArgument('use_rt', default_value='true',
                              description='Run ros2_control_node under SCHED_FIFO 99 (needs realtime group).'),
        DeclareLaunchArgument('use_safety', default_value='true',
                              description='Bring up armv7_safety (workspace bbox + E-Stop).'),
        DeclareLaunchArgument('use_diagnostics', default_value='true',
                              description='Bring up armv7_diagnostics (/diagnostics aggregator).'),
        OpaqueFunction(function=_setup),
    ])

# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""
Cartesian impedance bringup (mirrors armv7_zero_force_controller free_drive).

Brings the arm up in TORQUE mode and loads ONLY joint_state_broadcaster +
cartesian_impedance_controller — no MoveIt, because a drive can't be in
position and torque mode at the same time.

    # real hardware (drives switch to CiA-402 CST / mode 10):
    ros2 launch armv7_impedance_moveit impedance.launch.py

    # dry run on mock hardware:
    ros2 launch armv7_impedance_moveit impedance.launch.py \
        use_fake_hardware:=true use_rt:=false

DANGER: same caveats as free-drive. The controller starts DISABLED; on enable
the spring-damper engages. A bad target pose with high stiffness = large
wrench — keep `max_pose_error` and `wrench_limit` in impedance.yaml tight
while you tune.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _ros2_control_xacro(use_fake_hardware: str) -> str:
    name = ('armv7_fake.ros2_control.xacro'
            if use_fake_hardware == 'true'
            else 'armv7_cst.ros2_control.xacro')
    return str(get_package_share_path('armv7_bringup') / 'urdf' / name)


def _setup(context):
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    use_rt = LaunchConfiguration('use_rt').perform(context)
    identified_params = LaunchConfiguration('identified_params').perform(context)

    urdf_xacro = str(get_package_share_path('armv7_description') / 'urdf' / 'armv7.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_xacro,
                     ' ros2_control_xacro:=', _ros2_control_xacro(use_fake_hardware)],
                    on_stderr='ignore'),
            value_type=str),
    }

    controllers_yaml = str(
        get_package_share_path('armv7_impedance_moveit')
        / 'config' / 'impedance.yaml')

    cm_params = [robot_description, controllers_yaml]
    if identified_params:
        resolved = os.path.abspath(os.path.expanduser(os.path.expandvars(identified_params)))
        fd, override_yaml = tempfile.mkstemp(suffix='.yaml', prefix='imp_override_')
        os.close(fd)
        with open(override_yaml, 'w') as f:
            f.write(
                'cartesian_impedance_controller:\n'
                '  ros__parameters:\n'
                f'    identified_params_file: {resolved}\n')
        cm_params.append(override_yaml)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    rt_prefix = ['chrt -f 99'] if use_rt == 'true' else []
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=cm_params,
        prefix=rt_prefix,
        output='screen',
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen',
    )
    ic_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_impedance_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen',
    )

    return [
        rsp,
        ros2_control_node,
        jsb_spawner,
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[ic_spawner])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                              description='mock_components instead of EtherCAT CST.'),
        DeclareLaunchArgument('use_rt', default_value='true',
                              description='Run ros2_control_node under SCHED_FIFO 99.'),
        DeclareLaunchArgument(
            'identified_params',
            default_value='~/arm_ws/src/armv7_dyn_ident/config/identified_params.yaml',
            description='Optional armv7_dyn_ident identified_params.yaml path.'),
        OpaqueFunction(function=_setup),
    ])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("armv7", package_name="armv7_moveit")
        .robot_description(file_path="config/armv7ec.urdf.xacro")
        .to_moveit_configs()
    )
    
    ld = LaunchDescription()
    
    # Arguments
    ld.add_action(DeclareLaunchArgument("db", default_value="false", description="Start database"))
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true", description="Start RViz"))

    launch_package_path = moveit_config.package_path
    
    # 1. Virtual joints TF
    virtual_joints_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(virtual_joints_launch))))

    # 2. Robot State Publisher
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_package_path / "launch/rsp.launch.py"))))

    # 3. Move Group
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_package_path / "launch/move_group.launch.py"))))

    # 4. RViz
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_package_path / "launch/moveit_rviz.launch.py")),
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    ))

    # 5. Database (optional)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_package_path / "launch/warehouse_db.launch.py")),
        condition=IfCondition(LaunchConfiguration("db"))
    ))

    # 6. ros2_control_node (Delayed start to ensure MoveIt and RViz are fully loaded)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
    )
    
    # Delay ros2_control node by 5 seconds to ensure MoveIt and RViz are fully loaded
    delayed_ros2_control = TimerAction(
        period=5.0,
        actions=[ros2_control_node]
    )
    ld.add_action(delayed_ros2_control)

    # 7. Spawn controllers (Delayed further to ensure ros2_control is up)
    delayed_spawn_controllers = TimerAction(
        period=7.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/spawn_controllers.launch.py"))
        )]
    )
    ld.add_action(delayed_spawn_controllers)

    return ld

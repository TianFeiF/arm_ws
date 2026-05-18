from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('armv7')
    urdf_file = os.path.join(pkg_share, 'urdf', 'armv7.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    # Replace package:// with file:// for Gazebo compatibility
    robot_desc = robot_desc.replace('package://armv7', 'file://' + pkg_share)

    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_model',
            arguments=['-topic', 'robot_description', '-name', 'armv7', '-z', '0.1'],
            output='screen'
        )
    ])

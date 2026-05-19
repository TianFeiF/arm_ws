from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('armv7')
    urdf_file = os.path.join(pkg_share, 'urdf', 'armv7.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    # Replace package:// with file:// for compatibility
    robot_desc = robot_desc.replace('package://armv7', 'file://' + pkg_share)

    rviz_config_file = os.path.join(pkg_share, 'urdf.rviz')
    rviz_args = []
    if os.path.exists(rviz_config_file):
        rviz_args = ['-d', rviz_config_file]

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_args,
            output='screen'
        )
    ])

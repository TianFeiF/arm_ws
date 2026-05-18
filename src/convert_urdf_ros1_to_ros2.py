#!/usr/bin/env python3
import os
import re
import argparse

def convert_package(base_path, package_name):
    pkg_path = os.path.join(base_path, package_name)
    
    if not os.path.exists(pkg_path):
        print(f"Error: Package directory '{pkg_path}' does not exist.")
        return

    print(f"Converting '{package_name}' at '{pkg_path}' to ROS 2 format...")

    # 1. Update CMakeLists.txt
    cmake_path = os.path.join(pkg_path, 'CMakeLists.txt')
    if os.path.exists(cmake_path):
        with open(cmake_path, 'r') as f:
            cmake_content = f.read()
        
        # Replace cmake version
        cmake_content = re.sub(r'cmake_minimum_required\(VERSION .*?\)', 'cmake_minimum_required(VERSION 3.10)', cmake_content)
        # Replace catkin with ament_cmake
        cmake_content = cmake_content.replace('find_package(catkin REQUIRED)', 'find_package(ament_cmake REQUIRED)')
        # Remove catkin_package() and find_package(roslaunch)
        cmake_content = re.sub(r'catkin_package\(\)\s*', '', cmake_content)
        cmake_content = re.sub(r'find_package\(roslaunch\)\s*', '', cmake_content)
        
        # Update install rules
        old_install = r'foreach\(dir config launch meshes urdf\).*?install\(DIRECTORY \$\{dir\}/.*?DESTINATION \$\{CATKIN_PACKAGE_SHARE_DESTINATION\}/\$\{dir\}\).*?endforeach\(dir\)'
        new_install = 'install(DIRECTORY config launch meshes urdf\n  DESTINATION share/${PROJECT_NAME}\n)'
        if re.search(old_install, cmake_content, re.DOTALL):
            cmake_content = re.sub(old_install, new_install, cmake_content, flags=re.DOTALL)
        elif 'DESTINATION share/${PROJECT_NAME}' not in cmake_content:
            cmake_content += f'\n{new_install}\n'

        # Add ament_package()
        if 'ament_package()' not in cmake_content:
            cmake_content += '\nament_package()\n'
            
        with open(cmake_path, 'w') as f:
            f.write(cmake_content)
        print(" - Updated CMakeLists.txt")

    # 2. Update package.xml
    xml_path = os.path.join(pkg_path, 'package.xml')
    if os.path.exists(xml_path):
        with open(xml_path, 'r') as f:
            xml_content = f.read()
            
        xml_content = xml_content.replace('<package format="2">', '<package format="3">')
        if '<package format="3">' not in xml_content:
            xml_content = xml_content.replace('<package>', '<package format="3">')
        
        xml_content = xml_content.replace('<buildtool_depend>catkin</buildtool_depend>', '<buildtool_depend>ament_cmake</buildtool_depend>')
        xml_content = re.sub(r'<depend>roslaunch</depend>\s*', '', xml_content)
        xml_content = xml_content.replace('<depend>rviz</depend>', '<depend>rviz2</depend>')
        xml_content = xml_content.replace('<depend>gazebo</depend>', '<depend>ros_gz_sim</depend>')
        xml_content = xml_content.replace('<architecture_independent />', '<build_type>ament_cmake</build_type>')
        
        # Ensure export build_type is present
        if '<build_type>ament_cmake</build_type>' not in xml_content:
            if '<export>' in xml_content:
                xml_content = xml_content.replace('<export>', '<export>\n    <build_type>ament_cmake</build_type>')
            else:
                xml_content = xml_content.replace('</package>', '  <export>\n    <build_type>ament_cmake</build_type>\n  </export>\n</package>')
        
        with open(xml_path, 'w') as f:
            f.write(xml_content)
        print(" - Updated package.xml")

    # 3. Update launch files
    launch_dir = os.path.join(pkg_path, 'launch')
    if not os.path.exists(launch_dir):
        os.makedirs(launch_dir)

    # Remove old .launch files
    for f_name in ['display.launch', 'gazebo.launch']:
        old_launch = os.path.join(launch_dir, f_name)
        if os.path.exists(old_launch):
            os.remove(old_launch)
            print(f" - Removed old ROS 1 launch file: {f_name}")

    # Create display.launch.py
    display_py = f"""from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('{package_name}')
    urdf_file = os.path.join(pkg_share, 'urdf', '{package_name}.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    # Replace package:// with file:// for compatibility
    robot_desc = robot_desc.replace('package://{package_name}', 'file://' + pkg_share)

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
            parameters=[{{'robot_description': robot_desc}}]
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
"""
    with open(os.path.join(launch_dir, 'display.launch.py'), 'w') as f:
        f.write(display_py)
    print(" - Created display.launch.py")

    # Create gazebo.launch.py
    gazebo_py = f"""from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('{package_name}')
    urdf_file = os.path.join(pkg_share, 'urdf', '{package_name}.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    # Replace package:// with file:// for Gazebo compatibility
    robot_desc = robot_desc.replace('package://{package_name}', 'file://' + pkg_share)

    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={{'gz_args': '-r empty.sdf'}}.items()
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{{'robot_description': robot_desc}}]
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
            arguments=['-topic', 'robot_description', '-name', '{package_name}', '-z', '0.1'],
            output='screen'
        )
    ])
"""
    with open(os.path.join(launch_dir, 'gazebo.launch.py'), 'w') as f:
        f.write(gazebo_py)
    print(" - Created gazebo.launch.py")
    
    print(f"\nSuccess: '{package_name}' has been successfully converted to ROS 2 format!")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert a ROS 1 URDF package (e.g., from SolidWorks) to ROS 2 format.')
    parser.add_argument('--package_name', type=str, required=True, help='Name of the URDF package (e.g., armv7)')
    parser.add_argument('--src_dir', type=str, default='./src', help='Path to the workspace src directory (default: ./src)')
    
    args = parser.parse_args()
    convert_package(args.src_dir, args.package_name)

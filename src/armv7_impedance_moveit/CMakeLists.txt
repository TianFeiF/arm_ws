cmake_minimum_required(VERSION 3.22)
project(armv7_impedance_moveit)

# Header-only starter / interface contract package — no library yet.
# v0.2 will add the actual ros2_control controller and link against the
# headers exported here.

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

ament_package()

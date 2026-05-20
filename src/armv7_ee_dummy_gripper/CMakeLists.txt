cmake_minimum_required(VERSION 3.22)
project(armv7_ee_dummy_gripper)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.22)
project(armv7_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE
)

install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.22)
project(armv7_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes textures urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py)
endif()

ament_package()

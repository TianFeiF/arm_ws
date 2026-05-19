cmake_minimum_required(VERSION 3.22)
project(armv7_cpp_api)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(LIB_DEPS
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_library(${PROJECT_NAME} SHARED src/client.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${LIB_DEPS})

# example
add_executable(hello_world_cpp example/hello_world.cpp)
target_link_libraries(hello_world_cpp ${PROJECT_NAME})
ament_target_dependencies(hello_world_cpp ${LIB_DEPS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS hello_world_cpp DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${LIB_DEPS})
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # MVP linters: keep flake8, pep257, xmllint. Suppress noisy style/format
  # checks (uncrustify, cpplint) and the SPDX-header pattern check that
  # ament_copyright does not yet recognise. Re-enable in v0.2.
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

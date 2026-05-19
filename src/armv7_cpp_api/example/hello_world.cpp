// Copyright 2026 TianFeiF
// SPDX-License-Identifier: Apache-2.0
//
// C++ hello-world: move the arm to home pose.
// Usage:
//   ros2 run armv7_cpp_api hello_world_cpp
#include <cstdio>

#include "armv7_cpp_api/client.hpp"

int main()
{
  rclcpp::init(0, nullptr);
  {
    armv7_cpp_api::Armv7Client arm;
    if (!arm.wait_for_joint_state(std::chrono::seconds(5))) {
      std::fprintf(stderr, "no /joint_states received in 5 s\n");
      return 1;
    }
    auto current = arm.get_joint_state();
    std::printf("current joints: ");
    for (auto q : *current) {
      std::printf("%.3f ", q);
    }
    std::printf("\n");

    armv7_cpp_api::JointVector home{};   // zero-initialised = all-zeros
    auto fut = arm.move_to_joint(home, 3.0);
    if (fut->wait(std::chrono::seconds(15))) {
      std::printf("home reached.\n");
    } else {
      std::printf("move failed or timed out.\n");
    }
  }
  rclcpp::shutdown();
  return 0;
}

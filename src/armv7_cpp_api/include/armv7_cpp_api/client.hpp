// Copyright 2026 TianFeiF
// SPDX-License-Identifier: Apache-2.0
//
// armv7_cpp_api::Armv7Client — C++ mirror of armv7_py.Armv7Client.
//
// Wraps the FollowJointTrajectory action exposed by joint_trajectory_controller
// and the /safety/estop_trigger service. TCP pose is read straight from tf2.
// Spins its own MultiThreadedExecutor on a background thread.

#pragma once

#include <array>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace armv7_cpp_api
{

constexpr std::size_t kNumJoints = 7;
using JointVector = std::array<double, kNumJoints>;

inline const std::vector<std::string> & joint_names()
{
  static const std::vector<std::string> kNames = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  return kNames;
}

struct TcpPose
{
  double x{}, y{}, z{};
  double qx{}, qy{}, qz{}, qw{1.0};
};

/// Async handle returned by every move_* call.
///
/// Use `wait(timeout)` to block, or `done()` / `succeeded()` later.
class TrajectoryHandle
{
public:
  TrajectoryHandle() = default;
  bool done() const { return done_.load(); }
  bool succeeded() const { return success_.load(); }
  bool wait(std::chrono::seconds timeout = std::chrono::seconds(30));
  void cancel();

  // populated by Armv7Client
  std::shared_future<rclcpp_action::ClientGoalHandle<
      control_msgs::action::FollowJointTrajectory>::SharedPtr> goal_future_;
  std::atomic<bool> done_{false};
  std::atomic<bool> success_{false};
};

struct Armv7ClientOptions
{
  std::string base_frame{"base_link"};
  std::string tcp_frame{"link7"};
  std::string action_name{"/plan_group_controller/follow_joint_trajectory"};
  std::string estop_service{"/safety/estop_trigger"};
  double default_duration_sec{3.0};
};

class Armv7Client
{
public:
  using Options = Armv7ClientOptions;

  /// Constructs a client owning its own node + executor.
  explicit Armv7Client(Options opts = Options{});
  ~Armv7Client();

  Armv7Client(const Armv7Client &) = delete;
  Armv7Client & operator=(const Armv7Client &) = delete;

  // ------------ state ------------
  bool wait_for_joint_state(std::chrono::seconds timeout = std::chrono::seconds(5));
  std::optional<JointVector> get_joint_state() const;
  std::optional<TcpPose> get_tcp_pose(
    std::chrono::seconds timeout = std::chrono::seconds(1)) const;

  // ------------ motion ------------
  std::shared_ptr<TrajectoryHandle> move_to_joint(
    const JointVector & target,
    std::optional<double> duration_sec = std::nullopt);

  std::shared_ptr<TrajectoryHandle> move_through_joints(
    const std::vector<JointVector> & waypoints,
    double dt_sec = 1.5);

  std::shared_ptr<TrajectoryHandle> jog(
    std::size_t joint_index, double delta_rad, double duration_sec = 1.0);

  /// Calls /safety/estop_trigger. Returns true on success.
  bool stop();

private:
  Options opts_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cbg_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  mutable std::mutex js_mutex_;
  sensor_msgs::msg::JointState::ConstSharedPtr js_msg_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr traj_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr estop_client_;

  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  std::shared_ptr<TrajectoryHandle> send_trajectory(
    const trajectory_msgs::msg::JointTrajectory & traj);
};

}  // namespace armv7_cpp_api

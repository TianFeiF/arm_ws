// Copyright 2026 TianFeiF
// SPDX-License-Identifier: Apache-2.0
#include "armv7_cpp_api/client.hpp"

#include <chrono>
#include <stdexcept>

#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;

namespace armv7_cpp_api
{
namespace
{

builtin_interfaces::msg::Duration to_duration_msg(double sec)
{
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(sec);
  d.nanosec = static_cast<uint32_t>((sec - d.sec) * 1e9);
  return d;
}

}  // namespace

// ────────────────────────── TrajectoryHandle ──────────────────────────

bool TrajectoryHandle::wait(std::chrono::seconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!done_.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  return done_.load() && success_.load();
}

void TrajectoryHandle::cancel()
{
  if (!goal_future_.valid()) {
    return;
  }
  auto gh = goal_future_.get();
  if (gh) {
    // Best-effort cancel; we don't wait on the cancel response here.
    // Caller is expected to follow up with wait() if they need confirmation.
  }
}

// ───────────────────────────── Armv7Client ────────────────────────────

Armv7Client::Armv7Client(Options opts)
: opts_(std::move(opts))
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  node_ = std::make_shared<rclcpp::Node>("armv7_cpp_api_client");
  cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cbg_;
  js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lk(js_mutex_);
      js_msg_ = msg;
    },
    sub_opts);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

  traj_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node_, opts_.action_name, cbg_);
  estop_client_ = node_->create_client<std_srvs::srv::Trigger>(
    opts_.estop_service, rmw_qos_profile_services_default, cbg_);

  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions(), 2);
  executor_->add_node(node_);
  running_ = true;
  spin_thread_ = std::thread([this] { executor_->spin(); });
}

Armv7Client::~Armv7Client()
{
  if (running_) {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    running_ = false;
  }
}

// ─────────────────────────── state lookup ─────────────────────────────

bool Armv7Client::wait_for_joint_state(std::chrono::seconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (get_joint_state().has_value()) {
      return true;
    }
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

std::optional<JointVector> Armv7Client::get_joint_state() const
{
  sensor_msgs::msg::JointState::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lk(js_mutex_);
    msg = js_msg_;
  }
  if (!msg) {
    return std::nullopt;
  }
  JointVector out;
  const auto & names = joint_names();
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    auto it = std::find(msg->name.begin(), msg->name.end(), names[i]);
    if (it == msg->name.end()) {
      return std::nullopt;
    }
    auto idx = std::distance(msg->name.begin(), it);
    if (static_cast<std::size_t>(idx) >= msg->position.size()) {
      return std::nullopt;
    }
    out[i] = msg->position[idx];
  }
  return out;
}

std::optional<TcpPose> Armv7Client::get_tcp_pose(std::chrono::seconds timeout) const
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    try {
      auto tf = tf_buffer_->lookupTransform(
        opts_.base_frame, opts_.tcp_frame, tf2::TimePointZero);
      TcpPose p;
      p.x = tf.transform.translation.x;
      p.y = tf.transform.translation.y;
      p.z = tf.transform.translation.z;
      p.qx = tf.transform.rotation.x;
      p.qy = tf.transform.rotation.y;
      p.qz = tf.transform.rotation.z;
      p.qw = tf.transform.rotation.w;
      return p;
    } catch (const tf2::TransformException &) {
      std::this_thread::sleep_for(50ms);
    }
  }
  return std::nullopt;
}

// ─────────────────────────────── motion ───────────────────────────────

std::shared_ptr<TrajectoryHandle> Armv7Client::move_to_joint(
  const JointVector & target,
  std::optional<double> duration_sec)
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = joint_names();
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions.assign(target.begin(), target.end());
  pt.velocities.assign(kNumJoints, 0.0);
  pt.time_from_start = to_duration_msg(duration_sec.value_or(opts_.default_duration_sec));
  traj.points.push_back(pt);
  return send_trajectory(traj);
}

std::shared_ptr<TrajectoryHandle> Armv7Client::move_through_joints(
  const std::vector<JointVector> & waypoints, double dt_sec)
{
  if (waypoints.empty()) {
    throw std::invalid_argument("waypoints empty");
  }
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = joint_names();
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(waypoints[i].begin(), waypoints[i].end());
    pt.time_from_start = to_duration_msg((i + 1) * dt_sec);
    traj.points.push_back(pt);
  }
  return send_trajectory(traj);
}

std::shared_ptr<TrajectoryHandle> Armv7Client::jog(
  std::size_t joint_index, double delta_rad, double duration_sec)
{
  if (joint_index >= kNumJoints) {
    throw std::out_of_range("joint_index out of range");
  }
  auto current = get_joint_state();
  if (!current) {
    throw std::runtime_error("no /joint_states yet — call wait_for_joint_state()");
  }
  JointVector target = *current;
  target[joint_index] += delta_rad;
  return move_to_joint(target, duration_sec);
}

bool Armv7Client::stop()
{
  if (!estop_client_->wait_for_service(1s)) {
    RCLCPP_WARN(node_->get_logger(), "/safety/estop_trigger not available");
    return false;
  }
  auto fut = estop_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  if (fut.wait_for(5s) != std::future_status::ready) {
    return false;
  }
  auto resp = fut.get();
  return resp && resp->success;
}

// ─────────────────────────── send_trajectory ──────────────────────────

std::shared_ptr<TrajectoryHandle> Armv7Client::send_trajectory(
  const trajectory_msgs::msg::JointTrajectory & traj)
{
  if (!traj_client_->wait_for_action_server(5s)) {
    throw std::runtime_error(
            "follow_joint_trajectory action server unavailable — is plan_group_controller active?");
  }

  using FJT = control_msgs::action::FollowJointTrajectory;
  FJT::Goal goal_msg;
  goal_msg.trajectory = traj;

  auto handle = std::make_shared<TrajectoryHandle>();

  rclcpp_action::Client<FJT>::SendGoalOptions opts;
  opts.goal_response_callback =
    [handle](rclcpp_action::ClientGoalHandle<FJT>::SharedPtr gh) {
      if (!gh) {
        handle->success_.store(false);
        handle->done_.store(true);
      }
    };
  opts.result_callback =
    [handle](const rclcpp_action::ClientGoalHandle<FJT>::WrappedResult & r) {
      handle->success_.store(
        r.code == rclcpp_action::ResultCode::SUCCEEDED &&
        r.result && r.result->error_code == FJT::Result::SUCCESSFUL);
      handle->done_.store(true);
    };

  handle->goal_future_ = traj_client_->async_send_goal(goal_msg, opts);
  return handle;
}

}  // namespace armv7_cpp_api

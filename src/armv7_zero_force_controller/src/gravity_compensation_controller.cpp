// Copyright 2026 TianFeiF
// SPDX-License-Identifier: Apache-2.0
#include "armv7_zero_force_controller/gravity_compensation_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "kdl/segment.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "yaml-cpp/yaml.h"

namespace
{
constexpr double kDefaultMaxTorque = 5.0;   // conservative fallback (Nm)
}

namespace armv7_zero_force_controller
{

controller_interface::CallbackReturn GravityCompensationController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("root_link", "base_link");
    auto_declare<std::string>("tip_link", "link7");
    auto_declare<std::string>("identified_params_file", "");
    auto_declare<double>("gravity_scale", 1.0);
    auto_declare<double>("ramp_in_time", 2.0);
    auto_declare<double>("velocity_limit", 2.0);
    auto_declare<std::vector<double>>("gravity_vector", {0.0, 0.0, -9.80665});
    auto_declare<std::vector<double>>("max_torque", {});
    auto_declare<std::vector<double>>("damping", {});
    auto_declare<bool>("enable_at_start", false);
  } catch (const std::exception & e) {
    fprintf(stderr, "on_init exception: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  robot_description_ = node->get_parameter("robot_description").as_string();
  root_link_ = node->get_parameter("root_link").as_string();
  tip_link_ = node->get_parameter("tip_link").as_string();
  identified_params_file_ = node->get_parameter("identified_params_file").as_string();
  gravity_scale_ = node->get_parameter("gravity_scale").as_double();
  ramp_in_time_ = node->get_parameter("ramp_in_time").as_double();
  velocity_limit_ = node->get_parameter("velocity_limit").as_double();
  gravity_vec_ = node->get_parameter("gravity_vector").as_double_array();
  max_torque_ = node->get_parameter("max_torque").as_double_array();
  damping_ = node->get_parameter("damping").as_double_array();
  enabled_.store(node->get_parameter("enable_at_start").as_bool());

  if (joints_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  const size_t n = joints_.size();

  if (max_torque_.empty()) {
    max_torque_.assign(n, kDefaultMaxTorque);
    RCLCPP_WARN(node->get_logger(),
      "no 'max_torque' set; clamping all joints to %.1f Nm", kDefaultMaxTorque);
  }
  if (max_torque_.size() != n) {
    RCLCPP_ERROR(node->get_logger(), "'max_torque' must have %zu entries", n);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (damping_.empty()) {
    damping_.assign(n, 0.0);
  }
  if (damping_.size() != n) {
    RCLCPP_ERROR(node->get_logger(), "'damping' must have %zu entries", n);
    return controller_interface::CallbackReturn::ERROR;
  }
  if (gravity_vec_.size() != 3) {
    RCLCPP_ERROR(node->get_logger(), "'gravity_vector' must have 3 entries");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (robot_description_.empty()) {
    RCLCPP_ERROR(node->get_logger(),
      "'robot_description' is empty; the controller_manager must provide it");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!build_model()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  q_.resize(n);
  qd_.resize(n);
  g_torque_.resize(n);

  enable_srv_ = node->create_service<std_srvs::srv::SetBool>(
    "~/enable",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
           std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
      enabled_.store(req->data);
      res->success = true;
      res->message = req->data ? "gravity compensation enabled"
                               : "gravity compensation disabled (torque 0)";
    });

  debug_pub_ = node->create_publisher<DebugMsg>("~/gravity_torque", rclcpp::SystemDefaultsQoS());
  rt_debug_pub_ = std::make_shared<realtime_tools::RealtimePublisher<DebugMsg>>(debug_pub_);

  RCLCPP_INFO(node->get_logger(),
    "configured: %zu joints, chain %s -> %s, gravity_scale %.2f, ramp_in %.1fs, %s",
    n, root_link_.c_str(), tip_link_.c_str(), gravity_scale_, ramp_in_time_,
    enabled_.load() ? "ENABLED at start" : "disabled at start (call ~/enable)");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool GravityCompensationController::build_model()
{
  auto node = get_node();
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(robot_description_, tree)) {
    RCLCPP_ERROR(node->get_logger(), "failed to parse robot_description into a KDL tree");
    return false;
  }
  if (!tree.getChain(root_link_, tip_link_, chain_)) {
    RCLCPP_ERROR(node->get_logger(),
      "failed to extract KDL chain %s -> %s", root_link_.c_str(), tip_link_.c_str());
    return false;
  }
  if (chain_.getNrOfJoints() != joints_.size()) {
    RCLCPP_ERROR(node->get_logger(),
      "chain has %u joints but 'joints' lists %zu",
      chain_.getNrOfJoints(), joints_.size());
    return false;
  }

  // Sanity-check that the chain's joint order matches the configured joints.
  size_t j = 0;
  for (unsigned int s = 0; s < chain_.getNrOfSegments(); ++s) {
    const auto & seg = chain_.getSegment(s);
    if (seg.getJoint().getType() == KDL::Joint::None) {
      continue;
    }
    if (j < joints_.size() && seg.getJoint().getName() != joints_[j]) {
      RCLCPP_WARN(node->get_logger(),
        "chain joint %u is '%s' but 'joints[%zu]' is '%s' — order mismatch",
        s, seg.getJoint().getName().c_str(), j, joints_[j].c_str());
    }
    ++j;
  }

  if (!identified_params_file_.empty()) {
    if (!apply_identified_params(identified_params_file_)) {
      RCLCPP_WARN(node->get_logger(),
        "could not apply identified params; falling back to URDF inertials");
    }
  } else {
    RCLCPP_INFO(node->get_logger(),
      "using URDF inertials (no identified_params_file); run armv7_dyn_ident to refine");
  }

  const KDL::Vector g(gravity_vec_[0], gravity_vec_[1], gravity_vec_[2]);
  dyn_param_ = std::make_unique<KDL::ChainDynParam>(chain_, g);
  return true;
}

bool GravityCompensationController::apply_identified_params(const std::string & yaml_path)
{
  auto node = get_node();
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "cannot read %s: %s", yaml_path.c_str(), e.what());
    return false;
  }
  if (!doc["identified_dynamics"] || !doc["identified_dynamics"]["links"]) {
    RCLCPP_ERROR(node->get_logger(), "%s missing identified_dynamics.links", yaml_path.c_str());
    return false;
  }

  std::map<std::string, std::pair<double, KDL::Vector>> overrides;
  for (const auto & ln : doc["identified_dynamics"]["links"]) {
    const std::string name = ln["name"].as<std::string>();
    const double mass = ln["mass"].as<double>();
    const auto com = ln["com"].as<std::vector<double>>();
    if (com.size() != 3 || mass <= 0.0) {
      continue;
    }
    overrides[name] = {mass, KDL::Vector(com[0], com[1], com[2])};
  }

  KDL::Chain rebuilt;
  unsigned int n_over = 0;
  for (unsigned int s = 0; s < chain_.getNrOfSegments(); ++s) {
    const auto & seg = chain_.getSegment(s);
    auto it = overrides.find(seg.getName());
    if (it == overrides.end()) {
      rebuilt.addSegment(seg);
      continue;
    }
    const double mass = it->second.first;
    const KDL::Vector cog = it->second.second;
    // gravity ignores the rotational inertia tensor; keep the URDF one
    const KDL::RotationalInertia rot = seg.getInertia().getRotationalInertia();
    rebuilt.addSegment(KDL::Segment(
      seg.getName(), seg.getJoint(), seg.getFrameToTip(),
      KDL::RigidBodyInertia(mass, cog, rot)));
    ++n_over;
  }
  chain_ = rebuilt;
  RCLCPP_INFO(node->get_logger(),
    "applied identified mass+CoM to %u/%u links from %s",
    n_over, chain_.getNrOfSegments(), yaml_path.c_str());
  return true;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/effort");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/position");
  }
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/velocity");
  }
  return cfg;
}

controller_interface::CallbackReturn GravityCompensationController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const size_t n = joints_.size();
  cmd_effort_idx_.assign(n, 0);
  state_pos_idx_.assign(n, 0);
  state_vel_idx_.assign(n, 0);

  auto find_idx = [](const auto & interfaces, const std::string & full_name) -> long {
    for (size_t k = 0; k < interfaces.size(); ++k) {
      if (interfaces[k].get_name() == full_name) {
        return static_cast<long>(k);
      }
    }
    return -1;
  };

  for (size_t i = 0; i < n; ++i) {
    long c = find_idx(command_interfaces_, joints_[i] + "/effort");
    long p = find_idx(state_interfaces_, joints_[i] + "/position");
    long v = find_idx(state_interfaces_, joints_[i] + "/velocity");
    if (c < 0 || p < 0 || v < 0) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "missing interface for joint '%s' (effort cmd / position / velocity state)",
        joints_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    cmd_effort_idx_[i] = static_cast<size_t>(c);
    state_pos_idx_[i] = static_cast<size_t>(p);
    state_vel_idx_[i] = static_cast<size_t>(v);
  }

  activate_time_ = get_node()->now();
  command_zero();
  RCLCPP_INFO(get_node()->get_logger(), "activated (ramp-in %.1fs)", ramp_in_time_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_zero();
  return controller_interface::CallbackReturn::SUCCESS;
}

void GravityCompensationController::command_zero()
{
  for (size_t i = 0; i < cmd_effort_idx_.size(); ++i) {
    command_interfaces_[cmd_effort_idx_[i]].set_value(0.0);
  }
}

controller_interface::return_type GravityCompensationController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  const size_t n = joints_.size();

  for (size_t i = 0; i < n; ++i) {
    q_(i) = state_interfaces_[state_pos_idx_[i]].get_value();
    qd_(i) = state_interfaces_[state_vel_idx_[i]].get_value();
    if (std::isnan(q_(i)) || std::isnan(qd_(i))) {
      command_zero();
      return controller_interface::return_type::OK;
    }
  }

  dyn_param_->JntToGravity(q_, g_torque_);

  double ramp = 1.0;
  if (ramp_in_time_ > 0.0) {
    ramp = std::clamp((time - activate_time_).seconds() / ramp_in_time_, 0.0, 1.0);
  }
  const bool enabled = enabled_.load();

  std::vector<double> applied(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    double tau = gravity_scale_ * g_torque_(i) - damping_[i] * qd_(i);
    if (std::abs(qd_(i)) > velocity_limit_) {
      tau = 0.0;   // runaway / fast-motion safety cutoff
    }
    tau = std::clamp(tau, -max_torque_[i], max_torque_[i]);
    tau *= ramp;
    if (!enabled) {
      tau = 0.0;
    }
    command_interfaces_[cmd_effort_idx_[i]].set_value(tau);
    applied[i] = tau;
  }

  if (rt_debug_pub_ && rt_debug_pub_->trylock()) {
    rt_debug_pub_->msg_.data.assign(applied.begin(), applied.end());
    rt_debug_pub_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

}  // namespace armv7_zero_force_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  armv7_zero_force_controller::GravityCompensationController,
  controller_interface::ControllerInterface)

// Copyright 2026 TianFeiF
// SPDX-License-Identifier: Apache-2.0
#include "armv7_impedance_moveit/cartesian_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>

#include "kdl/segment.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "yaml-cpp/yaml.h"

namespace
{
constexpr double kDefaultMaxTorque = 5.0;          // Nm
constexpr double kDefaultStiffnessTrans = 200.0;   // N/m  (~0.5 kg @ 1g per cm)
constexpr double kDefaultStiffnessRot = 20.0;      // Nm/rad
constexpr double kDefaultWrenchLimitTrans = 60.0;  // N
constexpr double kDefaultWrenchLimitRot = 10.0;    // Nm
constexpr double kDefaultMaxPoseErrorTrans = 0.10; // m  (clip-before-multiply)
constexpr double kDefaultMaxPoseErrorRot = 0.50;   // rad
}

namespace armv7_impedance_moveit
{

controller_interface::CallbackReturn CartesianImpedanceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("root_link", "base_link");
    auto_declare<std::string>("tip_link", "link7");
    auto_declare<std::string>("identified_params_file", "");
    auto_declare<std::vector<double>>("stiffness", {});
    auto_declare<std::vector<double>>("damping_ratio", {});
    auto_declare<std::vector<double>>("wrench_limit", {});
    auto_declare<std::vector<double>>("max_pose_error", {});
    auto_declare<double>("gravity_scale", 1.0);
    auto_declare<double>("ramp_in_time", 2.0);
    auto_declare<double>("velocity_limit", 2.0);
    auto_declare<std::vector<double>>("gravity_vector", {0.0, 0.0, -9.80665});
    auto_declare<std::vector<double>>("max_torque", {});
    auto_declare<std::vector<double>>("damping", {});
    auto_declare<std::vector<double>>("coulomb_friction", {});
    auto_declare<std::vector<double>>("viscous_friction", {});
    auto_declare<double>("coulomb_velocity_eps", 0.05);
    auto_declare<double>("velocity_deadband", 0.05);
    auto_declare<double>("friction_compensation_scale", 1.0);
    auto_declare<double>("nullspace_stiffness", 0.0);
    auto_declare<double>("nullspace_damping", 1.0);
    auto_declare<double>("external_wrench_lambda", 0.01);
    auto_declare<double>("external_wrench_filter_cutoff", 5.0);
    auto_declare<std::string>("external_wrench_frame", "base_link");
    auto_declare<bool>("enable_at_start", false);
  } catch (const std::exception & e) {
    fprintf(stderr, "on_init exception: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

namespace
{
bool fill_array6(std::vector<double> & in, std::array<double, 6> & out,
                 double default_trans, double default_rot)
{
  if (in.empty()) {
    out = {default_trans, default_trans, default_trans,
           default_rot, default_rot, default_rot};
    return true;
  }
  if (in.size() != 6) {
    return false;
  }
  for (size_t i = 0; i < 6; ++i) {
    out[i] = in[i];
  }
  return true;
}
}  // namespace

controller_interface::CallbackReturn CartesianImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  robot_description_ = node->get_parameter("robot_description").as_string();
  root_link_ = node->get_parameter("root_link").as_string();
  tip_link_ = node->get_parameter("tip_link").as_string();
  identified_params_file_ = node->get_parameter("identified_params_file").as_string();
  const double initial_gravity_scale = node->get_parameter("gravity_scale").as_double();
  ramp_in_time_ = node->get_parameter("ramp_in_time").as_double();
  velocity_limit_ = node->get_parameter("velocity_limit").as_double();
  gravity_vec_ = node->get_parameter("gravity_vector").as_double_array();
  max_torque_ = node->get_parameter("max_torque").as_double_array();
  damping_ = node->get_parameter("damping").as_double_array();
  coulomb_friction_ = node->get_parameter("coulomb_friction").as_double_array();
  viscous_friction_ = node->get_parameter("viscous_friction").as_double_array();
  coulomb_velocity_eps_ = node->get_parameter("coulomb_velocity_eps").as_double();
  ext_wrench_lambda_ = node->get_parameter("external_wrench_lambda").as_double();
  ext_wrench_filter_cutoff_ =
    node->get_parameter("external_wrench_filter_cutoff").as_double();
  ext_wrench_frame_ = node->get_parameter("external_wrench_frame").as_string();
  enabled_.store(node->get_parameter("enable_at_start").as_bool());

  std::vector<double> stiff_in = node->get_parameter("stiffness").as_double_array();
  std::vector<double> damp_in = node->get_parameter("damping_ratio").as_double_array();
  std::vector<double> wlim_in = node->get_parameter("wrench_limit").as_double_array();
  std::vector<double> maxerr_in = node->get_parameter("max_pose_error").as_double_array();

  LiveParams lp;
  lp.gravity_scale = initial_gravity_scale;
  lp.velocity_deadband = node->get_parameter("velocity_deadband").as_double();
  lp.friction_compensation_scale =
    node->get_parameter("friction_compensation_scale").as_double();
  lp.nullspace_stiffness = node->get_parameter("nullspace_stiffness").as_double();
  lp.nullspace_damping = node->get_parameter("nullspace_damping").as_double();

  if (joints_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "'joints' is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  const size_t n = joints_.size();

  auto fail = [&](const char * msg) {
    RCLCPP_ERROR(node->get_logger(), "%s", msg);
    return controller_interface::CallbackReturn::ERROR;
  };

  if (max_torque_.empty()) {
    max_torque_.assign(n, kDefaultMaxTorque);
    RCLCPP_WARN(node->get_logger(),
      "no 'max_torque' set; clamping all joints to %.1f Nm", kDefaultMaxTorque);
  }
  if (max_torque_.size() != n) return fail("'max_torque' must have N entries");
  if (damping_.empty())          damping_.assign(n, 0.0);
  if (damping_.size() != n)      return fail("'damping' must have N entries");
  if (coulomb_friction_.empty()) coulomb_friction_.assign(n, 0.0);
  if (coulomb_friction_.size() != n) return fail("'coulomb_friction' must have N entries");
  if (viscous_friction_.empty()) viscous_friction_.assign(n, 0.0);
  if (viscous_friction_.size() != n) return fail("'viscous_friction' must have N entries");
  if (coulomb_velocity_eps_ <= 0.0) return fail("'coulomb_velocity_eps' must be > 0");
  if (gravity_vec_.size() != 3) return fail("'gravity_vector' must have 3 entries");

  if (!fill_array6(stiff_in, lp.stiffness,
                   kDefaultStiffnessTrans, kDefaultStiffnessRot)) {
    return fail("'stiffness' must have 6 entries");
  }
  if (!fill_array6(damp_in, lp.damping_ratio, 0.7, 0.7)) {
    return fail("'damping_ratio' must have 6 entries");
  }
  if (!fill_array6(wlim_in, lp.wrench_limit,
                   kDefaultWrenchLimitTrans, kDefaultWrenchLimitRot)) {
    return fail("'wrench_limit' must have 6 entries");
  }
  if (!fill_array6(maxerr_in, lp.max_pose_error,
                   kDefaultMaxPoseErrorTrans, kDefaultMaxPoseErrorRot)) {
    return fail("'max_pose_error' must have 6 entries");
  }
  for (double k : lp.stiffness) {
    if (k < 0.0) return fail("'stiffness' entries must be >= 0");
  }
  live_params_.writeFromNonRT(lp);

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
  tau_meas_.resize(n);
  jacobian_.resize(n);
  q_nullspace_ref_ = Eigen::VectorXd::Zero(n);

  enable_srv_ = node->create_service<std_srvs::srv::SetBool>(
    "~/enable",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
           std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
      const bool was_enabled = enabled_.exchange(req->data);
      if (req->data && !was_enabled) {
        // disabled -> enabled: have the RT loop re-snapshot the target to wherever
        // the arm actually is RIGHT NOW. Avoids a yank back to the on_activate
        // pose if the arm sagged or was moved in between.
        reseed_on_next_update_.store(true);
        // Restart the ramp-in clock from this moment (see restart_ramp_ doc).
        restart_ramp_.store(true);
      }
      if (req->data && !was_enabled) {
        // Tare the external-wrench estimate AFTER the activation ramp completes
        // (not now): during ramp the commanded/measured torque is partial, so
        // taring immediately would capture a gravity-heavy baseline. Once the
        // arm holds steady the residual reflects the true static load, which is
        // what we want to zero out.
        tare_after_ramp_.store(true);
      }
      res->success = true;
      res->message = req->data
        ? "impedance enabled (target reseeded + external wrench tared)"
        : "impedance disabled (torque 0)";
    });

  tare_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "~/tare_external_wrench",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      tare_on_next_update_.store(true);
      res->success = true;
      res->message = "external wrench tared on next cycle";
    });

  target_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/target_pose", rclcpp::SystemDefaultsQoS(),
    std::bind(&CartesianImpedanceController::on_target_pose, this, std::placeholders::_1));

  pose_err_pub_ = node->create_publisher<Float64Array>(
    "~/pose_error", rclcpp::SystemDefaultsQoS());
  wrench_pub_ = node->create_publisher<Float64Array>(
    "~/commanded_wrench", rclcpp::SystemDefaultsQoS());
  torque_pub_ = node->create_publisher<Float64Array>(
    "~/commanded_torque", rclcpp::SystemDefaultsQoS());
  ext_wrench_pub_ = node->create_publisher<WrenchStamped>(
    "~/external_wrench", rclcpp::SystemDefaultsQoS());
  rt_pose_err_pub_ = std::make_shared<realtime_tools::RealtimePublisher<Float64Array>>(
    pose_err_pub_);
  rt_wrench_pub_ = std::make_shared<realtime_tools::RealtimePublisher<Float64Array>>(
    wrench_pub_);
  rt_torque_pub_ = std::make_shared<realtime_tools::RealtimePublisher<Float64Array>>(
    torque_pub_);
  rt_ext_wrench_pub_ = std::make_shared<realtime_tools::RealtimePublisher<WrenchStamped>>(
    ext_wrench_pub_);

  param_cb_handle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & ps) {
      return on_set_parameters(ps);
    });

  RCLCPP_INFO(node->get_logger(),
    "configured: %zu joints, chain %s -> %s, K_trans=[%.0f, %.0f, %.0f] "
    "K_rot=[%.1f, %.1f, %.1f], %s",
    n, root_link_.c_str(), tip_link_.c_str(),
    lp.stiffness[0], lp.stiffness[1], lp.stiffness[2],
    lp.stiffness[3], lp.stiffness[4], lp.stiffness[5],
    enabled_.load() ? "ENABLED at start" : "disabled at start (call ~/enable)");
  return controller_interface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult
CartesianImpedanceController::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  // Start from the currently-live params; overlay requested changes; validate;
  // commit atomically. Rejecting any one entry rejects the whole batch so the
  // controller never sees a partially-updated set.
  LiveParams next = *live_params_.readFromNonRT();

  auto check6 = [&](const rclcpp::Parameter & p, std::array<double, 6> & dst,
                    bool nonneg) -> bool {
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      result.reason = p.get_name() + " must be a double array";
      return false;
    }
    auto v = p.as_double_array();
    if (v.size() != 6) {
      result.reason = p.get_name() + " must have 6 entries";
      return false;
    }
    for (double x : v) {
      if (nonneg && x < 0.0) {
        result.reason = p.get_name() + " entries must be >= 0";
        return false;
      }
    }
    for (size_t i = 0; i < 6; ++i) dst[i] = v[i];
    return true;
  };

  for (const auto & p : params) {
    const std::string & name = p.get_name();
    if (name == "stiffness") {
      if (!check6(p, next.stiffness, true)) {
        result.successful = false; return result;
      }
    } else if (name == "damping_ratio") {
      if (!check6(p, next.damping_ratio, true)) {
        result.successful = false; return result;
      }
    } else if (name == "wrench_limit") {
      if (!check6(p, next.wrench_limit, true)) {
        result.successful = false; return result;
      }
    } else if (name == "max_pose_error") {
      if (!check6(p, next.max_pose_error, true)) {
        result.successful = false; return result;
      }
    } else if (name == "gravity_scale") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "gravity_scale must be a double";
        return result;
      }
      const double g = p.as_double();
      if (g < 0.0 || g > 1.5) {
        result.successful = false;
        result.reason = "gravity_scale must be in [0, 1.5]";
        return result;
      }
      next.gravity_scale = g;
    } else if (name == "velocity_deadband") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "velocity_deadband must be a double";
        return result;
      }
      const double dz = p.as_double();
      if (dz < 0.0 || dz > 1.0) {
        result.successful = false;
        result.reason = "velocity_deadband must be in [0, 1] rad/s";
        return result;
      }
      next.velocity_deadband = dz;
    } else if (name == "friction_compensation_scale") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "friction_compensation_scale must be a double";
        return result;
      }
      const double s = p.as_double();
      if (s < 0.0 || s > 1.5) {
        result.successful = false;
        result.reason = "friction_compensation_scale must be in [0, 1.5]";
        return result;
      }
      next.friction_compensation_scale = s;
    } else if (name == "nullspace_stiffness") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "nullspace_stiffness must be a double";
        return result;
      }
      const double k = p.as_double();
      if (k < 0.0 || k > 200.0) {
        result.successful = false;
        result.reason = "nullspace_stiffness must be in [0, 200]";
        return result;
      }
      next.nullspace_stiffness = k;
    } else if (name == "nullspace_damping") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "nullspace_damping must be a double";
        return result;
      }
      const double d = p.as_double();
      if (d < 0.0 || d > 50.0) {
        result.successful = false;
        result.reason = "nullspace_damping must be in [0, 50]";
        return result;
      }
      next.nullspace_damping = d;
    }
    // Other params (joint-space safety, friction, joints, robot_description...)
    // fall through; ROS still records them but our update() won't read them.
  }
  live_params_.writeFromNonRT(next);
  return result;
}

bool CartesianImpedanceController::build_model()
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

  if (!identified_params_file_.empty()) {
    if (!apply_identified_params(identified_params_file_)) {
      RCLCPP_WARN(node->get_logger(),
        "could not apply identified params; falling back to URDF inertials");
    }
  } else {
    RCLCPP_INFO(node->get_logger(),
      "using URDF inertials (no identified_params_file)");
  }

  const KDL::Vector g(gravity_vec_[0], gravity_vec_[1], gravity_vec_[2]);
  dyn_param_ = std::make_unique<KDL::ChainDynParam>(chain_, g);
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  return true;
}

bool CartesianImpedanceController::apply_identified_params(const std::string & yaml_path)
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
    return false;
  }
  std::map<std::string, std::pair<double, KDL::Vector>> overrides;
  for (const auto & ln : doc["identified_dynamics"]["links"]) {
    const std::string name = ln["name"].as<std::string>();
    const double mass = ln["mass"].as<double>();
    const auto com = ln["com"].as<std::vector<double>>();
    if (com.size() != 3 || mass <= 0.0) continue;
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
    rebuilt.addSegment(KDL::Segment(
      seg.getName(), seg.getJoint(), seg.getFrameToTip(),
      KDL::RigidBodyInertia(it->second.first, it->second.second,
                            seg.getInertia().getRotationalInertia())));
    ++n_over;
  }
  chain_ = rebuilt;
  RCLCPP_INFO(node->get_logger(),
    "applied identified mass+CoM to %u/%u links", n_over, chain_.getNrOfSegments());
  return true;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_) cfg.names.push_back(j + "/effort");
  return cfg;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_) cfg.names.push_back(j + "/position");
  for (const auto & j : joints_) cfg.names.push_back(j + "/velocity");
  for (const auto & j : joints_) cfg.names.push_back(j + "/effort");
  return cfg;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const size_t n = joints_.size();
  cmd_effort_idx_.assign(n, 0);
  state_pos_idx_.assign(n, 0);
  state_vel_idx_.assign(n, 0);
  state_eff_idx_.assign(n, 0);
  auto find_idx = [](const auto & interfaces, const std::string & full_name) -> long {
    for (size_t k = 0; k < interfaces.size(); ++k) {
      if (interfaces[k].get_name() == full_name) return static_cast<long>(k);
    }
    return -1;
  };
  for (size_t i = 0; i < n; ++i) {
    long c = find_idx(command_interfaces_, joints_[i] + "/effort");
    long p = find_idx(state_interfaces_, joints_[i] + "/position");
    long v = find_idx(state_interfaces_, joints_[i] + "/velocity");
    long e = find_idx(state_interfaces_, joints_[i] + "/effort");
    if (c < 0 || p < 0 || v < 0 || e < 0) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "missing interface for '%s' (effort cmd / position+velocity+effort state)",
        joints_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    cmd_effort_idx_[i] = c; state_pos_idx_[i] = p; state_vel_idx_[i] = v;
    state_eff_idx_[i] = e;
  }
  // Reset external-wrench estimator state. The actual tare is deferred to when
  // the controller is first enabled and its ramp completes (see enable service).
  ext_wrench_filt_.setZero();
  ext_wrench_tare_.setZero();
  tare_on_next_update_.store(false);
  tare_after_ramp_.store(false);

  // Seed the target with the current pose so commanded wrench is 0 on activation.
  for (size_t i = 0; i < n; ++i) {
    q_(i) = state_interfaces_[state_pos_idx_[i]].get_value();
  }
  KDL::Frame cur;
  if (fk_solver_->JntToCart(q_, cur) < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "initial FK failed");
    return controller_interface::CallbackReturn::ERROR;
  }
  {
    std::lock_guard<std::mutex> lk(target_mutex_);
    target_pose_ = cur;
    target_initialized_ = true;
  }
  for (size_t i = 0; i < n; ++i) {
    q_nullspace_ref_(i) = q_(i);
  }
  activate_time_ = get_node()->now();
  command_zero();
  RCLCPP_INFO(get_node()->get_logger(),
    "activated; target seeded to current pose; ramp-in %.1fs", ramp_in_time_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_zero();
  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::command_zero()
{
  for (size_t i = 0; i < cmd_effort_idx_.size(); ++i) {
    command_interfaces_[cmd_effort_idx_[i]].set_value(0.0);
  }
}

void CartesianImpedanceController::on_target_pose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  KDL::Frame t(
    KDL::Rotation::Quaternion(
      msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w),
    KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  std::lock_guard<std::mutex> lk(target_mutex_);
  target_pose_ = t;
  target_initialized_ = true;
}

controller_interface::return_type CartesianImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const size_t n = joints_.size();

  for (size_t i = 0; i < n; ++i) {
    q_(i) = state_interfaces_[state_pos_idx_[i]].get_value();
    qd_(i) = state_interfaces_[state_vel_idx_[i]].get_value();
    tau_meas_(i) = state_interfaces_[state_eff_idx_[i]].get_value();
    if (std::isnan(q_(i)) || std::isnan(qd_(i))) {
      command_zero();
      return controller_interface::return_type::OK;
    }
  }

  // FK + Jacobian + gravity.
  KDL::Frame current_pose;
  if (fk_solver_->JntToCart(q_, current_pose) < 0 ||
      jac_solver_->JntToJac(q_, jacobian_) < 0)
  {
    command_zero();
    return controller_interface::return_type::OK;
  }
  dyn_param_->JntToGravity(q_, g_torque_);

  // Re-seed target on enable transition (set by the enable service callback).
  if (reseed_on_next_update_.exchange(false)) {
    std::lock_guard<std::mutex> lk(target_mutex_);
    target_pose_ = current_pose;
    target_initialized_ = true;
    // Capture the current posture as the null-space reference, so the redundant
    // DoF is held where it was when force engaged (no drift -> no joint jump).
    for (size_t i = 0; i < n; ++i) {
      q_nullspace_ref_(i) = q_(i);
    }
  }
  // Restart the ramp clock from the enable moment.
  if (restart_ramp_.exchange(false)) {
    activate_time_ = time;
  }

  // Cartesian error in base frame:
  //   e_trans = p_target - p_current
  //   e_rot   = axis * angle of (R_target * R_current^T)
  KDL::Frame target;
  {
    std::lock_guard<std::mutex> lk(target_mutex_);
    target = target_pose_;
  }
  const KDL::Vector e_trans = target.p - current_pose.p;
  const KDL::Rotation R_err = target.M * current_pose.M.Inverse();
  KDL::Vector axis;
  const double angle = R_err.GetRotAngle(axis);  // angle in [0, pi]
  const KDL::Vector e_rot = axis * angle;

  // One-shot snapshot of live-updatable params (lock-free via RealtimeBuffer).
  const LiveParams lp = *live_params_.readFromRT();

  // 6D error vector in base frame: [e_trans; e_rot]
  Eigen::Matrix<double, 6, 1> err;
  err << e_trans.x(), e_trans.y(), e_trans.z(),
         e_rot.x(),   e_rot.y(),   e_rot.z();
  // Clip per-axis to keep wrench from exploding when target jumps.
  for (int k = 0; k < 6; ++k) {
    err(k) = std::clamp(err(k), -lp.max_pose_error[k], lp.max_pose_error[k]);
  }

  // Cartesian twist v = J * qd (6 x 1)
  Eigen::Matrix<double, Eigen::Dynamic, 1> qd_vec(n);
  for (size_t i = 0; i < n; ++i) qd_vec(i) = qd_(i);
  Eigen::Matrix<double, 6, 1> twist = jacobian_.data * qd_vec;

  // Per-axis spring-damper with critical damping reference (mass = 1).
  Eigen::Matrix<double, 6, 1> wrench;
  for (int k = 0; k < 6; ++k) {
    const double K = lp.stiffness[k];
    const double D = 2.0 * lp.damping_ratio[k] * std::sqrt(std::max(K, 1e-9));
    wrench(k) = K * err(k) - D * twist(k);
    // Hard cap per axis -- prevents runaway from a bad target / IK pose / spike.
    wrench(k) = std::clamp(wrench(k), -lp.wrench_limit[k], lp.wrench_limit[k]);
  }

  // Joint torque from impedance.
  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_imp = jacobian_.data.transpose() * wrench;

  // Null-space torque for the redundant DoF (7-DoF arm, 6-DoF task). A
  // posture PD (toward q_nullspace_ref_) is PROJECTED into the null-space of J
  // so it never disturbs the Cartesian task; it only stops the slow internal
  // drift (= joint reconfiguration at fixed TCP) that reads as a "joint jump"
  // across mode switches.
  if (lp.nullspace_stiffness > 0.0 || lp.nullspace_damping > 0.0) {
    const Eigen::MatrixXd & J = jacobian_.data;            // 6 x n
    const Eigen::Matrix<double, 6, 6> JJt_lambda =
      J * J.transpose() + ext_wrench_lambda_ * Eigen::Matrix<double, 6, 6>::Identity();
    // null-space projector N = I - J^T (J J^T + lambda I)^-1 J   (n x n)
    const Eigen::MatrixXd N =
      Eigen::MatrixXd::Identity(n, n) - J.transpose() * JJt_lambda.inverse() * J;
    Eigen::VectorXd tau0(n);
    for (size_t i = 0; i < n; ++i) {
      tau0(i) = lp.nullspace_stiffness * (q_nullspace_ref_(i) - q_(i))
                - lp.nullspace_damping * qd_(i);
    }
    tau_imp += N * tau0;
  }

  // Activation ramp.
  double ramp = 1.0;
  if (ramp_in_time_ > 0.0) {
    ramp = std::clamp((time - activate_time_).seconds() / ramp_in_time_, 0.0, 1.0);
  }
  const bool enabled = enabled_.load();

  // Compose final command per joint.
  std::vector<double> applied(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    // Shift q_dot by the deadband: comp is forced to 0 when |q_dot| < dz,
    // killing the (coulomb/eps)*q_dot positive feedback at low velocity that
    // would otherwise destabilise the Cartesian spring loop.
    double qd_eff = 0.0;
    if (qd_(i) > lp.velocity_deadband) {
      qd_eff = qd_(i) - lp.velocity_deadband;
    } else if (qd_(i) < -lp.velocity_deadband) {
      qd_eff = qd_(i) + lp.velocity_deadband;
    }
    const double fs = lp.friction_compensation_scale;
    const double f_coulomb =
      fs * coulomb_friction_[i] * std::tanh(qd_eff / coulomb_velocity_eps_);
    const double f_viscous = fs * viscous_friction_[i] * qd_(i);
    double tau = tau_imp(i)
                 + lp.gravity_scale * g_torque_(i)
                 + f_coulomb + f_viscous
                 - damping_[i] * qd_(i);
    if (std::abs(qd_(i)) > velocity_limit_) tau = 0.0;
    tau = std::clamp(tau, -max_torque_[i], max_torque_[i]);
    tau *= ramp;
    if (!enabled) tau = 0.0;
    command_interfaces_[cmd_effort_idx_[i]].set_value(tau);
    applied[i] = tau;
  }

  // ---- External-wrench estimate (residual method) -----------------------
  // Quasi-static disturbance observer:
  //   tau_ext = tau_measured - gravity_scale * G(q)   (gravity-only residual;
  //             friction NOT subtracted in v1 -- it is velocity-dependent and
  //             sign-fragile, and the dominant use case (screw seating) is at
  //             near-zero velocity where friction ~ 0. The tare below absorbs
  //             the static bias at the tared pose. v2: subtract friction model.)
  //   F_ext   = (J^T)^+ tau_ext = (J J^T + lambda I)^-1 J tau_ext
  // then low-passed and tared.
  {
    Eigen::Matrix<double, Eigen::Dynamic, 1> tau_res(n);
    for (size_t i = 0; i < n; ++i) {
      const double tm = tau_meas_(i);
      tau_res(i) = (std::isnan(tm) ? 0.0 : tm) - lp.gravity_scale * g_torque_(i);
    }
    const Eigen::Matrix<double, 6, 6> JJt =
      jacobian_.data * jacobian_.data.transpose();
    const Eigen::Matrix<double, 6, 6> damped =
      JJt + ext_wrench_lambda_ * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 1> f_raw =
      damped.inverse() * (jacobian_.data * tau_res);

    bool do_tare = tare_on_next_update_.exchange(false);
    if (tare_after_ramp_.load() && ramp >= 1.0) {
      tare_after_ramp_.store(false);
      do_tare = true;
    }
    if (do_tare) {
      // Snap the filter to the current raw reading and take it as the new zero,
      // so the tare baseline is the converged value (not whatever the filter
      // happened to hold). Output is then exactly 0 at the tare instant.
      ext_wrench_filt_ = f_raw;
      ext_wrench_tare_ = f_raw;
    } else {
      // first-order low-pass: alpha = dt / (dt + tau_lp), tau_lp = 1/(2*pi*fc)
      const double dt = period.seconds() > 1e-6 ? period.seconds() : 0.005;
      const double tau_lp = ext_wrench_filter_cutoff_ > 1e-6
        ? 1.0 / (2.0 * M_PI * ext_wrench_filter_cutoff_) : 0.0;
      const double alpha = tau_lp > 1e-9 ? dt / (dt + tau_lp) : 1.0;
      ext_wrench_filt_ = alpha * f_raw + (1.0 - alpha) * ext_wrench_filt_;
    }
    const Eigen::Matrix<double, 6, 1> f_ext = ext_wrench_filt_ - ext_wrench_tare_;

    if (rt_ext_wrench_pub_ && rt_ext_wrench_pub_->trylock()) {
      auto & m = rt_ext_wrench_pub_->msg_;
      m.header.stamp = time;
      m.header.frame_id = ext_wrench_frame_;
      m.wrench.force.x = f_ext(0);
      m.wrench.force.y = f_ext(1);
      m.wrench.force.z = f_ext(2);
      m.wrench.torque.x = f_ext(3);
      m.wrench.torque.y = f_ext(4);
      m.wrench.torque.z = f_ext(5);
      rt_ext_wrench_pub_->unlockAndPublish();
    }
  }

  // Debug publishers (realtime-safe).
  if (rt_pose_err_pub_ && rt_pose_err_pub_->trylock()) {
    rt_pose_err_pub_->msg_.data = {err(0), err(1), err(2), err(3), err(4), err(5)};
    rt_pose_err_pub_->unlockAndPublish();
  }
  if (rt_wrench_pub_ && rt_wrench_pub_->trylock()) {
    rt_wrench_pub_->msg_.data = {wrench(0), wrench(1), wrench(2),
                                 wrench(3), wrench(4), wrench(5)};
    rt_wrench_pub_->unlockAndPublish();
  }
  if (rt_torque_pub_ && rt_torque_pub_->trylock()) {
    rt_torque_pub_->msg_.data.assign(applied.begin(), applied.end());
    rt_torque_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace armv7_impedance_moveit

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  armv7_impedance_moveit::CartesianImpedanceController,
  controller_interface::ControllerInterface)

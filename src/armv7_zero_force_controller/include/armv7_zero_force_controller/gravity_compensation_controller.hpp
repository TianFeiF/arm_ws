// Copyright 2026 TianFeiF
// SPDX-License-Identifier: Apache-2.0
#ifndef ARMV7_ZERO_FORCE_CONTROLLER__GRAVITY_COMPENSATION_CONTROLLER_HPP_
#define ARMV7_ZERO_FORCE_CONTROLLER__GRAVITY_COMPENSATION_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "kdl/chain.hpp"
#include "kdl/chaindynparam.hpp"
#include "kdl/jntarray.hpp"

namespace armv7_zero_force_controller
{

/// Effort controller that holds the arm against gravity (free-drive / zero-force).
///
/// Each cycle it computes the gravity torque G(q) from a KDL model built off the
/// robot_description (per-link mass + CoM optionally overridden by the output of
/// armv7_dyn_ident), then commands that torque on the effort interfaces. With
/// gravity cancelled the arm is weightless and can be pushed around by hand.
///
/// Requires the drives in a torque mode (CiA-402 CST, mode 10) exposing an
/// `effort` command interface — see armv7_bringup free_drive.launch.py.
class GravityCompensationController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  bool build_model();
  bool apply_identified_params(const std::string & yaml_path);
  void command_zero();

  std::vector<std::string> joints_;
  std::string robot_description_;
  std::string root_link_;
  std::string tip_link_;
  std::string identified_params_file_;
  double gravity_scale_{1.0};
  double ramp_in_time_{2.0};
  double velocity_limit_{2.0};
  std::vector<double> gravity_vec_{0.0, 0.0, -9.80665};
  std::vector<double> max_torque_;
  std::vector<double> damping_;

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_param_;
  KDL::JntArray q_;
  KDL::JntArray qd_;
  KDL::JntArray g_torque_;

  std::vector<size_t> cmd_effort_idx_;
  std::vector<size_t> state_pos_idx_;
  std::vector<size_t> state_vel_idx_;

  std::atomic<bool> enabled_{false};
  rclcpp::Time activate_time_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;

  using DebugMsg = std_msgs::msg::Float64MultiArray;
  rclcpp::Publisher<DebugMsg>::SharedPtr debug_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<DebugMsg>> rt_debug_pub_;
};

}  // namespace armv7_zero_force_controller

#endif  // ARMV7_ZERO_FORCE_CONTROLLER__GRAVITY_COMPENSATION_CONTROLLER_HPP_

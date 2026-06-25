// Copyright 2026 TianFeiF
// SPDX-License-Identifier: Apache-2.0
//
// 6-DoF Cartesian impedance controller.
//
// Each cycle:
//   1. Read q, q_dot from state interfaces.
//   2. FK -> current TCP pose; J(q) -> 6xN Jacobian.
//   3. Translation error      e_p = p_target - p_current.
//      Rotation error vector  e_R = axis * angle of (R_target * R_current^T).
//      Cartesian twist        v   = J . q_dot.
//   4. Diagonal Cartesian spring-damper in BASE frame:
//          W = diag(K) e - diag(D) v
//      where D_i = 2 * zeta_i * sqrt(K_i) (per-axis critical-damping scaling
//      with effective_mass = 1 -- conservative, no mass identification needed).
//   5. Joint torque:
//          tau = J^T W
//                + gravity_scale * G(q)                    [KDL ChainDynParam]
//                + coulomb_friction . tanh(q_dot / eps)    [reuses v0.1 calibration]
//                + viscous_friction . q_dot
//                - damping . q_dot
//   6. Clamp |tau| <= max_torque, scale by activation ramp, gate by enable flag,
//      write to effort command interfaces.
//
// Stiffness lives in the chain's root (base_link) frame. Tool-frame stiffness
// is a v0.2.x extension; for now, mount the work piece so its compliant
// directions align with the base axes (typical for vertical pick / press / screw).
//
// Same safety net as armv7_zero_force_controller: velocity_limit cutoff per
// joint, ramp-in on activation, atomic enable service, hard wrench clamp on
// each axis, hard joint-torque clamp.
//
// Hardware requirement: the drives must expose an `effort` command interface
// (CiA-402 CST, mode 10). See armv7_bringup/urdf/armv7_cst.ros2_control.xacro.
#ifndef ARMV7_IMPEDANCE_MOVEIT__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_
#define ARMV7_IMPEDANCE_MOVEIT__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "kdl/chain.hpp"
#include "kdl/chaindynparam.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/frames.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/jntarray.hpp"

namespace armv7_impedance_moveit
{

class CartesianImpedanceController : public controller_interface::ControllerInterface
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
  void on_target_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void command_zero();

  // Parameters
  std::vector<std::string> joints_;
  std::string robot_description_;
  std::string root_link_;
  std::string tip_link_;
  std::string identified_params_file_;
  /// Bundle of parameters that may be updated live from the ROS param callback.
  /// Read once at the top of update() via realtime_tools::RealtimeBuffer so the
  /// non-RT callback thread cannot stall the RT loop. Joint-space safety knobs
  /// (max_torque, velocity_limit, friction, damping) intentionally NOT included
  /// — those require deactivate / reactivate.
  struct LiveParams
  {
    std::array<double, 6> stiffness{};
    std::array<double, 6> damping_ratio{};
    std::array<double, 6> wrench_limit{};
    std::array<double, 6> max_pose_error{};
    double gravity_scale{1.0};
    /// Velocity deadband (rad/s) applied to friction compensation. Coulomb +
    /// viscous comp terms see q̇_shifted = sign(q̇) * max(0, |q̇| − dz). When
    /// |q̇| < dz, comp = 0, eliminating the (coulomb/eps)·q̇ positive feedback
    /// that the unshifted tanh introduces near zero velocity. Without this,
    /// impedance loops with high Coulomb comp diverge into limit-cycle shake.
    double velocity_deadband{0.05};
    /// 0..1 scale on coulomb_friction and viscous_friction. Lets the operator
    /// dial all friction comp down (or off) live without re-editing yaml.
    double friction_compensation_scale{1.0};
    /// Null-space control for the 7-DoF redundancy (1 free internal DoF for a
    /// 6-DoF task). Acts ONLY in the null-space of J, so it never disturbs the
    /// Cartesian task. nullspace_damping (Nm*s/rad) suppresses null-space drift
    /// -- the slow joint reconfiguration at fixed TCP that otherwise shows up as
    /// a "joint jump" across force<->position switches. nullspace_stiffness
    /// (Nm/rad) additionally pulls the redundant DoF toward the posture captured
    /// when force was engaged (0 = free to reconfigure, only damped).
    double nullspace_stiffness{0.0};
    double nullspace_damping{1.0};
  };
  realtime_tools::RealtimeBuffer<LiveParams> live_params_;
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  double ramp_in_time_{2.0};
  double velocity_limit_{2.0};
  double coulomb_velocity_eps_{0.05};
  std::vector<double> gravity_vec_{0.0, 0.0, -9.80665};
  std::vector<double> max_torque_;
  std::vector<double> damping_;
  std::vector<double> coulomb_friction_;
  std::vector<double> viscous_friction_;

  // External-wrench estimation (residual method)
  double ext_wrench_lambda_{0.01};       // damped pseudo-inverse regularisation
  double ext_wrench_filter_cutoff_{5.0};  // low-pass cutoff (Hz)
  std::string ext_wrench_frame_{"base_link"};
  Eigen::Matrix<double, 6, 1> ext_wrench_filt_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> ext_wrench_tare_{Eigen::Matrix<double, 6, 1>::Zero()};
  std::atomic<bool> tare_on_next_update_{false};   // manual ~/tare service: immediate
  std::atomic<bool> tare_after_ramp_{false};       // on enable: tare once ramp completes
  KDL::JntArray tau_meas_;

  // KDL
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_param_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  KDL::JntArray q_;
  KDL::JntArray qd_;
  KDL::JntArray g_torque_;
  KDL::Jacobian jacobian_;
  /// Reference posture for null-space control, captured when force is engaged
  /// (target reseed). The null-space term pulls the redundant DoF toward this.
  Eigen::VectorXd q_nullspace_ref_;

  // Target pose (set on activate to current pose; updated via topic)
  std::mutex target_mutex_;
  KDL::Frame target_pose_;
  bool target_initialized_{false};
  /// Set true by the enable service on disabled -> enabled; the next update()
  /// cycle re-snapshots target_pose_ to the current FK so the arm holds in
  /// place at the moment of enable rather than springing back to whatever the
  /// pose was at on_activate (which can be minutes stale, with the arm having
  /// sagged or been moved by hand in the meantime).
  std::atomic<bool> reseed_on_next_update_{false};
  /// Set by the enable service on disabled -> enabled; the next update() resets
  /// activate_time_ to now so the torque ramp-in restarts FROM THE ENABLE moment
  /// (not from on_activate, which may be minutes earlier -> ramp already 1.0 ->
  /// torque would step instantly to full on enable).
  std::atomic<bool> restart_ramp_{false};

  // Interface index maps (resolved on activate)
  std::vector<size_t> cmd_effort_idx_;
  std::vector<size_t> state_pos_idx_;
  std::vector<size_t> state_vel_idx_;
  std::vector<size_t> state_eff_idx_;

  // Lifecycle / safety
  std::atomic<bool> enabled_{false};
  rclcpp::Time activate_time_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_srv_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;

  // Debug pubs (realtime-safe)
  using Float64Array = std_msgs::msg::Float64MultiArray;
  using WrenchStamped = geometry_msgs::msg::WrenchStamped;
  std::shared_ptr<realtime_tools::RealtimePublisher<Float64Array>> rt_pose_err_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<Float64Array>> rt_wrench_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<Float64Array>> rt_torque_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<WrenchStamped>> rt_ext_wrench_pub_;
  rclcpp::Publisher<Float64Array>::SharedPtr pose_err_pub_;
  rclcpp::Publisher<Float64Array>::SharedPtr wrench_pub_;
  rclcpp::Publisher<Float64Array>::SharedPtr torque_pub_;
  rclcpp::Publisher<WrenchStamped>::SharedPtr ext_wrench_pub_;
};

}  // namespace armv7_impedance_moveit

#endif  // ARMV7_IMPEDANCE_MOVEIT__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_

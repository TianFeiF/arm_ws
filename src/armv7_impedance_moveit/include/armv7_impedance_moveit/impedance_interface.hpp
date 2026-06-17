// Copyright 2026 TianFeiF
// SPDX-License-Identifier: Apache-2.0
//
// Interface contract for a 6-DoF Cartesian impedance / admittance controller.
//
// THIS HEADER DEFINES TYPES ONLY. The v0.1 release does NOT ship a controller
// implementation — see the package README for what v0.2 will deliver and which
// pieces are still TODO. Downstream code can build against these structs today
// so that switching to the real controller later is a drop-in change.
//
// The interface is intentionally symmetric for impedance and admittance:
//   - impedance: input = target pose, output = wrench (drives effort).
//   - admittance: input = external wrench, output = velocity (drives joint vel).
// Both share the same stiffness / damping parameterisation in task space.

#ifndef ARMV7_IMPEDANCE_MOVEIT__IMPEDANCE_INTERFACE_HPP_
#define ARMV7_IMPEDANCE_MOVEIT__IMPEDANCE_INTERFACE_HPP_

#include <array>
#include <string>

namespace armv7_impedance_moveit
{

/// Six task-space DOFs, ordered consistently across this header:
/// [translation_x, translation_y, translation_z, rotation_x, rotation_y, rotation_z].
inline constexpr std::size_t kTaskDof = 6;

/// Diagonal stiffness / damping per task-space axis. Off-diagonal coupling
/// (anisotropy along non-axis-aligned directions) is out of scope for v0.2.
struct CartesianImpedanceParams
{
  /// N/m for translation axes (0..2), N*m/rad for rotation axes (3..5).
  std::array<double, kTaskDof> stiffness{};
  /// Dimensionless damping ratio per axis. 1.0 = critical damping at the
  /// corresponding stiffness; the controller derives absolute damping from
  /// 2 * damping_ratio * sqrt(stiffness * effective_mass).
  std::array<double, kTaskDof> damping_ratio{};
  /// Hard clamp on commanded wrench (N for trans, N*m for rot). Safety
  /// ceiling against runaway when stiffness is large and tracking error big.
  std::array<double, kTaskDof> wrench_limit{};
};

/// Where the impedance acts. The v0.2 controller will look up `tip_frame` in
/// the chain and apply (stiffness, damping) about `reference_frame`.
struct CartesianImpedanceConfig
{
  std::string reference_frame{"base_link"};
  std::string tip_frame{"tcp"};
  CartesianImpedanceParams params{};
};

/// Planned ROS interface. Each name below is what v0.2 will publish / accept;
/// keeping them in code (not just docs) makes future renames a compiler error.
namespace topics
{
/// geometry_msgs/PoseStamped — target Cartesian pose for impedance mode.
inline constexpr const char * kTargetPose = "~/target_pose";
/// geometry_msgs/WrenchStamped — target wrench feed-forward for admittance mode.
inline constexpr const char * kTargetWrench = "~/target_wrench";
/// geometry_msgs/WrenchStamped — measured / estimated external wrench at TCP.
inline constexpr const char * kExternalWrench = "~/external_wrench";
/// Cartesian tracking error (geometry_msgs/Vector3Stamped twice — translation + rotation).
inline constexpr const char * kPoseError = "~/pose_error";
}  // namespace topics

namespace services
{
/// std_srvs/SetBool — enable / disable the controller, same semantics as
/// armv7_zero_force_controller for consistency.
inline constexpr const char * kEnable = "~/enable";
/// armv7_impedance_moveit/srv/SetImpedance (TODO: define msg) — push a new
/// CartesianImpedanceParams while the controller is running.
inline constexpr const char * kSetImpedance = "~/set_impedance";
}  // namespace services

}  // namespace armv7_impedance_moveit

#endif  // ARMV7_IMPEDANCE_MOVEIT__IMPEDANCE_INTERFACE_HPP_

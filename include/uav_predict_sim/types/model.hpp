#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace uav {
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;

struct VehicleParams {
  double mass = 0.98, gravity = 9.81;
  Mat3 inertia = Vec3(2.64e-3, 2.64e-3, 4.96e-3).asDiagonal();
  double arm = 0.26, kf = 8.98132e-9, km = 0.07 * (3 * 0.062) * 8.98132e-9;
  double motor_tau = 1.0 / 30, min_rpm = 1200, max_rpm = 35000;
  double drag = 0;  // F_drag = -drag * ||v|| * v, world frame.
  void validate() const;
};
struct State {
  double time = 0;
  Vec3 position = Vec3::Zero(), velocity = Vec3::Zero();
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity(); // body -> world
  Vec3 omega = Vec3::Zero(); // body rad/s
  Vec4 rpm = Vec4::Zero();
  Mat3 rotation() const { return attitude.normalized().toRotationMatrix(); }
};
struct Disturbance {
  Vec3 force = Vec3::Zero(), torque = Vec3::Zero(); // world N, body N m
};
struct Wrench {
  double thrust = 0;
  Vec3 torque = Vec3::Zero();
  Vec4 vector() const { Vec4 v; v << thrust, torque; return v; }
};
struct FlatSample {
  Vec3 position = Vec3::Zero(), velocity = Vec3::Zero(), acceleration = Vec3::Zero();
  Vec3 jerk = Vec3::Zero(), snap = Vec3::Zero();
  double heading = 0, heading_rate = 0, heading_acceleration = 0;
};
struct Limits {
  double max_tilt = 0.7853981633974483;
  double max_speed = 10, max_acceleration = 12, max_jerk = 30, max_snap = 100;
  double max_omega = 8, max_alpha = 40;
  void validate() const;
};
struct ConstraintReport {
  std::vector<std::string> violations;
  bool feasible() const { return violations.empty(); }
};
Mat3 hat(const Vec3& v);
Vec3 vee(const Mat3& m);
} // namespace uav

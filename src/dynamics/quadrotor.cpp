// Adapted from ZJU-FAST-Lab/ego-planner uav_simulator dynamics and getControl.
// SPDX-License-Identifier: GPL-3.0-only
// See third_party/README.md for original paths, revision and changes.
#include "uav_predict_sim/dynamics/quadrotor.hpp"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uav {
void VehicleParams::validate() const {
  for (double v : {mass, gravity, arm, kf, km, motor_tau, max_rpm})
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("vehicle parameters must be finite and positive");
  if (!std::isfinite(min_rpm) || min_rpm < 0 || min_rpm >= max_rpm ||
      !std::isfinite(drag) || drag < 0 || !inertia.allFinite() ||
      !inertia.isApprox(inertia.transpose(), 1e-12) || Eigen::LLT<Mat3>(inertia).info() != Eigen::Success)
    throw std::invalid_argument("invalid RPM, drag or inertia");
}
void Limits::validate() const {
  for (double v : {max_tilt, max_speed, max_acceleration, max_jerk, max_snap, max_omega, max_alpha})
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("limits must be positive");
  if (max_tilt >= 1.5707963267948966) throw std::invalid_argument("max tilt must be below pi/2");
}
Mat3 hat(const Vec3& v) {
  Mat3 m; m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0; return m;
}
Vec3 vee(const Mat3& m) {
  return 0.5 * Vec3(m(2,1)-m(1,2), m(0,2)-m(2,0), m(1,0)-m(0,1));
}
Eigen::Matrix4d allocation_matrix(const VehicleParams& p) {
  Eigen::Matrix4d a;
  a << p.kf, p.kf, p.kf, p.kf,
       0, 0, p.arm*p.kf, -p.arm*p.kf,
       -p.arm*p.kf, p.arm*p.kf, 0, 0,
       p.km, p.km, -p.km, -p.km;
  return a;
}
Wrench rotor_wrench(const Vec4& rpm, const VehicleParams& p) {
  const Vec4 w = allocation_matrix(p) * rpm.array().square().matrix();
  return {w[0], w.tail<3>()};
}
Allocation allocate(const Wrench& w, const VehicleParams& p) {
  if (!w.vector().allFinite()) throw std::invalid_argument("nonfinite wrench");
  Allocation a;
  // Keep upstream analytic inverse rather than solving an ill-scaled matrix.
  a.raw_squared_rpm << w.thrust/(4*p.kf)-w.torque.y()/(2*p.arm*p.kf)+w.torque.z()/(4*p.km),
      w.thrust/(4*p.kf)+w.torque.y()/(2*p.arm*p.kf)+w.torque.z()/(4*p.km),
      w.thrust/(4*p.kf)+w.torque.x()/(2*p.arm*p.kf)-w.torque.z()/(4*p.km),
      w.thrust/(4*p.kf)-w.torque.x()/(2*p.arm*p.kf)-w.torque.z()/(4*p.km);
  for (int i = 0; i < 4; ++i) {
    const double n2 = a.raw_squared_rpm[i];
    const double low = p.min_rpm*p.min_rpm, high = p.max_rpm*p.max_rpm;
    if (n2 < low - 1e-6 || n2 > high + 1e-6) a.saturated = true;
    a.commanded_rpm[i] = std::sqrt(std::clamp(n2, low, high));
  }
  a.achievable = rotor_wrench(a.commanded_rpm, p);
  return a;
}

namespace {
using Packed = Eigen::Matrix<double, 17, 1>;
void validate_state(const State& s) {
  if (!std::isfinite(s.time) || !s.position.allFinite() || !s.velocity.allFinite() ||
      !s.attitude.coeffs().allFinite() || s.attitude.norm() < 1e-12 ||
      !s.omega.allFinite() || !s.rpm.allFinite() || (s.rpm.array() < 0).any())
    throw std::runtime_error("invalid quadrotor state");
}
Packed pack(const State& s) {
  Packed x; x << s.position, s.velocity, s.attitude.coeffs(), s.omega, s.rpm; return x;
}
State unpack(const Packed& x) {
  State s; s.position=x.segment<3>(0); s.velocity=x.segment<3>(3);
  s.attitude.coeffs()=x.segment<4>(6); s.omega=x.segment<3>(10); s.rpm=x.segment<4>(13);
  return s;
}
}
Quadrotor::Quadrotor(VehicleParams p) : params_(std::move(p)) { params_.validate(); }
Vec3 Quadrotor::acceleration(const State& s, const Disturbance& d) const {
  const auto& p = params_;
  return -p.gravity*Vec3::UnitZ() + (rotor_wrench(s.rpm,p).thrust*s.rotation().col(2) +
         d.force - p.drag*s.velocity.norm()*s.velocity)/p.mass;
}
Vec3 Quadrotor::jerk(const State& s, const Vec4& command, const Disturbance& d) const {
  const auto& p = params_;
  const Vec4 ndot=(command-s.rpm)/p.motor_tau;
  const double fdot=2*p.kf*s.rpm.dot(ndot);
  const double f=rotor_wrench(s.rpm,p).thrust;
  const Vec3 a=acceleration(s,d);
  Vec3 drag_dot=Vec3::Zero();
  const double speed=s.velocity.norm();
  if (speed>1e-12) drag_dot=-p.drag*(speed*a+s.velocity*(s.velocity.dot(a)/speed));
  return (fdot*s.rotation().col(2)+f*s.rotation()*s.omega.cross(Vec3::UnitZ())+drag_dot)/p.mass;
}
State Quadrotor::step(const State& state, const Vec4& command, double dt,
                     const Disturbance& disturbance, double* norm_error) const {
  validate_state(state);
  if (!std::isfinite(dt) || dt<=0 || dt>params_.motor_tau/2 || !command.allFinite() ||
      (command.array()<params_.min_rpm).any() || (command.array()>params_.max_rpm).any() ||
      !disturbance.force.allFinite() || !disturbance.torque.allFinite())
    throw std::invalid_argument("invalid integration step, motor command or disturbance");
  const auto derivative = [&](const Packed& x) -> Packed {
    const State s=unpack(x);
    validate_state(s);
    const Wrench w=rotor_wrench(s.rpm,params_);
    const Eigen::Quaterniond spin(0,s.omega.x(),s.omega.y(),s.omega.z());
    const Eigen::Quaterniond qdot=s.attitude*spin;
    Packed dx;
    dx << s.velocity, acceleration(s,disturbance), 0.5*qdot.coeffs(),
          params_.inertia.ldlt().solve(w.torque-s.omega.cross(params_.inertia*s.omega)+disturbance.torque),
          (command-s.rpm)/params_.motor_tau;
    return dx;
  };
  const Packed x=pack(state), k1=derivative(x), k2=derivative(x+dt*k1/2),
               k3=derivative(x+dt*k2/2), k4=derivative(x+dt*k3);
  State result=unpack(x+dt*(k1+2*k2+2*k3+k4)/6);
  result.time=state.time+dt;
  validate_state(result);
  if (norm_error) *norm_error=std::abs(result.attitude.norm()-1);
  result.attitude.normalize();
  return result;
}
} // namespace uav

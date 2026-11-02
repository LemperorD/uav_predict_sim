// SPDX-License-Identifier: GPL-3.0-only
#include "uav_predict_sim/flatness/map.hpp"
#include <cmath>
#include <stdexcept>

namespace uav {
namespace {
struct Jet {
  Vec3 value, rate, acceleration;
};
Jet normalize(const Jet& a) {
  const double n=a.value.norm();
  if (!std::isfinite(n) || n<1e-8) throw std::domain_error("singular force or heading axis");
  const double dn=a.value.dot(a.rate)/n;
  const double ddn=(a.rate.squaredNorm()+a.value.dot(a.acceleration)-dn*dn)/n;
  Jet b; b.value=a.value/n;
  b.rate=(a.rate-b.value*dn)/n;
  b.acceleration=(a.acceleration-2*b.rate*dn-b.value*ddn)/n;
  return b;
}
Jet cross(const Jet& a, const Jet& b) {
  return {a.value.cross(b.value), a.rate.cross(b.value)+a.value.cross(b.rate),
    a.acceleration.cross(b.value)+2*a.rate.cross(b.rate)+a.value.cross(b.acceleration)};
}
}
AttitudeReference force_attitude(const Vec3& force,const Vec3& df,const Vec3& ddf,
    double psi,double dpsi,double ddpsi,const VehicleParams& p) {
  if (!force.allFinite() || !df.allFinite() || !ddf.allFinite() ||
      !std::isfinite(psi) || !std::isfinite(dpsi) || !std::isfinite(ddpsi))
    throw std::invalid_argument("nonfinite attitude reference");
  const Vec3 c(std::cos(psi),std::sin(psi),0), tangent(-std::sin(psi),std::cos(psi),0);
  const Jet heading{c,dpsi*tangent,ddpsi*tangent-dpsi*dpsi*c};
  const Jet b3=normalize({force,df,ddf});
  const Jet b2=normalize(cross(b3,heading));
  const Jet b1=cross(b2,b3);
  AttitudeReference result;
  result.rotation << b1.value,b2.value,b3.value;
  result.rotation_rate << b1.rate,b2.rate,b3.rate;
  result.rotation_acceleration << b1.acceleration,b2.acceleration,b3.acceleration;
  result.omega=vee(result.rotation.transpose()*result.rotation_rate);
  result.alpha=vee(result.rotation.transpose()*result.rotation_acceleration-hat(result.omega)*hat(result.omega));
  result.wrench={force.norm(),p.inertia*result.alpha+result.omega.cross(p.inertia*result.omega)};
  return result;
}
AttitudeReference flatness_map(const FlatSample& s,const VehicleParams& p) {
  return force_attitude(p.mass*(s.acceleration+p.gravity*Vec3::UnitZ()),p.mass*s.jerk,
                       p.mass*s.snap,s.heading,s.heading_rate,s.heading_acceleration,p);
}
ConstraintReport check_flatness(const FlatSample& s,const VehicleParams& p,const Limits& l) {
  ConstraintReport report;
  if (!s.position.allFinite() || !s.velocity.allFinite()) report.violations.push_back("nonfinite_state");
  try {
    const auto f=flatness_map(s,p);
    if (f.rotation(2,2)<std::cos(l.max_tilt)) report.violations.push_back("tilt");
    if (f.omega.norm()>l.max_omega) report.violations.push_back("angular_velocity");
    if (f.alpha.norm()>l.max_alpha) report.violations.push_back("angular_acceleration");
    if (allocate(f.wrench,p).saturated) report.violations.push_back("rotor_allocation");
  } catch(const std::exception&) { report.violations.push_back("singular_or_nonfinite_flatness"); }
  if (s.velocity.norm()>l.max_speed) report.violations.push_back("speed");
  if (s.acceleration.norm()>l.max_acceleration) report.violations.push_back("acceleration");
  if (s.jerk.norm()>l.max_jerk) report.violations.push_back("jerk");
  if (s.snap.norm()>l.max_snap) report.violations.push_back("snap");
  return report;
}
} // namespace uav

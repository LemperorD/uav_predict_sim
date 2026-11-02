#pragma once
#include "uav_predict_sim/dynamics/quadrotor.hpp"
#include "uav_predict_sim/flatness/map.hpp"

namespace uav {
struct Gains {
  Vec3 position=Vec3(5.7,5.7,6.2), velocity=Vec3(3.4,3.4,4);
  Vec3 rotation=Vec3(0.3,0.3,0.15), omega=Vec3(0.06,0.06,0.04);
  void validate() const;
};
struct ControlOutput {
  Vec3 force_world=Vec3::Zero();
  AttitudeReference desired;
  Wrench requested;
  double attitude_error=0;
};
ControlOutput control(const State& state,const FlatSample& target,const Vec4& held_command,
    const Quadrotor& quad,const Gains& gains,const Disturbance& disturbance={},bool upstream_compatible=false);
} // namespace uav

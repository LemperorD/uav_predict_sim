// SPDX-License-Identifier: GPL-3.0-only
#include "uav_predict_sim/trajectory/reference.hpp"
#include <cmath>
#include <stdexcept>

namespace uav {
void TrajectoryParams::validate() const {
  if (scene!="hover" && scene!="vertical" && scene!="helix" && scene!="eight")
    throw std::invalid_argument("scene must be hover, vertical, helix or eight");
  if (!std::isfinite(time_scale) || time_scale<=0 || !std::isfinite(heading_amplitude))
    throw std::invalid_argument("invalid trajectory time scale or heading amplitude");
}
FlatSample reference(const TrajectoryParams& params,double time) {
  FlatSample sample;
  sample.position=Vec3(0,0,3);
  const double t=time/params.time_scale;
  const auto sine=[&](int axis,double amplitude,double frequency,double phase) {
    Vec3* derivatives[]={&sample.position,&sample.velocity,&sample.acceleration,&sample.jerk,&sample.snap};
    for(int k=0;k<5;++k)
      (*derivatives[k])[axis]+=amplitude*std::pow(frequency/params.time_scale,k)*
          std::sin(frequency*t+phase+k*1.5707963267948966);
  };
  if(params.scene=="vertical") sine(2,1,0.5,0);
  else if(params.scene=="helix") {
    sine(0,3,0.4,1.5707963267948966); sine(1,3,0.4,0);
    sample.position.z()+=0.15*t; sample.velocity.z()=0.15/params.time_scale;
  } else if(params.scene=="eight") {
    sine(0,3,0.4,0); sine(1,2,0.8,0); sine(2,0.8,0.6,0);
  } else if(params.scene!="hover") throw std::invalid_argument("unknown scene");
  if(params.scene!="hover") {
    const double frequency=0.25/params.time_scale, angle=frequency*time;
    sample.heading=params.heading_amplitude*std::sin(angle);
    sample.heading_rate=params.heading_amplitude*frequency*std::cos(angle);
    sample.heading_acceleration=-params.heading_amplitude*frequency*frequency*std::sin(angle);
  }
  return sample;
}
} // namespace uav

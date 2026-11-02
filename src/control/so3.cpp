// Adapted from EGO-Planner SO3Control.cpp and quadrotor_simulator_so3.cpp.
// SPDX-License-Identifier: GPL-3.0-only
// See third_party/README.md. Adds consistent desired angular-rate feedforward.
#include "uav_predict_sim/control/so3.hpp"
#include <stdexcept>

namespace uav {
void Gains::validate() const {
  for(const auto* v : {&position,&velocity,&rotation,&omega})
    if(!v->allFinite() || (v->array()<=0).any()) throw std::invalid_argument("gains must be positive");
}
ControlOutput control(const State& state,const FlatSample& target,const Vec4& held,
    const Quadrotor& quad,const Gains& g,const Disturbance& disturbance,bool upstream) {
  const auto& p=quad.params();
  const Vec3 ep=target.position-state.position, ev=target.velocity-state.velocity;
  const Vec3 a=quad.acceleration(state,disturbance), j=quad.jerk(state,held,disturbance);
  ControlOutput out;
  out.force_world=p.mass*(target.acceleration+p.gravity*Vec3::UnitZ())+
                  g.position.cwiseProduct(ep)+g.velocity.cwiseProduct(ev);
  Vec3 df=p.mass*target.jerk+g.position.cwiseProduct(ev)+g.velocity.cwiseProduct(target.acceleration-a);
  Vec3 ddf=p.mass*target.snap+g.position.cwiseProduct(target.acceleration-a)+g.velocity.cwiseProduct(target.jerk-j);
  if(upstream) {
    const Vec3 total=ep+ev+target.acceleration-a;
    Vec3 ka;
    for(int i=0;i<3;++i) ka[i]=std::abs(total[i])>3 ? 0 : std::abs(total[i])*0.2;
    out.force_world+=p.mass*ka.cwiseProduct(target.acceleration-a);
    df.setZero(); ddf.setZero();
  }
  out.desired=force_attitude(out.force_world,df,ddf,target.heading,
      upstream?0:target.heading_rate,upstream?0:target.heading_acceleration,p);
  const Mat3 r=state.rotation(), rc=out.desired.rotation, relative=r.transpose()*rc;
  const Vec3 er=0.5*vee(rc.transpose()*r-r.transpose()*rc);
  const Vec3 ew=state.omega-relative*out.desired.omega;
  out.attitude_error=0.5*(3-(rc.transpose()*r).trace());
  out.requested.thrust=out.force_world.dot(r.col(2));
  out.requested.torque=-g.rotation.cwiseProduct(er)-g.omega.cwiseProduct(ew)+
      state.omega.cross(p.inertia*state.omega);
  if(!upstream)
    out.requested.torque-=p.inertia*(hat(state.omega)*relative*out.desired.omega-relative*out.desired.alpha);
  else if(out.attitude_error>=1) out.requested.thrust=0;
  return out;
}
} // namespace uav

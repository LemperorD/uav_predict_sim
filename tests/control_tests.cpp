#include "uav_predict_sim/control/so3.hpp"
#include "uav_predict_sim/trajectory/reference.hpp"
#include <iostream>
#include <stdexcept>

void check(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
int main() {
  try {
    using namespace uav;
    VehicleParams p; Quadrotor quad(p); Limits limits; Gains gains;
    // Independent central differences validate the analytic first/second map.
    TrajectoryParams trajectory; trajectory.scene="eight";
    for(double t : {0.0,1.0,3.0,7.0}) {
      const double h=1e-4;
      const auto s=reference(trajectory,t);
      const auto mid=flatness_map(s,p);
      const auto lo=flatness_map(reference(trajectory,t-h),p), hi=flatness_map(reference(trajectory,t+h),p);
      check((mid.rotation_rate-(hi.rotation-lo.rotation)/(2*h)).norm()<1e-8,"R dot derivative");
      check((mid.rotation_acceleration-(hi.rotation-2*mid.rotation+lo.rotation)/(h*h)).norm()<2e-6,"R ddot derivative");
      check((mid.alpha-(hi.omega-lo.omega)/(2*h)).norm()<1e-8,"alpha derivative");
      check((p.mass*(s.acceleration+p.gravity*Vec3::UnitZ())-mid.wrench.thrust*mid.rotation.col(2)).norm()<1e-12,"flatness force residual");
      check(check_flatness(s,p,limits).feasible(),"reference envelope");
    }
    FlatSample singular; singular.acceleration=-p.gravity*Vec3::UnitZ();
    check(!check_flatness(singular,p,limits).feasible(),"free-fall singularity");
    for(const std::string scene : {"hover","vertical","helix","eight"}) {
      trajectory.scene=scene;
      const auto start=reference(trajectory,0);
      const auto flat=flatness_map(start,p);
      State state; state.position=start.position; state.velocity=start.velocity;
      state.attitude=Eigen::Quaterniond(flat.rotation); state.omega=flat.omega;
      state.rpm=allocate(flat.wrench,p).commanded_rpm;
      Vec4 held=state.rpm;
      double squared=0,max_error=0; int saturations=0;
      for(int tick=0;tick<10000;++tick) {
        state.time=tick*0.001;
        const auto target=reference(trajectory,state.time);
        const auto output=control(state,target,held,quad,gains);
        const auto allocation=allocate(output.requested,p);
        held=allocation.commanded_rpm; saturations+=allocation.saturated;
        const double error=(state.position-target.position).norm(); squared+=error*error;
        max_error=std::max(max_error,error);
        state=quad.step(state,held,0.001);
      }
      const double rmse=std::sqrt(squared/10000);
      std::cout<<scene<<": tracking RMSE="<<rmse<<", max="<<max_error<<", saturations="<<saturations<<'\n';
      check(rmse<0.12 && max_error<0.3,"3D tracking tolerance");
      check(saturations==0,"nominal motor saturation");
    }
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}

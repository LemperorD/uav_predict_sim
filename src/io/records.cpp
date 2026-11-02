// SPDX-License-Identifier: GPL-3.0-only
#include "uav_predict_sim/simulation/experiment.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <stdexcept>

#ifndef UAV_REVISION
#define UAV_REVISION "unknown"
#endif
namespace uav {
namespace {
std::ofstream stream(const std::filesystem::path& path) {
  std::ofstream f;
  f.exceptions(std::ios::failbit|std::ios::badbit); f.open(path); f<<std::setprecision(17); return f;
}
void v3(std::ostream& s,const Vec3& v) { s<<v.x()<<','<<v.y()<<','<<v.z(); }
void v4(std::ostream& s,const Vec4& v) { for(int i=0;i<4;++i) { if(i) s<<','; s<<v[i]; } }
void quaternion(std::ostream& s,const Eigen::Quaterniond& q) { s<<q.w()<<','<<q.x()<<','<<q.y()<<','<<q.z(); }
}
void save(const Config& c,const Experiment& e,const std::filesystem::path& directory) {
  if(e.frames.empty()) throw std::invalid_argument("empty experiment");
  if(!std::filesystem::create_directories(directory)) throw std::runtime_error("output already exists");
  auto config=stream(directory/"config.yaml"); config<<config_yaml(c); config.close();
  auto metadata=stream(directory/"metadata.txt");
  metadata<<"model=quadrotor_6dof_motor_lag\nworld=z_up\nrotation=body_to_world\nquaternion=qw,qx,qy,qz\n"
      <<"rotor_layout=plus_front_back_left_right\nrotor_unit=RPM\n"
      <<"flatness=ideal_rigid_body_no_drag\nreference_validation=sampled_at_every_physics_tick\n"
      <<"observation_mode=position_only\npredictor_physical_guarantee=none_CV_CA_baselines\n"
      <<"state_feedback=ideal_current_full_state\nrevision_at_build="<<UAV_REVISION
      <<"\nupstream_revision=bfda51284c8c1b476043255a8145ef925a3778a5\n"
      <<"compiler="<<__VERSION__<<"\nreproducibility=same_binary_and_standard_library_except_timing\n";
  metadata.close();
  auto state=stream(directory/"state.csv"), ref=stream(directory/"reference.csv"), ctrl=stream(directory/"control.csv");
  state<<"time,x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,rpm1,rpm2,rpm3,rpm4,ax,ay,az,tilt,tracking_error,qnorm_error,state_feasible\n";
  ref<<"time,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,sx,sy,sz,heading,heading_rate,heading_acceleration\n";
  ctrl<<"time,fx,fy,fz,thrust_cmd,tx_cmd,ty_cmd,tz_cmd,rpm1_cmd,rpm2_cmd,rpm3_cmd,rpm4_cmd,raw_n1_sq,raw_n2_sq,raw_n3_sq,raw_n4_sq,thrust_actual,tx_actual,ty_actual,tz_actual,saturated,attitude_error,rc_qw,rc_qx,rc_qy,rc_qz,wcx,wcy,wcz,alphacx,alphacy,alphacz\n";
  for(const auto& f:e.frames) {
    const auto& s=f.state;
    state<<s.time<<','; v3(state,s.position); state<<','; v3(state,s.velocity); state<<',';
    quaternion(state,s.attitude); state<<','; v3(state,s.omega); state<<','; v4(state,s.rpm);
    state<<','; v3(state,f.acceleration);
    state<<','<<f.tilt<<','<<f.position_error<<','<<f.quaternion_norm_error<<','<<f.state_feasible<<'\n';
    const auto& r=f.reference;
    ref<<s.time<<','; v3(ref,r.position); ref<<','; v3(ref,r.velocity); ref<<','; v3(ref,r.acceleration);
    ref<<','; v3(ref,r.jerk); ref<<','; v3(ref,r.snap);
    ref<<','<<r.heading<<','<<r.heading_rate<<','<<r.heading_acceleration<<'\n';
    const auto actual=rotor_wrench(s.rpm,c.vehicle);
    ctrl<<s.time<<','; v3(ctrl,f.control.force_world); ctrl<<','<<f.control.requested.thrust<<',';
    v3(ctrl,f.control.requested.torque); ctrl<<','; v4(ctrl,f.allocation.commanded_rpm);
    ctrl<<','; v4(ctrl,f.allocation.raw_squared_rpm);
    ctrl<<','<<actual.thrust<<','; v3(ctrl,actual.torque);
    ctrl<<','<<f.allocation.saturated<<','<<f.control.attitude_error<<',';
    quaternion(ctrl,Eigen::Quaterniond(f.control.desired.rotation));
    ctrl<<','; v3(ctrl,f.control.desired.omega); ctrl<<','; v3(ctrl,f.control.desired.alpha); ctrl<<'\n';
  }
  state.close(); ref.close(); ctrl.close();
  auto obs=stream(directory/"observation.csv"); obs<<"time,x,y,z\n";
  for(const auto& o:e.observations) { obs<<o.time<<','; v3(obs,o.position); obs<<'\n'; } obs.close();
  auto forecast=stream(directory/"prediction.csv"), metrics=stream(directory/"metrics.csv");
  forecast<<"forecast_id,model,origin,time,lead,x,y,z,true_x,true_y,true_z,error_m\n";
  metrics<<"forecast_id,model,origin,status,ade_m,fde_m,predict_us,physical_constraints\n";
  for(std::size_t id=0;id<e.forecasts.size();++id) {
    const auto& f=e.forecasts[id];
    metrics<<id<<','<<name(f.model)<<','<<f.origin<<','<<f.status<<',';
    if(f.status=="complete") metrics<<f.ade<<','<<f.fde; else metrics<<',';
    metrics<<','<<f.elapsed_us<<",not_applicable\n";
    for(std::size_t i=0;i<f.predicted.size();++i) {
      const auto& point=f.predicted[i];
      forecast<<id<<','<<name(f.model)<<','<<f.origin<<','<<point.time<<','<<point.time-f.origin<<',';
      v3(forecast,point.position); forecast<<',';
      if(i<f.truth.size()) {
        v3(forecast,f.truth[i].position); forecast<<','<<(point.position-f.truth[i].position).norm();
      } else forecast<<",,,";
      forecast<<'\n';
    }
  }
  forecast.close(); metrics.close();
  auto leads=stream(directory/"lead_metrics.csv");
  leads<<"model,lead,position_rmse_m,x_rmse_m,y_rmse_m,z_rmse_m,samples\n";
  for(auto model:{PredictionModel::CV,PredictionModel::CA}) {
    const std::size_t steps=e.forecasts.front().predicted.size();
    for(std::size_t i=0;i<steps;++i) {
      Vec3 squared=Vec3::Zero(); std::size_t n=0;
      for(const auto& f:e.forecasts) if(f.model==model && f.status=="complete") {
        squared+=(f.predicted[i].position-f.truth[i].position).array().square().matrix(); ++n;
      }
      const Vec3 rmse=(squared/static_cast<double>(n)).array().sqrt().matrix();
      leads<<name(model)<<','<<(i+1)*c.prediction_step<<','<<std::sqrt(squared.sum()/n)<<',';
      v3(leads,rmse); leads<<','<<n<<'\n';
    }
  }
  leads.close();
  auto events=stream(directory/"events.csv"); events<<"time,type,detail\n";
  for(const auto& event:e.events) events<<event.time<<','<<event.type<<','<<event.detail<<'\n';
  events.close();
}
void print_summary(const Experiment& e) {
  double squared=0,max_error=0; std::size_t saturated=0;
  for(const auto& f:e.frames) {
    squared+=f.position_error*f.position_error; max_error=std::max(max_error,f.position_error);
    saturated+=f.allocation.saturated;
  }
  std::cout<<"Tracking RMSE="<<std::sqrt(squared/e.frames.size())<<" m, max="<<max_error
           <<" m, saturated ticks="<<saturated<<'\n';
  for(auto model:{PredictionModel::CV,PredictionModel::CA}) {
    double ade=0,fde=0; std::size_t complete=0,incomplete=0;
    for(const auto& f:e.forecasts) if(f.model==model) {
      if(f.status=="complete") { ade+=f.ade; fde+=f.fde; ++complete; } else ++incomplete;
    }
    std::cout<<name(model)<<": complete="<<complete<<", incomplete="<<incomplete
             <<", ADE="<<ade/complete<<" m, FDE="<<fde/complete<<" m\n";
  }
}
} // namespace uav

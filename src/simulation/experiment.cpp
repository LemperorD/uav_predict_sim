// SPDX-License-Identifier: GPL-3.0-only
#include "uav_predict_sim/simulation/experiment.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace uav {
namespace {
int ticks(double value,double dt,const char* field) {
  const double n=value/dt;
  if(!std::isfinite(n) || n<1 || n>10000000 || std::abs(n-std::round(n))>1e-7)
    throw std::invalid_argument(std::string(field)+" must be an integer multiple of dt (1..10000000 ticks)");
  return static_cast<int>(std::round(n));
}
}
Experiment run(const Config& c,ReferenceFunction ref) {
  c.vehicle.validate(); c.gains.validate(); c.limits.validate(); c.trajectory.validate();
  if(!std::isfinite(c.dt) || c.dt<=0 || c.dt>c.vehicle.motor_tau/2 ||
     !std::isfinite(c.noise_std) || c.noise_std<0 || !c.initial_position_offset.allFinite())
    throw std::invalid_argument("invalid step, noise or initial offset");
  const int total=ticks(c.duration,c.dt,"duration"), obs=ticks(c.observation_period,c.dt,"observation period"),
      pred=ticks(c.prediction_period,c.dt,"prediction period"), step=ticks(c.prediction_step,c.dt,"prediction step"),
      horizon=ticks(c.horizon,c.dt,"horizon"), history=ticks(c.history_duration,c.dt,"history duration");
  if(pred%obs || step%obs || horizon%step || history%obs || history/obs<2)
    throw std::invalid_argument("periods must align with observation ticks; history requires at least 3 observations");
  const int first=((history+pred-1)/pred)*pred;
  if(first+horizon>total) throw std::invalid_argument("duration must contain history and one full horizon");
  if(!ref) ref=[&](double t) { return reference(c.trajectory,t); };
  // Preflight every integration tick, not just plotting/observation times.
  for(int tick=0;tick<=total;++tick) {
    const auto report=check_flatness(ref(tick*c.dt),c.vehicle,c.limits);
    if(!report.feasible()) throw std::runtime_error("reference infeasible at t="+
        std::to_string(tick*c.dt)+": "+report.violations.front());
  }
  const auto initial=ref(0);
  const auto flat=flatness_map(initial,c.vehicle);
  State state;
  state.position=initial.position+c.initial_position_offset; state.velocity=initial.velocity;
  state.attitude=Eigen::Quaterniond(flat.rotation); state.omega=flat.omega;
  state.rpm=allocate(flat.wrench,c.vehicle).commanded_rpm;
  Vec4 held=state.rpm;
  Quadrotor quad(c.vehicle);
  Experiment result;
  result.frames.reserve(static_cast<std::size_t>(total)+1);
  std::vector<Observation> window;
  std::vector<std::size_t> pending;
  std::mt19937 generator(c.seed);
  std::normal_distribution<double> normal(0,1);
  double previous_norm_error=0;
  bool previous_saturated=false, previous_violation=false;
  for(int tick=0;tick<=total;++tick) {
    state.time=tick*c.dt;
    const auto target=ref(state.time);
    const auto ctrl=control(state,target,held,quad,c.gains,{},c.upstream_controller);
    const auto allocation=allocate(ctrl.requested,c.vehicle);
    held=allocation.commanded_rpm;
    const Vec3 acceleration=quad.acceleration(state);
    const double tilt=std::acos(std::clamp(state.rotation()(2,2),-1.0,1.0));
    const bool violation=tilt>c.limits.max_tilt || state.omega.norm()>c.limits.max_omega ||
                         state.velocity.norm()>c.limits.max_speed || acceleration.norm()>c.limits.max_acceleration;
    result.frames.push_back({state,target,ctrl,allocation,acceleration,
                             (state.position-target.position).norm(),tilt,previous_norm_error,!violation});
    if(allocation.saturated!=previous_saturated) {
      result.events.push_back({state.time,allocation.saturated?"saturation_enter":"saturation_exit","motor_allocation"});
      previous_saturated=allocation.saturated;
    }
    if(violation!=previous_violation) {
      result.events.push_back({state.time,violation?"state_limit_enter":"state_limit_exit","motion_envelope"});
      previous_violation=violation;
    }
    if(state.position.z()<=0) throw std::runtime_error("ground contact at t="+std::to_string(state.time));

    // Resolve pending forecasts only when their target truth is reached.
    for(auto id : pending) {
      auto& forecast=result.forecasts[id];
      const int next=forecast.origin_tick+(static_cast<int>(forecast.truth.size())+1)*forecast.step_ticks;
      if(next==tick) {
        forecast.truth.push_back({state.time,state.position});
        if(forecast.truth.size()==forecast.predicted.size()) {
          double sum=0;
          for(std::size_t i=0;i<forecast.truth.size();++i)
            sum+=(forecast.predicted[i].position-forecast.truth[i].position).norm();
          forecast.ade=sum/forecast.truth.size();
          forecast.fde=(forecast.predicted.back().position-forecast.truth.back().position).norm();
          forecast.status="complete";
        }
      }
    }
    pending.erase(std::remove_if(pending.begin(),pending.end(),[&](std::size_t id) {
      return result.forecasts[id].status=="complete";
    }),pending.end());
    if(tick%obs==0) {
      Vec3 noise;
      for(int axis=0;axis<3;++axis) noise[axis]=c.noise_std*normal(generator);
      window.push_back({state.time,state.position+noise});
      result.observations.push_back(window.back());
      if(window.size()>static_cast<std::size_t>(history/obs+1)) window.erase(window.begin());
      if(tick>=history && tick%pred==0) {
        for(auto model : {PredictionModel::CV,PredictionModel::CA}) {
          const auto start=std::chrono::steady_clock::now();
          auto points=predict(window,model,c.prediction_step,horizon/step);
          const double elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
          Forecast forecast;
          forecast.model=model; forecast.origin_tick=tick; forecast.step_ticks=step; forecast.origin=state.time;
          forecast.predicted=std::move(points); forecast.elapsed_us=elapsed;
          result.forecasts.push_back(std::move(forecast)); pending.push_back(result.forecasts.size()-1);
        }
      }
    }
    if(tick<total) state=quad.step(state,held,c.dt,{},&previous_norm_error);
  }
  return result;
}
} // namespace uav

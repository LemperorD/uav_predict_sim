#include "uav_predict_sim/simulation/experiment.hpp"
#include <iostream>
#include <stdexcept>

void check(bool ok,const char* text) { if(!ok) throw std::runtime_error(text); }
template<class F> void rejects(F call) {
  bool rejected=false;
  try { call(); } catch(const std::exception&) { rejected=true; }
  check(rejected,"invalid input accepted");
}
int main() {
  try {
    using namespace uav;
    for(auto model:{PredictionModel::CV,PredictionModel::CA}) {
      const auto value=[&](double t) -> Vec3 {
        return Vec3(1,2,3)+t*Vec3(2,-1,0.5)+
          (model==PredictionModel::CA?t*t/2:0)*Vec3(0.2,0.3,-0.1);
      };
      std::vector<Observation> history;
      for(int i=0;i<20;++i) history.push_back({100+i*0.02,value(i*0.02)});
      const auto future=predict(history,model,0.1,20);
      for(int i=0;i<20;++i) check((future[i].position-value(0.38+(i+1)*0.1)).norm()<1e-9,"prediction analytic error");
    }
    Config c; c.duration=4; c.horizon=0.5; c.history_duration=0.3; c.observation_period=0.02;
    c.trajectory.scene="eight";
    const auto first=run(c), second=run(c);
    check(first.frames.size()==4001 && first.observations.size()==201,"sample counts");
    check(first.forecasts.size()==76,"forecast counts");
    int complete=0,incomplete=0;
    for(std::size_t i=0;i<first.forecasts.size();++i) {
      const auto& a=first.forecasts[i]; const auto& b=second.forecasts[i];
      if(a.status=="complete") ++complete; else ++incomplete;
      for(std::size_t j=0;j<a.predicted.size();++j)
        check(a.predicted[j].position==b.predicted[j].position,"seed reproducibility");
    }
    check(complete==66 && incomplete==10,"complete horizon accounting");
    const auto modified=run(c,[&](double t) {
      auto sample=reference(c.trajectory,t);
      // Hidden future manoeuvre: C4 at t=2, unavailable to predictor.
      if(t>2) {
        const double u=t-2,k=0.01;
        sample.position.z()+=k*std::pow(u,5); sample.velocity.z()+=5*k*std::pow(u,4);
        sample.acceleration.z()+=20*k*std::pow(u,3); sample.jerk.z()+=60*k*u*u; sample.snap.z()+=120*k*u;
      }
      return sample;
    });
    for(std::size_t i=0;i<first.forecasts.size();++i) {
      if(first.forecasts[i].origin>2) continue;
      for(std::size_t j=0;j<first.forecasts[i].predicted.size();++j)
        check(first.forecasts[i].predicted[j].position==modified.forecasts[i].predicted[j].position,"future reference leakage");
    }
    check((first.frames.back().state.position-modified.frames.back().state.position).norm()>0.1,"future manoeuvre test ineffective");
    c.initial_position_offset=Vec3(0.1,-0.08,0.06);
    const auto perturbed=run(c);
    check(perturbed.frames.back().position_error<0.01,"position feedback recovery");
    c.dt=0; rejects([&] { run(c); });
    c.dt=0.001; c.prediction_period=0.015; rejects([&] { run(c); });
    c.prediction_period=0.1; c.duration=0.2; rejects([&] { run(c); });
    rejects([] { predict({},PredictionModel::CV,0.1,10); });
    std::cout<<"Causal prediction, complete horizon and feedback recovery checks passed\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}

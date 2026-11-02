// SPDX-License-Identifier: GPL-3.0-only
#include "uav_predict_sim/simulation/experiment.hpp"
#include <fstream>
#include <sstream>
#include <map>
#include <cmath>
#include <stdexcept>

namespace uav {
namespace {
// Reader for our unquoted numeric CSV schema, not a general spreadsheet parser.
using Row=std::map<std::string,std::string>;
std::vector<std::string> split(const std::string& text) {
  std::vector<std::string> result; std::string field; std::istringstream input(text);
  while(std::getline(input,field,',')) result.push_back(field);
  if(!text.empty() && text.back()==',') result.emplace_back();
  return result;
}
std::vector<Row> table(const std::filesystem::path& file) {
  std::ifstream input(file);
  if(!input) throw std::runtime_error("cannot open replay file: "+file.string());
  std::string line;
  if(!std::getline(input,line)) throw std::runtime_error("missing CSV header");
  const auto columns=split(line);
  std::vector<Row> result;
  while(std::getline(input,line)) {
    const auto values=split(line);
    if(values.size()!=columns.size()) throw std::runtime_error("invalid CSV row in "+file.string());
    Row row;
    for(std::size_t i=0;i<columns.size();++i)
      if(!row.emplace(columns[i],values[i]).second) throw std::runtime_error("duplicate CSV header");
    result.push_back(std::move(row));
  }
  return result;
}
double number(const Row& row,const char* key) {
  const auto& text=row.at(key); std::size_t used=0;
  const double value=std::stod(text,&used);
  if(used!=text.size() || !std::isfinite(value)) throw std::runtime_error("invalid replay number");
  return value;
}
Vec3 vector(const Row& row,const char* x,const char* y,const char* z) {
  return Vec3(number(row,x),number(row,y),number(row,z));
}
Eigen::Quaterniond quaternion(const Row& row,const char* w,const char* x,const char* y,const char* z) {
  Eigen::Quaterniond q(number(row,w),number(row,x),number(row,y),number(row,z));
  if(std::abs(q.norm()-1)>1e-6) throw std::runtime_error("invalid replay quaternion");
  return q.normalized();
}
PredictionModel model(const std::string& value) {
  if(value=="cv") return PredictionModel::CV;
  if(value=="ca") return PredictionModel::CA;
  throw std::runtime_error("unknown replay prediction model");
}
}
Experiment load_experiment(const std::filesystem::path& directory) {
  const auto config=load_config(directory/"config.yaml");
  const auto states=table(directory/"state.csv"), references=table(directory/"reference.csv"), controls=table(directory/"control.csv");
  if(states.size()<2 || references.size()!=states.size() || controls.size()!=states.size())
    throw std::runtime_error("replay frame counts do not match");
  Experiment e;
  e.frames.reserve(states.size());
  for(std::size_t i=0;i<states.size();++i) {
    const auto& s=states[i]; const auto& r=references[i]; const auto& c=controls[i];
    Frame f;
    f.state.time=number(s,"time");
    if(std::abs(f.state.time-number(r,"time"))>1e-9 || std::abs(f.state.time-number(c,"time"))>1e-9 ||
       std::abs(f.state.time-i*config.dt)>1e-8) throw std::runtime_error("replay timestamp mismatch");
    f.state.position=vector(s,"x","y","z"); f.state.velocity=vector(s,"vx","vy","vz");
    f.state.attitude=quaternion(s,"qw","qx","qy","qz"); f.state.omega=vector(s,"wx","wy","wz");
    f.state.rpm<<number(s,"rpm1"),number(s,"rpm2"),number(s,"rpm3"),number(s,"rpm4");
    f.acceleration=vector(s,"ax","ay","az"); f.tilt=number(s,"tilt");
    f.position_error=number(s,"tracking_error"); f.quaternion_norm_error=number(s,"qnorm_error");
    f.state_feasible=number(s,"state_feasible")!=0;
    f.reference.position=vector(r,"x","y","z"); f.reference.velocity=vector(r,"vx","vy","vz");
    f.reference.acceleration=vector(r,"ax","ay","az"); f.reference.jerk=vector(r,"jx","jy","jz");
    f.reference.snap=vector(r,"sx","sy","sz"); f.reference.heading=number(r,"heading");
    f.reference.heading_rate=number(r,"heading_rate"); f.reference.heading_acceleration=number(r,"heading_acceleration");
    f.control.force_world=vector(c,"fx","fy","fz"); f.control.requested.thrust=number(c,"thrust_cmd");
    f.control.requested.torque=vector(c,"tx_cmd","ty_cmd","tz_cmd");
    f.control.attitude_error=number(c,"attitude_error");
    f.control.desired.rotation=quaternion(c,"rc_qw","rc_qx","rc_qy","rc_qz").toRotationMatrix();
    f.control.desired.omega=vector(c,"wcx","wcy","wcz");
    f.control.desired.alpha=vector(c,"alphacx","alphacy","alphacz");
    f.allocation.commanded_rpm<<number(c,"rpm1_cmd"),number(c,"rpm2_cmd"),number(c,"rpm3_cmd"),number(c,"rpm4_cmd");
    f.allocation.raw_squared_rpm<<number(c,"raw_n1_sq"),number(c,"raw_n2_sq"),number(c,"raw_n3_sq"),number(c,"raw_n4_sq");
    f.allocation.achievable=rotor_wrench(f.allocation.commanded_rpm,config.vehicle);
    f.allocation.saturated=number(c,"saturated")!=0;
    e.frames.push_back(std::move(f));
  }
  for(const auto& row:table(directory/"observation.csv"))
    e.observations.push_back({number(row,"time"),vector(row,"x","y","z")});
  for(const auto& row:table(directory/"metrics.csv")) {
    if(number(row,"forecast_id")!=e.forecasts.size()) throw std::runtime_error("replay forecast id mismatch");
    Forecast f; f.model=model(row.at("model")); f.origin=number(row,"origin"); f.status=row.at("status");
    if(f.status!="complete" && f.status!="incomplete") throw std::runtime_error("unknown forecast status");
    if(f.status=="complete") { f.ade=number(row,"ade_m"); f.fde=number(row,"fde_m"); }
    f.elapsed_us=number(row,"predict_us"); e.forecasts.push_back(std::move(f));
  }
  for(const auto& row:table(directory/"prediction.csv")) {
    const double index=number(row,"forecast_id");
    if(index<0 || std::floor(index)!=index || index>=e.forecasts.size()) throw std::runtime_error("invalid forecast id");
    auto& f=e.forecasts[static_cast<std::size_t>(index)];
    const double t=number(row,"time");
    if(model(row.at("model"))!=f.model || std::abs(number(row,"origin")-f.origin)>1e-9 ||
       std::abs(t-f.origin-(f.predicted.size()+1)*config.prediction_step)>1e-8)
      throw std::runtime_error("prediction ordering mismatch");
    f.predicted.push_back({t,vector(row,"x","y","z")});
    if(!row.at("true_x").empty()) f.truth.push_back({t,vector(row,"true_x","true_y","true_z")});
  }
  for(const auto& f:e.forecasts)
    if(f.predicted.size()!=static_cast<std::size_t>(std::llround(config.horizon/config.prediction_step)) ||
       (f.status=="complete" && f.truth.size()!=f.predicted.size())) throw std::runtime_error("truncated forecast log");
  if(e.forecasts.empty()) throw std::runtime_error("no forecasts in replay");
  for(const auto& row:table(directory/"events.csv")) e.events.push_back({number(row,"time"),row.at("type"),row.at("detail")});
  return e;
}
} // namespace uav

// SPDX-License-Identifier: GPL-3.0-only
#include "uav_predict_sim/simulation/experiment.hpp"
#include <yaml-cpp/yaml.h>
#include <set>
#include <stdexcept>

namespace uav {
namespace {
void keys(const YAML::Node& node,std::initializer_list<const char*> allowed) {
  if(!node) return;
  if(!node.IsMap()) throw std::invalid_argument("configuration section must be a map");
  std::set<std::string> names;
  for(auto name : allowed) names.insert(name);
  std::set<std::string> seen;
  for(const auto& item : node) {
    const auto key=item.first.as<std::string>();
    if(!names.count(key) || !seen.insert(key).second) throw std::invalid_argument("unknown or duplicate configuration key: "+key);
  }
}
template<typename T> void read(const YAML::Node& node,const char* key,T& value) {
  if(node && node[key]) value=node[key].as<T>();
}
void vector(const YAML::Node& node,const char* key,Vec3& value) {
  if(!node || !node[key]) return;
  const auto v=node[key].as<std::vector<double>>();
  if(v.size()!=3) throw std::invalid_argument(std::string(key)+" needs 3 numbers");
  value=Vec3(v[0],v[1],v[2]);
}
YAML::Node vec(const Vec3& value) {
  YAML::Node n; for(int i=0;i<3;++i) n.push_back(value[i]); n.SetStyle(YAML::EmitterStyle::Flow); return n;
}
}
Config load_config(const std::filesystem::path& file) {
  const auto root=YAML::LoadFile(file.string());
  keys(root,{"simulation","vehicle","control","trajectory","observation","prediction","limits"});
  Config c;
  const auto sim=root["simulation"];
  keys(sim,{"duration","dt","initial_position_offset"});
  read(sim,"duration",c.duration); read(sim,"dt",c.dt); vector(sim,"initial_position_offset",c.initial_position_offset);
  const auto vehicle=root["vehicle"];
  keys(vehicle,{"mass","gravity","inertia_diagonal","arm","kf","km","motor_tau","min_rpm","max_rpm","drag"});
  read(vehicle,"mass",c.vehicle.mass); read(vehicle,"gravity",c.vehicle.gravity);
  Vec3 inertia=c.vehicle.inertia.diagonal(); vector(vehicle,"inertia_diagonal",inertia); c.vehicle.inertia=inertia.asDiagonal();
  read(vehicle,"arm",c.vehicle.arm); read(vehicle,"kf",c.vehicle.kf); read(vehicle,"km",c.vehicle.km);
  read(vehicle,"motor_tau",c.vehicle.motor_tau); read(vehicle,"min_rpm",c.vehicle.min_rpm);
  read(vehicle,"max_rpm",c.vehicle.max_rpm); read(vehicle,"drag",c.vehicle.drag);
  const auto ctrl=root["control"];
  keys(ctrl,{"position","velocity","rotation","omega","upstream_compatible"});
  vector(ctrl,"position",c.gains.position); vector(ctrl,"velocity",c.gains.velocity);
  vector(ctrl,"rotation",c.gains.rotation); vector(ctrl,"omega",c.gains.omega);
  read(ctrl,"upstream_compatible",c.upstream_controller);
  const auto traj=root["trajectory"];
  keys(traj,{"scene","time_scale","heading_amplitude"});
  read(traj,"scene",c.trajectory.scene); read(traj,"time_scale",c.trajectory.time_scale);
  read(traj,"heading_amplitude",c.trajectory.heading_amplitude);
  const auto obs=root["observation"];
  keys(obs,{"period","noise_std","seed"});
  read(obs,"period",c.observation_period); read(obs,"noise_std",c.noise_std);
  if(obs && obs["seed"]) {
    const auto seed=obs["seed"].as<long long>();
    if(seed<0 || seed>4294967295LL) throw std::invalid_argument("invalid seed");
    c.seed=static_cast<unsigned>(seed);
  }
  const auto pred=root["prediction"];
  keys(pred,{"period","horizon","step","history_duration"});
  read(pred,"period",c.prediction_period); read(pred,"horizon",c.horizon);
  read(pred,"step",c.prediction_step); read(pred,"history_duration",c.history_duration);
  const auto limits=root["limits"];
  keys(limits,{"max_tilt","max_speed","max_acceleration","max_jerk","max_snap","max_omega","max_alpha"});
  read(limits,"max_tilt",c.limits.max_tilt); read(limits,"max_speed",c.limits.max_speed);
  read(limits,"max_acceleration",c.limits.max_acceleration); read(limits,"max_jerk",c.limits.max_jerk);
  read(limits,"max_snap",c.limits.max_snap); read(limits,"max_omega",c.limits.max_omega); read(limits,"max_alpha",c.limits.max_alpha);
  return c;
}
std::string config_yaml(const Config& c) {
  YAML::Node n;
  auto sim=n["simulation"]; sim["duration"]=c.duration; sim["dt"]=c.dt;
  sim["initial_position_offset"]=vec(c.initial_position_offset);
  auto p=n["vehicle"]; p["mass"]=c.vehicle.mass; p["gravity"]=c.vehicle.gravity;
  p["inertia_diagonal"]=vec(c.vehicle.inertia.diagonal());
  p["arm"]=c.vehicle.arm; p["kf"]=c.vehicle.kf; p["km"]=c.vehicle.km;
  p["motor_tau"]=c.vehicle.motor_tau; p["min_rpm"]=c.vehicle.min_rpm; p["max_rpm"]=c.vehicle.max_rpm; p["drag"]=c.vehicle.drag;
  auto g=n["control"]; g["position"]=vec(c.gains.position); g["velocity"]=vec(c.gains.velocity);
  g["rotation"]=vec(c.gains.rotation); g["omega"]=vec(c.gains.omega); g["upstream_compatible"]=c.upstream_controller;
  auto t=n["trajectory"]; t["scene"]=c.trajectory.scene; t["time_scale"]=c.trajectory.time_scale;
  t["heading_amplitude"]=c.trajectory.heading_amplitude;
  auto o=n["observation"]; o["period"]=c.observation_period; o["noise_std"]=c.noise_std; o["seed"]=c.seed;
  auto f=n["prediction"]; f["period"]=c.prediction_period; f["horizon"]=c.horizon;
  f["step"]=c.prediction_step; f["history_duration"]=c.history_duration;
  auto l=n["limits"]; l["max_tilt"]=c.limits.max_tilt; l["max_speed"]=c.limits.max_speed;
  l["max_acceleration"]=c.limits.max_acceleration; l["max_jerk"]=c.limits.max_jerk; l["max_snap"]=c.limits.max_snap;
  l["max_omega"]=c.limits.max_omega; l["max_alpha"]=c.limits.max_alpha;
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter<<n;
  return std::string(emitter.c_str())+"\n";
}
} // namespace uav

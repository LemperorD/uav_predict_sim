#pragma once
#include "uav_predict_sim/control/so3.hpp"
#include "uav_predict_sim/trajectory/reference.hpp"
#include "uav_predict_sim/prediction/predictor.hpp"
#include <filesystem>
#include <functional>

namespace uav {
struct Config {
  VehicleParams vehicle;
  Gains gains;
  Limits limits;
  TrajectoryParams trajectory;
  double duration=12, dt=0.001, observation_period=0.01, prediction_period=0.1;
  double horizon=2, prediction_step=0.1, history_duration=0.5, noise_std=0.01;
  unsigned seed=42;
  bool upstream_controller=false;
  Vec3 initial_position_offset=Vec3::Zero();
};
Config load_config(const std::filesystem::path& file);
std::string config_yaml(const Config& config);
struct Frame {
  State state;
  FlatSample reference;
  ControlOutput control;
  Allocation allocation;
  Vec3 acceleration=Vec3::Zero();
  double position_error=0, tilt=0, quaternion_norm_error=0;
  bool state_feasible=true;
};
struct Forecast {
  PredictionModel model;
  int origin_tick=0, step_ticks=0;
  double origin=0;
  std::vector<Observation> predicted, truth;
  std::string status="incomplete";
  double ade=0, fde=0, elapsed_us=0;
};
struct Event { double time; std::string type, detail; };
struct Experiment {
  std::vector<Frame> frames;
  std::vector<Observation> observations;
  std::vector<Forecast> forecasts;
  std::vector<Event> events;
};
using ReferenceFunction=std::function<FlatSample(double)>;
Experiment run(const Config& config,ReferenceFunction reference_override={});
void save(const Config& config,const Experiment& experiment,const std::filesystem::path& directory);
void print_summary(const Experiment& experiment);
// Display and plotting consume saved frames; no dependency from physics to GUI.
void plot(const Experiment& experiment,const std::filesystem::path& directory);
void view(const Experiment& experiment,const VehicleParams& vehicle);
} // namespace uav

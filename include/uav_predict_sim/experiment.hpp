#pragma once
#include "uav_predict_sim/core.hpp"
#include <filesystem>

namespace uav {
struct Config {
  std::string scene = "circle";
  double duration = 12.0;
  double dt = 0.01;
  double observation_period = 0.1;
  double prediction_period = 0.2;
  double horizon = 2.0;
  int history_size = 10;
  double noise_std = 0.02;
  unsigned seed = 42;
  double max_acceleration = 5.0;
};
struct Forecast {
  Model model;
  double origin;
  std::vector<Observation> points;
  std::vector<Observation> truth;
  Metrics error;
  double elapsed_us;
};
struct Experiment {
  std::vector<State> states;
  std::vector<Observation> observations;
  std::vector<Forecast> forecasts;
};
Experiment run(const Config& config);
// Creates a new directory, refusing to overwrite an existing run.
void save(const Config& config, const Experiment& experiment,
          const std::filesystem::path& directory);
}  // namespace uav

#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace uav {
using Vec3 = Eigen::Vector3d;

// Right-handed world frame, z up; seconds, metres, metres/second.
struct State {
  double time = 0.0;
  Vec3 position = Vec3::Zero();
  Vec3 velocity = Vec3::Zero();
};

struct Observation {
  double time;
  Vec3 position;
};

// Translational test model with world-frame acceleration input, gravity compensated.
// This is not the EGO-Planner rigid-body simulator.
State integrate(const State& state, const Vec3& acceleration, double dt);
State reference(const std::string& scene, double time);
Vec3 control(const State& state, const State& target, double max_acceleration);

enum class Model { CV, CA };
const char* model_name(Model model);

// Fits a linear/quadratic polynomial to past positions only. Forecasts are
// relative to the most recent observation, excluding the origin itself.
std::vector<Observation> predict(const std::vector<Observation>& history,
                                 Model model, double period, int steps);

struct Metrics {
  double ade;
  double fde;
};
Metrics evaluate(const std::vector<Observation>& predicted,
                 const std::vector<Observation>& truth);
}  // namespace uav

#include "uav_predict_sim/core.hpp"

#include <Eigen/QR>
#include <cmath>
#include <stdexcept>

namespace uav {
State integrate(const State& state, const Vec3& acceleration, double dt) {
  if (!std::isfinite(dt) || dt <= 0 || !acceleration.allFinite())
    throw std::invalid_argument("integration requires finite acceleration and dt > 0");
  return {state.time + dt,
          state.position + state.velocity * dt + 0.5 * acceleration * dt * dt,
          state.velocity + acceleration * dt};
}

State reference(const std::string& scene, double time) {
  if (scene == "hover") return {time, Vec3(0, 0, 1), Vec3::Zero()};
  if (scene == "line") return {time, Vec3(time, 0, 1), Vec3(1, 0, 0)};
  if (scene == "circle") {
    constexpr double radius = 3.0, omega = 0.4;
    const double angle = omega * time;
    return {time, Vec3(radius * std::cos(angle), radius * std::sin(angle), 1),
            Vec3(-radius * omega * std::sin(angle), radius * omega * std::cos(angle), 0)};
  }
  throw std::invalid_argument("scene must be hover, line or circle");
}

Vec3 control(const State& state, const State& target, double max_acceleration) {
  if (!std::isfinite(max_acceleration) || max_acceleration <= 0)
    throw std::invalid_argument("max acceleration must be positive");
  Vec3 acceleration = 4.0 * (target.position - state.position) +
                      2.8 * (target.velocity - state.velocity);
  const double norm = acceleration.norm();
  if (norm > max_acceleration) acceleration *= max_acceleration / norm;
  return acceleration;
}

const char* model_name(Model model) { return model == Model::CV ? "cv" : "ca"; }

std::vector<Observation> predict(const std::vector<Observation>& history,
                                 Model model, double period, int steps) {
  const int columns = model == Model::CV ? 2 : 3;
  if (history.size() < static_cast<std::size_t>(columns) ||
      !std::isfinite(period) || period <= 0 || steps <= 0)
    throw std::invalid_argument("insufficient history or invalid prediction horizon");
  for (std::size_t i = 0; i < history.size(); ++i) {
    if (!std::isfinite(history[i].time) || !history[i].position.allFinite() ||
        (i && history[i].time <= history[i - 1].time))
      throw std::invalid_argument("history must be finite and strictly increasing");
  }
  // Normalized local time keeps the fit well conditioned across sampling rates.
  const double origin = history.back().time;
  const double scale = origin - history.front().time;
  Eigen::MatrixXd design(history.size(), columns), positions(history.size(), 3);
  for (std::size_t i = 0; i < history.size(); ++i) {
    const double t = (history[i].time - origin) / scale;
    design(i, 0) = 1;
    design(i, 1) = t;
    if (columns == 3) design(i, 2) = 0.5 * t * t;
    positions.row(i) = history[i].position.transpose();
  }
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(design);
  if (qr.rank() != columns) throw std::invalid_argument("rank deficient history");
  const Eigen::MatrixXd coefficients = qr.solve(positions);
  std::vector<Observation> result;
  result.reserve(steps);
  for (int i = 1; i <= steps; ++i) {
    const double t = i * period / scale;
    Vec3 position = coefficients.row(0).transpose() + t * coefficients.row(1).transpose();
    if (columns == 3) position += 0.5 * t * t * coefficients.row(2).transpose();
    result.push_back({origin + i * period, position});
  }
  return result;
}

Metrics evaluate(const std::vector<Observation>& predicted,
                 const std::vector<Observation>& truth) {
  if (predicted.empty() || predicted.size() != truth.size())
    throw std::invalid_argument("evaluation requires equal nonempty trajectories");
  double total = 0, last = 0;
  for (std::size_t i = 0; i < predicted.size(); ++i) {
    if (!std::isfinite(predicted[i].time) || !std::isfinite(truth[i].time) ||
        std::abs(predicted[i].time - truth[i].time) > 1e-8 ||
        !predicted[i].position.allFinite() || !truth[i].position.allFinite())
      throw std::invalid_argument("evaluation requires finite, time-aligned trajectories");
    last = (predicted[i].position - truth[i].position).norm();
    total += last;
  }
  return {total / predicted.size(), last};
}
}  // namespace uav

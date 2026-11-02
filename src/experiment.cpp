#include "uav_predict_sim/experiment.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <stdexcept>

#ifndef UAV_GIT_COMMIT
#define UAV_GIT_COMMIT "unknown"
#endif

namespace uav {
namespace {
int ticks(double period, double dt, const char* name) {
  const double count = period / dt;
  if (!std::isfinite(count) || count < 1 || count > std::numeric_limits<int>::max() - 1 ||
      std::abs(count - std::round(count)) > 1e-7)
    throw std::invalid_argument(std::string(name) + " must be a positive integer multiple of dt");
  return static_cast<int>(std::round(count));
}
std::ofstream output(const std::filesystem::path& path) {
  std::ofstream stream;
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.open(path);
  stream << std::setprecision(17);
  return stream;
}
void xyz(std::ostream& out, const Vec3& point) {
  out << point.x() << ',' << point.y() << ',' << point.z();
}
}  // namespace

Experiment run(const Config& config) {
  if (!std::isfinite(config.dt) || config.dt <= 0 || config.history_size < 3 ||
      !std::isfinite(config.noise_std) || config.noise_std < 0 ||
      !std::isfinite(config.max_acceleration) || config.max_acceleration <= 0)
    throw std::invalid_argument("invalid step, history, noise or acceleration limit");
  const int total = ticks(config.duration, config.dt, "duration");
  const int observation_stride = ticks(config.observation_period, config.dt, "observation period");
  const int prediction_stride = ticks(config.prediction_period, config.dt, "prediction period");
  const int horizon_ticks = ticks(config.horizon, config.dt, "horizon");
  if (prediction_stride % observation_stride || horizon_ticks % observation_stride)
    throw std::invalid_argument("prediction period and horizon must be multiples of observation period");
  const long long warmup = static_cast<long long>(config.history_size - 1) * observation_stride;
  const long long first_origin = ((warmup + prediction_stride - 1) / prediction_stride) * prediction_stride;
  if (first_origin + horizon_ticks > total)
    throw std::invalid_argument("duration too short for history and a complete prediction horizon");

  Experiment result;
  result.states.reserve(static_cast<std::size_t>(total) + 1);
  State state = reference(config.scene, 0);
  for (int tick = 0; tick <= total; ++tick) {
    state.time = tick * config.dt;
    result.states.push_back(state);
    if (tick < total)
      state = integrate(state, control(state, reference(config.scene, state.time),
                                      config.max_acceleration), config.dt);
  }

  std::mt19937 generator(config.seed);
  std::normal_distribution<double> normal(0.0, 1.0);
  std::vector<Observation> history;
  for (int tick = 0; tick <= total; tick += observation_stride) {
    const auto& truth = result.states[tick];
    Vec3 noise;
    for (int axis = 0; axis < 3; ++axis) noise[axis] = config.noise_std * normal(generator);
    history.push_back({truth.time, truth.position + noise});
    result.observations.push_back(history.back());
    if (history.size() > static_cast<std::size_t>(config.history_size)) history.erase(history.begin());
    if (history.size() < static_cast<std::size_t>(config.history_size) ||
        tick % prediction_stride || tick > total - horizon_ticks) continue;
    for (const auto model : {Model::CV, Model::CA}) {
      const auto start = std::chrono::steady_clock::now();
      auto points = predict(history, model, config.observation_period, horizon_ticks / observation_stride);
      const double elapsed = std::chrono::duration<double, std::micro>(
          std::chrono::steady_clock::now() - start).count();
      std::vector<Observation> future;
      for (int offset = observation_stride; offset <= horizon_ticks; offset += observation_stride) {
        const auto& sample = result.states[tick + offset];
        future.push_back({sample.time, sample.position});
      }
      const auto error = evaluate(points, future);
      result.forecasts.push_back({model, truth.time, std::move(points), std::move(future), error, elapsed});
    }
  }
  return result;
}

void save(const Config& config, const Experiment& experiment,
          const std::filesystem::path& directory) {
  if (experiment.states.empty() || experiment.forecasts.empty())
    throw std::invalid_argument("cannot save an empty experiment");
  if (!std::filesystem::create_directories(directory))
    throw std::runtime_error("output directory already exists; choose a new run directory");
  auto metadata = output(directory / "config.txt");
  metadata << "model=gravity_compensated_point_mass\nworld_frame=right_handed_z_up\n"
           << "position_unit=m\ntime_unit=s\nnoise=independent_gaussian_position\n"
           << "git_commit=" << UAV_GIT_COMMIT << "\nscene=" << config.scene
           << "\nduration=" << config.duration << "\ndt=" << config.dt
           << "\nobservation_period=" << config.observation_period
           << "\nprediction_period=" << config.prediction_period << "\nhorizon=" << config.horizon
           << "\nhistory_size=" << config.history_size << "\nnoise_std=" << config.noise_std
           << "\nseed=" << config.seed << "\nmax_acceleration=" << config.max_acceleration
           << "\ncontroller_kp=4\ncontroller_kd=2.8\n"
           << "prediction_fit=least_squares_positions\n"
           << "evaluation=complete_horizons_only\n"
           << "reproducibility=same_executable_and_standard_library\n";
  auto states = output(directory / "states.csv");
  states << "time,x,y,z,vx,vy,vz\n";
  for (const auto& state : experiment.states) {
    states << state.time << ','; xyz(states, state.position); states << ',';
    xyz(states, state.velocity); states << '\n';
  }
  auto observations = output(directory / "observations.csv");
  observations << "time,x,y,z\n";
  for (const auto& sample : experiment.observations) {
    observations << sample.time << ','; xyz(observations, sample.position); observations << '\n';
  }
  auto predictions = output(directory / "predictions.csv");
  predictions << "model,origin,time,lead,x,y,z,true_x,true_y,true_z,error_m\n";
  auto metrics = output(directory / "metrics.csv");
  metrics << "model,origin,ade_m,fde_m,predict_us\n";
  for (const auto& forecast : experiment.forecasts) {
    metrics << model_name(forecast.model) << ',' << forecast.origin << ',' << forecast.error.ade
            << ',' << forecast.error.fde << ',' << forecast.elapsed_us << '\n';
    for (std::size_t i = 0; i < forecast.points.size(); ++i) {
      const auto& sample = forecast.points[i];
      predictions << model_name(forecast.model) << ',' << forecast.origin << ',' << sample.time
                  << ',' << sample.time - forecast.origin << ',';
      xyz(predictions, sample.position); predictions << ',';
      xyz(predictions, forecast.truth[i].position);
      predictions << ',' << (sample.position - forecast.truth[i].position).norm() << '\n';
    }
  }
  auto leads = output(directory / "lead_metrics.csv");
  leads << "model,lead,position_rmse_m,samples\n";
  for (auto model : {Model::CV, Model::CA}) {
    const std::size_t steps = experiment.forecasts.front().points.size();
    for (std::size_t i = 0; i < steps; ++i) {
      double squared_error = 0;
      std::size_t count = 0;
      for (const auto& forecast : experiment.forecasts) {
        if (forecast.model != model) continue;
        squared_error += (forecast.points[i].position - forecast.truth[i].position).squaredNorm();
        ++count;
      }
      leads << model_name(model) << ',' << (i + 1) * config.observation_period << ','
            << std::sqrt(squared_error / count) << ',' << count << '\n';
    }
  }
  // Flush errors are reported as failures rather than silently losing results.
  metadata.close(); states.close(); observations.close(); predictions.close(); metrics.close(); leads.close();
}
}  // namespace uav

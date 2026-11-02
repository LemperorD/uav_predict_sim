// matplotlib-cpp includes Python.h, which must precede system headers.
#include "matplotlibcpp.h"
#include "uav_predict_sim/presentation.hpp"
#include <cmath>

namespace uav {
void plot(const Experiment& experiment, const std::filesystem::path& directory) {
  namespace plt = matplotlibcpp;
  plt::backend("Agg");
  const auto checked_plot = [](const std::vector<double>& x, const std::vector<double>& y,
                               const std::map<std::string, std::string>& options) {
    if (!matplotlibcpp::plot(x, y, options)) {
      PyErr_Print();
      throw std::runtime_error("matplotlib plot failed");
    }
  };
  std::vector<double> x, y, ox, oy;
  for (const auto& state : experiment.states) {
    x.push_back(state.position.x()); y.push_back(state.position.y());
  }
  for (const auto& observation : experiment.observations) {
    ox.push_back(observation.position.x()); oy.push_back(observation.position.y());
  }
  plt::figure_size(1200, 500);
  plt::subplot(1, 2, 1);
  checked_plot(x, y, {{"label", "Truth"}, {"color", "black"}});
  checked_plot(ox, oy, {{"label", "Observations"}, {"linestyle", "none"},
                      {"marker", "."}, {"color", "0.65"}});
  // Show the middle forecast from each method at the same origin.
  const std::size_t middle = (experiment.forecasts.size() / 4) * 2;
  for (std::size_t i = middle; i < middle + 2; ++i) {
    std::vector<double> px, py;
    for (const auto& point : experiment.forecasts.at(i).points) {
      px.push_back(point.position.x()); py.push_back(point.position.y());
    }
    const auto model = experiment.forecasts[i].model;
    checked_plot(px, py, {{"label", model_name(model)},
                         {"color", model == Model::CV ? "tab:blue" : "tab:orange"}});
  }
  plt::title("XY trajectory and one forecast");
  plt::xlabel("x [m]"); plt::ylabel("y [m]"); plt::axis("equal"); plt::legend();
  plt::subplot(1, 2, 2);
  for (auto model : {Model::CV, Model::CA}) {
    std::vector<double> lead, rmse;
    for (std::size_t i = 0; i < experiment.forecasts.front().points.size(); ++i) {
      double sum = 0;
      std::size_t count = 0;
      for (const auto& forecast : experiment.forecasts) {
        if (forecast.model != model) continue;
        sum += (forecast.points[i].position - forecast.truth[i].position).squaredNorm();
        ++count;
      }
      lead.push_back(experiment.forecasts.front().points[i].time - experiment.forecasts.front().origin);
      rmse.push_back(std::sqrt(sum / count));
    }
    checked_plot(lead, rmse, {{"label", model_name(model)},
                            {"color", model == Model::CV ? "tab:blue" : "tab:orange"}});
  }
  plt::title("Position RMSE over all complete forecasts");
  plt::xlabel("Prediction lead [s]"); plt::ylabel("Position RMSE [m]"); plt::legend();
  plt::subplots_adjust({{"wspace", 0.3}, {"bottom", 0.14}});
  plt::save((directory / "summary.png").string());
  plt::close();
}
}  // namespace uav

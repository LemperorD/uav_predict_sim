#include "uav_predict_sim/experiment.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
double number(const std::string& text) {
  std::size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value)) throw std::invalid_argument("invalid number: " + text);
  return value;
}
unsigned integer(const std::string& text) {
  const double value = number(text);
  if (value < 0 || value > std::numeric_limits<unsigned>::max() || value != std::floor(value))
    throw std::invalid_argument("invalid unsigned integer: " + text);
  return static_cast<unsigned>(value);
}
}  // namespace

int main(int argc, char** argv) {
  try {
    uav::Config config;
    std::filesystem::path directory = "outputs/demo";
    for (int i = 1; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "--help") {
        std::cout << "Usage: uav_sim [options]\n"
                  << "  --scene hover|line|circle   --output outputs/new-run\n"
                  << "  --duration 12 --dt 0.01 --observation-period 0.1\n"
                  << "  --prediction-period 0.2 --horizon 2 --history 10\n"
                  << "  --noise 0.02 --seed 42 --max-acceleration 5\n"
                  << "All time values are seconds; noise is position standard deviation in metres.\n";
        return 0;
      }
      if (i + 1 >= argc) throw std::invalid_argument("missing value for " + option);
      const std::string value = argv[++i];
      if (option == "--scene") config.scene = value;
      else if (option == "--output") directory = value;
      else if (option == "--duration") config.duration = number(value);
      else if (option == "--dt") config.dt = number(value);
      else if (option == "--observation-period") config.observation_period = number(value);
      else if (option == "--prediction-period") config.prediction_period = number(value);
      else if (option == "--horizon") config.horizon = number(value);
      else if (option == "--noise") config.noise_std = number(value);
      else if (option == "--max-acceleration") config.max_acceleration = number(value);
      else if (option == "--seed") config.seed = integer(value);
      else if (option == "--history") {
        const unsigned count = integer(value);
        if (count > static_cast<unsigned>(std::numeric_limits<int>::max()))
          throw std::invalid_argument("history too large");
        config.history_size = static_cast<int>(count);
      } else throw std::invalid_argument("unknown option: " + option);
    }
    if (std::filesystem::exists(directory))
      throw std::runtime_error("output already exists: " + directory.string());
    const auto experiment = uav::run(config);
    uav::save(config, experiment, directory);
    for (auto model : {uav::Model::CV, uav::Model::CA}) {
      double ade = 0, fde = 0;
      std::size_t count = 0;
      for (const auto& forecast : experiment.forecasts) {
        if (forecast.model != model) continue;
        ade += forecast.error.ade; fde += forecast.error.fde; ++count;
      }
      std::cout << uav::model_name(model) << ": " << count << " forecasts, ADE=" << ade / count
                << " m, FDE=" << fde / count << " m\n";
    }
    std::cout << "Saved " << std::filesystem::absolute(directory) << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }
}

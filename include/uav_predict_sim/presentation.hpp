#pragma once
#include "uav_predict_sim/experiment.hpp"

namespace uav {
void plot(const Experiment& experiment, const std::filesystem::path& directory);
void view(const Experiment& experiment);
}  // namespace uav

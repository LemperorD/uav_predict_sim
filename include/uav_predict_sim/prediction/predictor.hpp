#pragma once
#include "uav_predict_sim/types/model.hpp"

namespace uav {
struct Observation { double time; Vec3 position; };
enum class PredictionModel { CV, CA };
const char* name(PredictionModel model);
// The predictor cannot access the simulator, trajectory or future observations.
std::vector<Observation> predict(const std::vector<Observation>& history,
    PredictionModel model,double period,int steps);
} // namespace uav

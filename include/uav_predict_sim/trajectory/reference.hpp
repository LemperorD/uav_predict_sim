#pragma once
#include "uav_predict_sim/types/model.hpp"

namespace uav {
// All coordinates have analytic derivatives to snap. No hidden future access
// is granted to predictors through this trajectory generator.
struct TrajectoryParams {
  std::string scene = "helix";
  double time_scale = 1, heading_amplitude = 0.3;
  void validate() const;
};
FlatSample reference(const TrajectoryParams& params, double time);
} // namespace uav

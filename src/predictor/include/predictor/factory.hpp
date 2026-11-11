#pragma once

#include <memory>
#include <string>
#include "core/predictor.hpp"
#include "predictor/imm.hpp"

namespace predictor
{
  // Algorithm construction is isolated here; ROS transport only owns Predictor.
  std::unique_ptr<Predictor> makePredictor(const std::string &algorithm,
                                           const Settings    &imm_settings = {},
                                           bool               use_attitude = true);
}  // namespace predictor

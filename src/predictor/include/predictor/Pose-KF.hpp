#pragma once

#include "core/predictor.hpp"

namespace predictor
{
  // Algorithm placeholder: remains abstract until update(), predict() and reset()
  // are implemented. The internal state is entirely up to this algorithm.
  class PoseKF : public Predictor
  {
   public:
    PoseKF()           = default;
    ~PoseKF() override = default;
  };
}  // namespace predictor

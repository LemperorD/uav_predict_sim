#pragma once
#include "uav_predict_sim/types/model.hpp"

namespace uav {
struct Allocation {
  Vec4 raw_squared_rpm = Vec4::Zero();
  Vec4 commanded_rpm = Vec4::Zero();
  Wrench achievable;
  bool saturated = false;
};
Eigen::Matrix4d allocation_matrix(const VehicleParams& params);
Wrench rotor_wrench(const Vec4& rpm, const VehicleParams& params);
Allocation allocate(const Wrench& requested, const VehicleParams& params);
} // namespace uav

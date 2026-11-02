#pragma once
#include "uav_predict_sim/actuation/rotors.hpp"

namespace uav {
struct AttitudeReference {
  Mat3 rotation = Mat3::Identity();
  Mat3 rotation_rate = Mat3::Zero(), rotation_acceleration = Mat3::Zero();
  Vec3 omega = Vec3::Zero(), alpha = Vec3::Zero();
  Wrench wrench;
};
// Analytic first/second derivatives of the SAME force/heading attitude map.
AttitudeReference force_attitude(const Vec3& force, const Vec3& force_rate,
    const Vec3& force_acceleration, double heading, double heading_rate,
    double heading_acceleration, const VehicleParams& params);
AttitudeReference flatness_map(const FlatSample& sample, const VehicleParams& params);
ConstraintReport check_flatness(const FlatSample& sample, const VehicleParams& params,
                                const Limits& limits);
} // namespace uav

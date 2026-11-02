#pragma once
#include "uav_predict_sim/actuation/rotors.hpp"

namespace uav {
class Quadrotor {
 public:
  explicit Quadrotor(VehicleParams params = {});
  const VehicleParams& params() const { return params_; }
  Vec3 acceleration(const State& state, const Disturbance& disturbance = {}) const;
  // Derivative under the currently held motor command and constant disturbance.
  Vec3 jerk(const State& state, const Vec4& motor_command,
            const Disturbance& disturbance = {}) const;
  State step(const State& state, const Vec4& motor_command, double dt,
             const Disturbance& disturbance = {}, double* quaternion_norm_error = nullptr) const;
 private:
  VehicleParams params_;
};
} // namespace uav

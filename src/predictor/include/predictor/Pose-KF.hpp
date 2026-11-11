#pragma once

#include <Eigen/Dense>
#include "core/diagnostics.hpp"
#include "core/predictor.hpp"

namespace predictor
{
  struct PoseKFSettings
  {
    double process_variance     = 0.01;  // paper sigma_q (variance, not stddev)
    double measurement_variance = 0.1;   // paper sigma_r
    double gravity              = 9.81;
    double world_up_sign        = 1.0;  // +1: ROS ENU; -1: paper's down-positive convention
  };

  // Six states [world position, world velocity]. Attitude is an external input;
  // velocity/acceleration measurements are deliberately not used.
  class PoseKF final : public Predictor, public DiagnosticsProvider
  {
   public:
    explicit PoseKF(PoseKFSettings settings = {}, bool use_attitude = true);
    void               update(const Observation &observation) override;
    PositionTrajectory predict(double step, int count) const override;
    void               reset() override;
    Diagnostics        diagnostics() const override;

   private:
    using State      = Eigen::Matrix<double, 6, 1>;
    using Covariance = Eigen::Matrix<double, 6, 6>;
    void                  propagate(State &state, Covariance &covariance, double dt,
                                    const Eigen::Quaterniond &orientation) const;
    PoseKFSettings        settings_;
    bool                  use_attitude_;
    std::optional<double> timestamp_;
    State                 state_       = State::Zero();
    Covariance            covariance_  = Covariance::Identity();
    Eigen::Quaterniond    orientation_ = Eigen::Quaterniond::Identity();
    mutable std::optional<Eigen::Matrix3d> forecast_covariance_;
  };
}  // namespace predictor

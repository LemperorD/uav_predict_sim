#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace predictor
{
  struct ModelProbability
  {
    std::string name;
    double      probability;
  };

  // Optional simulation/visualization data. Absence is represented by NaN/nullopt,
  // never by an invented zero covariance or a fabricated estimated position.
  // This interface is deliberately separate from the position-only Predictor contract.
  struct Diagnostics
  {
    static constexpr double           unavailable      = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d                   position         = Eigen::Vector3d::Constant(unavailable);
    Eigen::Vector3d                   velocity         = Eigen::Vector3d::Constant(unavailable);
    Eigen::Vector3d                   angular_velocity = Eigen::Vector3d::Constant(unavailable);
    std::optional<Eigen::Quaterniond> orientation;
    std::optional<Eigen::Matrix<double, 6, 6>> pose_covariance;  // world rotation axes
    std::optional<Eigen::Matrix<double, 6, 6>> twist_covariance;
    std::optional<Eigen::Matrix3d> forecast_endpoint_covariance;  // most recent predict() horizon
    std::vector<ModelProbability>  model_probabilities;
  };

  class DiagnosticsProvider
  {
   public:
    virtual ~DiagnosticsProvider()          = default;
    virtual Diagnostics diagnostics() const = 0;
  };
}  // namespace predictor

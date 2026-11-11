#ifndef PREDICTOR_CORE_PREDICTOR_HPP
#define PREDICTOR_CORE_PREDICTOR_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

namespace predictor
{
  // SI units, one common world frame. Orientation maps body coordinates to world.
  // Algorithms choose which optional measurements they need; no internal state is prescribed.
  struct Observation
  {
    double                         timestamp = 0;  // seconds, strictly increasing between updates
    Eigen::Vector3d                position  = Eigen::Vector3d::Zero();
    std::optional<Eigen::Vector3d> velocity;
    std::optional<Eigen::Vector3d> acceleration;
    std::optional<Eigen::Quaterniond> orientation;
    std::optional<Eigen::Vector3d>    angular_velocity;  // body frame

    void validate() const
    {
      if (!std::isfinite(timestamp) || !position.allFinite())
      {
        throw std::invalid_argument("Observation requires finite timestamp and position");
      }
    }
  };

  // Exactly count world-frame XYZ positions at t+step, ..., t+count*step.
  // No current point, velocity, attitude, covariance or algorithm-specific state in this output.
  using PositionTrajectory = std::vector<Eigen::Vector3d>;

  class Predictor
  {
   public:
    virtual ~Predictor()                                              = default;
    virtual void               update(const Observation &observation) = 0;
    virtual PositionTrajectory predict(double step, int count) const  = 0;
    virtual void               reset()                                = 0;

   protected:
    static void validateHorizon(double step, int count)
    {
      if (!std::isfinite(step) || step <= 0 || count <= 0 || count > 10000 ||
          !std::isfinite(step * count))
      {
        throw std::invalid_argument(
            "Prediction requires positive finite step and count in [1,10000]");
      }
    }
  };
}  // namespace predictor

#endif  // PREDICTOR_CORE_PREDICTOR_HPP

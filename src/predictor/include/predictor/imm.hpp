#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace predictor
{
  // Error state: world position/velocity/acceleration, local rotation error,
  // body angular velocity. Rotation error is zero at the nominal state.
  using State       = Eigen::Matrix<double, 15, 1>;
  using Covariance  = Eigen::Matrix<double, 15, 15>;
  using Measurement = Eigen::Matrix<double, 6, 1>;

  struct ModelState
  {
    State              mean        = State::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // body -> world
  };

  // All models share the same nominal/error state, so IMM mixing is well defined.
  // Implementations must be immutable: forecast copies share model instances.
  class MotionModel
  {
   public:
    virtual ~MotionModel()                                                  = default;
    virtual std::string name() const                                        = 0;
    virtual ModelState  propagate(const ModelState &state, double dt) const = 0;
    virtual Covariance  processNoise(double dt) const                       = 0;
  };

  struct Settings
  {
    double                   position_std = 0.15, velocity_std = 0.1;
    double                   attitude_std = 0.05, angular_velocity_std = 0.1;
    double                   cv_accel_density = 0.3, ca_jerk_density = 1.0;
    double                   angular_accel_density = 0.3;
    double                   stay_probability      = 0.98;
    std::vector<std::string> models                = {"cv", "ca"};
    // Row source -> column destination. Empty uses symmetric stay_probability.
    Eigen::MatrixXd transition;
    Eigen::VectorXd initial_probability;
    void            validate() const;
  };

  struct Estimate : ModelState
  {
    Covariance      covariance = Covariance::Identity();
    Eigen::VectorXd probability;
  };

  // Low-level numerical filter engine. Applications use ImmPredictor through
  // core/predictor.hpp; this stateful API stays available for numerical tests.
  class Imm
  {
   public:
    explicit Imm(Settings settings = {});
    Imm(Settings settings, std::vector<std::shared_ptr<const MotionModel>> models);
    Estimate update(const Measurement &measurement, double dt);
    Estimate update(const Measurement &measurement, const Eigen::Quaterniond &orientation,
                    double dt, std::optional<Eigen::Vector3d> angular_velocity = std::nullopt);
    std::vector<Estimate> forecast(double step, int count) const;
    Estimate              estimate() const;

   private:
    Estimate updateImpl(const Measurement &, double, const Eigen::Quaterniond *,
                        const std::optional<Eigen::Vector3d> &);
    void     predict(double dt);
    Settings settings_;
    bool     initialized_ = false;
    std::vector<std::shared_ptr<const MotionModel>> models_;
    std::vector<ModelState>                         x_;
    std::vector<Covariance>                         p_;
    Eigen::MatrixXd                                 transition_;
    Eigen::VectorXd                                 weights_;
  };
}  // namespace predictor

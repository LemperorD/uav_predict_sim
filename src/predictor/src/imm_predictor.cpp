#include "predictor/imm_predictor.hpp"

namespace predictor
{
  ImmPredictor::ImmPredictor(Settings settings, bool use_attitude)
      : settings_(std::move(settings)), use_attitude_(use_attitude), filter_(settings_)
  {
  }

  void ImmPredictor::reset()
  {
    filter_ = Imm(settings_);
    last_timestamp_.reset();
    diagnostics_ = Diagnostics{};
  }

  void ImmPredictor::update(const Observation &observation)
  {
    observation.validate();
    if (!observation.velocity || !observation.velocity->allFinite())
    {
      throw std::invalid_argument("IMM requires finite world velocity observations");
    }
    if (last_timestamp_ && observation.timestamp <= *last_timestamp_)
    {
      throw std::invalid_argument("Observation timestamps must be strictly increasing");
    }
    if (use_attitude_ &&
        (!observation.orientation || !observation.orientation->coeffs().allFinite() ||
         !std::isfinite(observation.orientation->norm()) ||
         observation.orientation->norm() < 1e-12))
    {
      throw std::invalid_argument("Attitude-enabled IMM requires a finite nonzero quaternion");
    }
    if (use_attitude_ && observation.angular_velocity && !observation.angular_velocity->allFinite())
    {
      throw std::invalid_argument("IMM requires finite angular velocity");
    }
    const double dt = last_timestamp_ ? observation.timestamp - *last_timestamp_ : .01;
    if (!std::isfinite(dt))
    {
      throw std::invalid_argument("Observation time interval must be finite");
    }
    Measurement measurement;
    measurement << observation.position, *observation.velocity;
    const auto estimate           = use_attitude_
                                        ? filter_.update(measurement, observation.orientation->normalized(),
                                                         dt, observation.angular_velocity)
                                        : filter_.update(measurement, dt);
    last_timestamp_               = observation.timestamp;
    diagnostics_                  = Diagnostics{};
    diagnostics_.position         = estimate.mean.head<3>();
    diagnostics_.velocity         = estimate.mean.segment<3>(3);
    diagnostics_.angular_velocity = estimate.mean.tail<3>();
    if (use_attitude_)
    {
      diagnostics_.orientation = estimate.orientation;
    }
    Eigen::Matrix<double, 6, 15> pose_map = Eigen::Matrix<double, 6, 15>::Zero();
    pose_map.topLeftCorner<3, 3>().setIdentity();
    pose_map.block<3, 3>(3, 9)             = estimate.orientation.toRotationMatrix();
    Eigen::Matrix<double, 6, 15> twist_map = Eigen::Matrix<double, 6, 15>::Zero();
    twist_map.block<3, 3>(0, 3).setIdentity();
    twist_map.block<3, 3>(3, 12).setIdentity();
    diagnostics_.pose_covariance  = pose_map * estimate.covariance * pose_map.transpose();
    diagnostics_.twist_covariance = twist_map * estimate.covariance * twist_map.transpose();
    for (size_t i = 0; i < settings_.models.size(); ++i)
    {
      diagnostics_.model_probabilities.push_back({settings_.models[i], estimate.probability(i)});
    }
  }

  PositionTrajectory ImmPredictor::predict(double step, int count) const
  {
    validateHorizon(step, count);
    if (!last_timestamp_)
    {
      throw std::logic_error("Predictor must receive an observation before predicting");
    }
    const auto         future = filter_.forecast(step, count);
    PositionTrajectory positions;
    positions.reserve(future.size());
    for (const auto &point : future)
    {
      positions.emplace_back(point.mean.head<3>());
    }
    diagnostics_.forecast_endpoint_covariance = future.back().covariance.topLeftCorner<3, 3>();
    return positions;
  }

  Diagnostics ImmPredictor::diagnostics() const { return diagnostics_; }
}  // namespace predictor

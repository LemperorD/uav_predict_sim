#include "predictor/Pose-KF.hpp"

namespace predictor
{
  PoseKF::PoseKF(PoseKFSettings settings, bool use_attitude)
      : settings_(settings), use_attitude_(use_attitude)
  {
    if (!std::isfinite(settings.process_variance) || settings.process_variance < 0 ||
        !std::isfinite(settings.measurement_variance) || settings.measurement_variance <= 0 ||
        !std::isfinite(settings.gravity) || settings.gravity <= 0 ||
        (settings.world_up_sign != 1 && settings.world_up_sign != -1))
      throw std::invalid_argument("Invalid Pose-KF variances, gravity or world_up_sign");
  }

  void PoseKF::propagate(State &x, Covariance &p, double dt, const Eigen::Quaterniond &q) const
  {
    const Eigen::Vector3d axis = q * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d acceleration =
        use_attitude_
            ? (settings_.world_up_sign * settings_.gravity * (axis - Eigen::Vector3d::UnitZ()))
                  .eval()
            : Eigen::Vector3d::Zero();
    // Forward Euler, as in equation (23); no half-dt-squared position term.
    x.head<3>() += dt * x.tail<3>();
    x.tail<3>() += dt * acceleration;
    Covariance f             = Covariance::Identity();
    f.topRightCorner<3, 3>() = dt * Eigen::Matrix3d::Identity();
    // The paper prints a 6x1 Gamma but a 6x6 Q. Interpret Gamma's six
    // entries as a diagonal map so Gamma Q Gamma^T is dimensionally valid.
    State gamma = State::Ones();
    if (use_attitude_) gamma.tail<3>() += dt * settings_.world_up_sign * axis;
    p = (f * p * f.transpose()).eval();
    p.diagonal() += settings_.process_variance * gamma.array().square().matrix();
    p = (0.5 * (p + p.transpose())).eval();
    if (!x.allFinite() || !p.allFinite()) throw std::overflow_error("Pose-KF propagation overflow");
  }

  void PoseKF::update(const Observation &o)
  {
    o.validate();
    if (timestamp_ && (o.timestamp <= *timestamp_ || !std::isfinite(o.timestamp - *timestamp_)))
      throw std::invalid_argument("Observation timestamps must be strictly increasing and finite");
    if (use_attitude_ && (!o.orientation || !o.orientation->coeffs().allFinite() ||
                          !std::isfinite(o.orientation->norm()) || o.orientation->norm() < 1e-12))
      throw std::invalid_argument("Pose-KF requires a finite nonzero body-to-world quaternion");
    const Eigen::Quaterniond q =
        use_attitude_ ? o.orientation->normalized() : Eigen::Quaterniond::Identity();
    State      x = state_;
    Covariance p = covariance_;
    if (!timestamp_)
    {
      x.setZero();
      x.head<3>() = o.position;
      p.setIdentity();
    }
    else
    {
      propagate(x, p, o.timestamp - *timestamp_, q);
      const Eigen::Matrix3d r = settings_.measurement_variance * Eigen::Matrix3d::Identity();
      const Eigen::Matrix3d innovation = p.topLeftCorner<3, 3>() + r;
      const Eigen::Matrix<double, 6, 3> k =
          innovation.ldlt().solve(p.leftCols<3>().transpose()).transpose();
      x += k * (o.position - x.head<3>());
      Covariance correction = Covariance::Identity();
      correction.leftCols<3>() -= k;
      p = (correction * p * correction.transpose() + k * r * k.transpose()).eval();
      p = (0.5 * (p + p.transpose())).eval();
    }
    if (!x.allFinite() || !p.allFinite()) throw std::runtime_error("Nonfinite Pose-KF update");
    state_       = x;
    covariance_  = p;
    timestamp_   = o.timestamp;
    orientation_ = q;
    forecast_covariance_.reset();
  }

  PositionTrajectory PoseKF::predict(double step, int count) const
  {
    validateHorizon(step, count);
    if (!timestamp_) throw std::logic_error("Pose-KF requires an observation before predicting");
    State              x = state_;
    Covariance         p = covariance_;
    PositionTrajectory result;
    result.reserve(count);
    for (int i = 0; i < count; ++i)
    {
      // Hold the last observed attitude; future attitudes are unavailable.
      propagate(x, p, step, orientation_);
      result.push_back(x.head<3>());
    }
    forecast_covariance_ = p.topLeftCorner<3, 3>();
    return result;
  }

  void PoseKF::reset()
  {
    timestamp_.reset();
    state_.setZero();
    covariance_.setIdentity();
    orientation_.setIdentity();
    forecast_covariance_.reset();
  }

  Diagnostics PoseKF::diagnostics() const
  {
    Diagnostics d;
    if (!timestamp_) return d;
    d.position = state_.head<3>();
    d.velocity = state_.tail<3>();
    if (use_attitude_) d.orientation = orientation_;  // observed input, not an estimated attitude
    Eigen::Matrix<double, 6, 6> pose =
        Eigen::Matrix<double, 6, 6>::Constant(Diagnostics::unavailable);
    Eigen::Matrix<double, 6, 6> twist = pose;
    pose.topLeftCorner<3, 3>()        = covariance_.topLeftCorner<3, 3>();
    twist.topLeftCorner<3, 3>()       = covariance_.bottomRightCorner<3, 3>();
    d.pose_covariance                 = pose;
    d.twist_covariance                = twist;
    d.forecast_endpoint_covariance    = forecast_covariance_;
    return d;
  }
}  // namespace predictor

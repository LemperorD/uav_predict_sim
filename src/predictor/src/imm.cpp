#include "predictor/imm.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace predictor
{
  namespace
  {
    Eigen::Quaterniond expRotation(const Eigen::Vector3d &v)
    {
      const double angle = v.norm();
      if (angle < 1e-12)
      {
        return Eigen::Quaterniond(1, v.x() / 2, v.y() / 2, v.z() / 2).normalized();
      }
      return Eigen::Quaterniond(Eigen::AngleAxisd(angle, v / angle));
    }

    Eigen::Vector3d logRotation(Eigen::Quaterniond q)
    {
      q.normalize();
      if (q.w() < 0)
      {
        q.coeffs() *= -1;
      }
      Eigen::AngleAxisd aa(q);
      return aa.angle() * aa.axis();
    }

    ModelState plus(ModelState x, const State &d)
    {
      x.mean += d;
      x.orientation = (x.orientation * expRotation(d.segment<3>(9))).normalized();
      x.mean.segment<3>(9).setZero();
      return x;
    }

    State minus(const ModelState &a, const ModelState &b)
    {
      State d         = a.mean - b.mean;
      d.segment<3>(9) = logRotation(b.orientation.conjugate() * a.orientation);
      return d;
    }

    // Transport a local rotation error covariance between nominal orientations.
    Covariance transport(const ModelState &from, const ModelState &to)
    {
      Covariance       j   = Covariance::Identity();
      constexpr double eps = 1e-6;
      for (int k = 9; k < 12; ++k)
      {
        State d  = State::Zero();
        d(k)     = eps;
        j.col(k) = (minus(plus(from, d), to) - minus(plus(from, -d), to)) / (2 * eps);
      }
      return j;
    }

    ModelState average(const std::vector<ModelState> &x, const Eigen::VectorXd &w)
    {
      ModelState result = x[0];
      result.mean.setZero();
      for (int i = 0; i < w.size(); ++i)
      {
        result.mean += w(i) * x[i].mean;
      }
      for (int iter = 0; iter < 30; ++iter)
      {
        Eigen::Vector3d d = Eigen::Vector3d::Zero();
        for (int i = 0; i < w.size(); ++i)
        {
          d += w(i) * logRotation(result.orientation.conjugate() * x[i].orientation);
        }
        result.orientation = (result.orientation * expRotation(d)).normalized();
        if (d.norm() < 1e-12)
        {
          break;
        }
      }
      return result;
    }

    class KinematicModel : public MotionModel
    {
     public:
      KinematicModel(std::string name, const Settings &s) : name_(std::move(name)), settings_(s) {}
      std::string name() const override { return name_; }
      ModelState  propagate(const ModelState &x, double dt) const override
      {
        ModelState y  = x;
        y.orientation = (x.orientation * expRotation(x.mean.tail<3>() * dt)).normalized();
        y.mean.head<3>() += x.mean.segment<3>(3) * dt;
        if (name_ == "ca")
        {
          y.mean.head<3>() += x.mean.segment<3>(6) * (dt * dt / 2);
          y.mean.segment<3>(3) += x.mean.segment<3>(6) * dt;
        }
        else if (name_ == "pose_kf")
        {
          // Pose-KF's equal-weight thrust hypothesis embedded in the common
          // IMM error state. Current filtered attitude drives acceleration;
          // all hypotheses retain the same measurement update and likelihood.
          const Eigen::Vector3d acceleration =
              settings_.pose_world_up_sign * settings_.pose_gravity *
              (x.orientation * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ());
          y.mean.segment<3>(3) += acceleration * dt;
          y.mean.segment<3>(6) = acceleration;
        }
        else if (name_ == "body_velocity")
        {
          // Constant velocity in the rotating body frame. Exact integrated arc.
          const Eigen::Vector3d omega = x.orientation * x.mean.tail<3>();
          Eigen::Matrix3d       w;
          w << 0, -omega.z(), omega.y(), omega.z(), 0, -omega.x(), -omega.y(), omega.x(), 0;
          const double rate = omega.norm(), angle = rate * dt;
          double       a, b;
          if (std::abs(angle) < 1e-4)
          {
            a = dt * dt * (0.5 - angle * angle / 24);
            b = dt * dt * dt * (1.0 / 6 - angle * angle / 120);
          }
          else
          {
            a = (1 - std::cos(angle)) / (rate * rate);
            b = (angle - std::sin(angle)) / (rate * rate * rate);
          }
          const Eigen::Vector3d v = x.mean.segment<3>(3);
          y.mean.head<3>() += (a * w + b * w * w) * v;
          y.mean.segment<3>(3) = (y.orientation * x.orientation.conjugate()) * v;
          y.mean.segment<3>(6) = omega.cross(y.mean.segment<3>(3));
        }
        else
        {
          y.mean.segment<3>(6).setZero();
        }
        return y;
      }
      Covariance processNoise(double dt) const override
      {
        Covariance q = Covariance::Zero();
        for (int k = 0; k < 3; ++k)
        {
          if (name_ == "ca")
          {
            Eigen::Matrix3d block;
            block << std::pow(dt, 5) / 20, std::pow(dt, 4) / 8, std::pow(dt, 3) / 6,
                std::pow(dt, 4) / 8, std::pow(dt, 3) / 3, dt * dt / 2, std::pow(dt, 3) / 6,
                dt * dt / 2, dt;
            for (int a = 0; a < 3; ++a)
            {
              for (int b = 0; b < 3; ++b)
              {
                q(k + 3 * a, k + 3 * b) = settings_.ca_jerk_density * block(a, b);
              }
            }
          }
          else
          {
            const double n =
                name_ == "pose_kf" ? settings_.pose_accel_density : settings_.cv_accel_density;
            q(k, k)     = n * dt * dt * dt / 3;
            q(k, k + 3) = q(k + 3, k) = n * dt * dt / 2;
            q(k + 3, k + 3)           = n * dt;
            q(k + 6, k + 6)           = n;
          }
          const double n   = settings_.angular_accel_density;
          q(k + 9, k + 9)  = n * dt * dt * dt / 3;
          q(k + 9, k + 12) = q(k + 12, k + 9) = n * dt * dt / 2;
          q(k + 12, k + 12)                   = n * dt;
        }
        return q;
      }

     private:
      std::string name_;
      Settings    settings_;
    };
  }  // namespace

  void Settings::validate() const
  {
    for (double v : {position_std, velocity_std, attitude_std, angular_velocity_std,
                     cv_accel_density, ca_jerk_density, angular_accel_density, pose_accel_density})
    {
      if (!std::isfinite(v) || v <= 0)
      {
        throw std::invalid_argument("IMM noise values must be finite and positive");
      }
    }
    if (!std::isfinite(pose_gravity) || pose_gravity <= 0 ||
        (pose_world_up_sign != 1 && pose_world_up_sign != -1))
      throw std::invalid_argument("Invalid IMM pose gravity or world up sign");
    if (!std::isfinite(stay_probability) || stay_probability <= 0 || stay_probability >= 1)
    {
      throw std::invalid_argument("stay_probability must be in (0,1)");
    }
    const int n = models.size();
    if (n == 0 || std::set<std::string>(models.begin(), models.end()).size() != models.size())
    {
      throw std::invalid_argument("models must be nonempty and unique");
    }
    if (transition.size() && (transition.rows() != n || transition.cols() != n ||
                              !transition.allFinite() || (transition.array() < 0).any() ||
                              (transition.rowwise().sum().array() - 1).abs().maxCoeff() > 1e-10))
    {
      throw std::invalid_argument("transition must be a row-stochastic NxN matrix");
    }
    if (initial_probability.size() &&
        (initial_probability.size() != n || !initial_probability.allFinite() ||
         (initial_probability.array() < 0).any() ||
         std::abs(initial_probability.sum() - 1) > 1e-10))
    {
      throw std::invalid_argument("initial_probability must be a probability vector of length N");
    }
  }

  Imm::Imm(Settings settings) : Imm(std::move(settings), {}) {}

  Imm::Imm(Settings settings, std::vector<std::shared_ptr<const MotionModel>> models)
      : settings_(std::move(settings)), models_(std::move(models))
  {
    settings_.validate();
    const int n = settings_.models.size();
    if (models_.empty())
    {
      for (const auto &name : settings_.models)
      {
        if (name != "cv" && name != "ca" && name != "body_velocity" && name != "pose_kf")
        {
          throw std::invalid_argument("unknown motion model: " + name);
        }
        models_.push_back(std::make_shared<KinematicModel>(name, settings_));
      }
    }
    if (static_cast<int>(models_.size()) != n)
    {
      throw std::invalid_argument("custom model count differs from settings.models");
    }
    for (int i = 0; i < n; ++i)
    {
      if (!models_[i] || models_[i]->name() != settings_.models[i])
      {
        throw std::invalid_argument("custom model name/order differs from settings.models");
      }
    }
    x_.resize(n);
    p_.assign(n, Covariance::Identity());
    weights_    = settings_.initial_probability.size() ? settings_.initial_probability
                                                       : Eigen::VectorXd::Constant(n, 1.0 / n);
    transition_ = settings_.transition;
    if (!transition_.size())
    {
      transition_ =
          Eigen::MatrixXd::Constant(n, n, n == 1 ? 0 : (1 - settings_.stay_probability) / (n - 1));
      transition_.diagonal().setConstant(n == 1 ? 1 : settings_.stay_probability);
    }
  }

  void Imm::predict(double dt)
  {
    if (!std::isfinite(dt) || dt <= 0)
    {
      throw std::invalid_argument("dt must be finite and positive");
    }
    const Eigen::VectorXd prior = transition_.transpose() * weights_;
    const auto            old_x = x_;
    const auto            old_p = p_;
    for (int j = 0; j < weights_.size(); ++j)
    {
      // An unreachable destination has zero mixture weight. Keep its own state.
      if (prior(j) > 0)
      {
        const Eigen::VectorXd w = weights_.cwiseProduct(transition_.col(j)) / prior(j);
        x_[j]                   = average(old_x, w);
        p_[j].setZero();
        for (int i = 0; i < weights_.size(); ++i)
        {
          const State      d = minus(old_x[i], x_[j]);
          const Covariance t = transport(old_x[i], x_[j]);
          p_[j] += w(i) * (t * old_p[i] * t.transpose() + d * d.transpose());
        }
      }
      const ModelState predicted = models_[j]->propagate(x_[j], dt);
      Covariance       f;
      constexpr double eps = 1e-6;
      for (int k = 0; k < 15; ++k)
      {
        State d  = State::Zero();
        d(k)     = eps;
        f.col(k) = (minus(models_[j]->propagate(plus(x_[j], d), dt), predicted) -
                    minus(models_[j]->propagate(plus(x_[j], -d), dt), predicted)) /
                   (2 * eps);
      }
      x_[j] = predicted;
      p_[j] = (f * p_[j] * f.transpose() + models_[j]->processNoise(dt)).eval();
      p_[j] = (0.5 * (p_[j] + p_[j].transpose())).eval();
    }
    weights_ = prior;
  }

  Estimate Imm::update(const Measurement &z, double dt)
  {
    return updateImpl(z, dt, nullptr, std::nullopt);
  }

  Estimate Imm::update(const Measurement &z, const Eigen::Quaterniond &q, double dt,
                       std::optional<Eigen::Vector3d> omega)
  {
    return updateImpl(z, dt, &q, omega);
  }

  Estimate Imm::updateImpl(const Measurement &z, double dt, const Eigen::Quaterniond *q,
                           const std::optional<Eigen::Vector3d> &omega)
  {
    if (!z.allFinite() || !std::isfinite(dt) || dt <= 0 ||
        (q && (!q->coeffs().allFinite() || !std::isfinite(q->norm()) || q->norm() < 1e-12)) ||
        (omega && !omega->allFinite()))
    {
      throw std::invalid_argument("invalid measurement or dt");
    }
    if (!q && std::find(settings_.models.begin(), settings_.models.end(), "pose_kf") !=
                  settings_.models.end())
      throw std::invalid_argument("IMM pose_kf hypothesis requires attitude observations");
    const int       dim = q ? (omega ? 12 : 9) : 6;
    Eigen::MatrixXd h   = Eigen::MatrixXd::Zero(dim, 15);
    h.topLeftCorner(6, 6).setIdentity();
    if (q)
    {
      h.block<3, 3>(6, 9).setIdentity();
    }
    if (omega)
    {
      h.block<3, 3>(9, 12).setIdentity();
    }
    Eigen::MatrixXd r = Eigen::MatrixXd::Zero(dim, dim);
    r.diagonal().head<3>().setConstant(settings_.position_std * settings_.position_std);
    r.diagonal().segment<3>(3).setConstant(settings_.velocity_std * settings_.velocity_std);
    if (q)
    {
      r.diagonal().segment<3>(6).setConstant(settings_.attitude_std * settings_.attitude_std);
    }
    if (omega)
    {
      r.diagonal().tail<3>().setConstant(settings_.angular_velocity_std *
                                         settings_.angular_velocity_std);
    }
    if (!initialized_)
    {
      for (size_t i = 0; i < x_.size(); ++i)
      {
        x_[i].mean.head<6>()        = z;
        p_[i].topLeftCorner<6, 6>() = r.topLeftCorner<6, 6>();
        if (q)
        {
          x_[i].orientation       = q->normalized();
          p_[i].block<3, 3>(9, 9) = r.block<3, 3>(6, 6);
        }
        if (omega)
        {
          x_[i].mean.tail<3>()            = *omega;
          p_[i].bottomRightCorner<3, 3>() = r.bottomRightCorner<3, 3>();
        }
      }
      initialized_ = true;
      return estimate();
    }
    predict(dt);
    Eigen::VectorXd logs(weights_.size());
    for (int j = 0; j < weights_.size(); ++j)
    {
      Eigen::VectorXd residual(dim);
      residual.head<6>() = z - x_[j].mean.head<6>();
      if (q)
      {
        residual.segment<3>(6) = logRotation(x_[j].orientation.conjugate() * q->normalized());
      }
      if (omega)
      {
        residual.tail<3>() = *omega - x_[j].mean.tail<3>();
      }
      const Eigen::MatrixXd        innovation = h * p_[j] * h.transpose() + r;
      Eigen::LDLT<Eigen::MatrixXd> ldlt(innovation);
      if (ldlt.info() != Eigen::Success || (ldlt.vectorD().array() <= 0).any())
      {
        throw std::runtime_error("innovation covariance is not positive definite");
      }
      const Eigen::MatrixXd gain = ldlt.solve((p_[j] * h.transpose()).transpose()).transpose();
      const ModelState      old  = x_[j];
      const State           correction = gain * residual;
      x_[j]                            = plus(old, correction);
      const Covariance a               = Covariance::Identity() - gain * h;
      Covariance       reset           = Covariance::Identity();
      constexpr double eps             = 1e-6;
      for (int k = 9; k < 12; ++k)
      {
        State d = State::Zero();
        d(k)    = eps;
        reset.col(k) =
            (minus(plus(old, correction + d), x_[j]) - minus(plus(old, correction - d), x_[j])) /
            (2 * eps);
      }
      p_[j] =
          (reset * (a * p_[j] * a.transpose() + gain * r * gain.transpose()) * reset.transpose())
              .eval();
      p_[j]   = (0.5 * (p_[j] + p_[j].transpose())).eval();
      logs(j) = weights_(j) > 0
                    ? std::log(weights_(j)) - 0.5 * (residual.dot(ldlt.solve(residual)) +
                                                     ldlt.vectorD().array().log().sum())
                    : -std::numeric_limits<double>::infinity();
    }
    const double max_log = logs.maxCoeff();
    // Scalar exp preserves exact zero for unreachable models (-infinity).
    // Eigen's vectorized exp may clamp -infinity to a tiny positive value.
    for (int j = 0; j < weights_.size(); ++j)
    {
      weights_(j) = std::exp(logs(j) - max_log);
    }
    weights_ /= weights_.sum();
    return estimate();
  }

  Estimate Imm::estimate() const
  {
    Estimate e;
    static_cast<ModelState &>(e) = average(x_, weights_);
    e.covariance.setZero();
    e.probability = weights_;
    for (int i = 0; i < weights_.size(); ++i)
    {
      const State      d = minus(x_[i], e);
      const Covariance t = transport(x_[i], e);
      e.covariance += weights_(i) * (t * p_[i] * t.transpose() + d * d.transpose());
    }
    return e;
  }

  std::vector<Estimate> Imm::forecast(double step, int count) const
  {
    if (!initialized_ || count <= 0 || count > 10000 || !std::isfinite(step) || step <= 0)
    {
      throw std::invalid_argument(
          "forecast requires initialized filter, positive step and count in [1,10000]");
    }
    Imm                   copy = *this;
    std::vector<Estimate> result;
    result.reserve(count);
    for (int i = 0; i < count; ++i)
    {
      copy.predict(step);
      result.push_back(copy.estimate());
    }
    return result;
  }
}  // namespace predictor

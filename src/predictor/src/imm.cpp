#include "predictor/imm.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace predictor
{

    void Settings::validate() const
    {
        for (double v : {position_std, velocity_std, cv_accel_density, ca_jerk_density})
        {
            if (!std::isfinite(v) || v <= 0)
            {
                throw std::invalid_argument("IMM noise values must be finite and positive");
            }
        }

        if (!std::isfinite(stay_probability) || stay_probability <= 0 || stay_probability >= 1)
        {
            throw std::invalid_argument("stay_probability must be in (0,1)");
        }
    }

    Imm::Imm(Settings settings) : settings_(settings)
    {
        settings_.validate();
        for (int i = 0; i < 2; ++i)
        {
            x_[i].setZero();
            p_[i].setIdentity();
        }
    }

    void Imm::predict(double dt)
    {
        if (!std::isfinite(dt) || dt <= 0)
        {
            throw std::invalid_argument("dt must be finite and positive");
        }

        Eigen::Matrix2d transition;
        const double    s = settings_.stay_probability;
        transition << s, 1 - s, 1 - s, s;  // row: source, column: destination
        const Eigen::Vector2d prior = transition.transpose() * weights_;

        const auto old_x = x_;
        const auto old_p = p_;

        for (int j = 0; j < 2; ++j)
        {
            x_[j].setZero();
            p_[j].setZero();
            for (int i = 0; i < 2; ++i)
            {
                x_[j] += weights_(i) * transition(i, j) / prior(j) * old_x[i];
            }
            for (int i = 0; i < 2; ++i)
            {
                State d = old_x[i] - x_[j];
                p_[j] += weights_(i) * transition(i, j) / prior(j) * (old_p[i] + d * d.transpose());
            }

            Covariance f = Covariance::Identity(), q = Covariance::Zero();
            for (int k = 0; k < 3; ++k)
            {
                f(k, k + 3) = dt;
                if (j == 0)
                {
                    f(k + 6, k + 6) = 0;
                    const double n  = settings_.cv_accel_density;
                    q(k, k)         = n * dt * dt * dt / 3;
                    q(k, k + 3) = q(k + 3, k) = n * dt * dt / 2;
                    q(k + 3, k + 3)           = n * dt;
                    q(k + 6, k + 6)           = n;
                }
                else
                {
                    f(k, k + 6)       = dt * dt / 2;
                    f(k + 3, k + 6)   = dt;
                    const double    n = settings_.ca_jerk_density;
                    Eigen::Matrix3d block;
                    block << std::pow(dt, 5) / 20, std::pow(dt, 4) / 8, std::pow(dt, 3) / 6,
                        std::pow(dt, 4) / 8, std::pow(dt, 3) / 3, dt * dt / 2, std::pow(dt, 3) / 6,
                        dt * dt / 2, dt;
                    for (int a = 0; a < 3; ++a)
                    {
                        for (int b = 0; b < 3; ++b)
                        {
                            q(k + 3 * a, k + 3 * b) = n * block(a, b);
                        }
                    }
                }
            }

            x_[j] = (f * x_[j]).eval();
            p_[j] = (f * p_[j] * f.transpose() + q).eval();
        }

        weights_ = prior;
    }

    Estimate Imm::update(const Measurement &z, double dt)
    {
        if (!z.allFinite() || !std::isfinite(dt) || dt <= 0)
        {
            throw std::invalid_argument("invalid measurement or dt");
        }

        Eigen::Matrix<double, 6, 6> r = Eigen::Matrix<double, 6, 6>::Zero();
        r.diagonal().head<3>().setConstant(settings_.position_std * settings_.position_std);
        r.diagonal().tail<3>().setConstant(settings_.velocity_std * settings_.velocity_std);

        if (!initialized_)
        {
            for (int i = 0; i < 2; ++i)
            {
                x_[i].head<6>()             = z;
                p_[i].topLeftCorner<6, 6>() = r;
            }
            initialized_ = true;
            return estimate();
        }

        predict(dt);
        Eigen::Vector2d log_weight;

        for (int j = 0; j < 2; ++j)
        {
            const Measurement                        residual   = z - x_[j].head<6>();
            const Eigen::Matrix<double, 6, 6>        innovation = p_[j].topLeftCorner<6, 6>() + r;
            Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(innovation);
            if (ldlt.info() != Eigen::Success || (ldlt.vectorD().array() <= 0).any())
            {
                throw std::runtime_error("innovation covariance is not positive definite");
            }

            const Eigen::Matrix<double, 9, 6> gain =
                ldlt.solve(p_[j].leftCols<6>().transpose()).transpose();
            x_[j] += gain * residual;

            Covariance a = Covariance::Identity();
            a.leftCols<6>() -= gain;
            // Joseph form keeps the covariance symmetric and positive semidefinite.
            p_[j] = (a * p_[j] * a.transpose() + gain * r * gain.transpose()).eval();
            p_[j] = (0.5 * (p_[j] + p_[j].transpose())).eval();

            log_weight(j) =
                std::log(std::max(weights_(j), 1e-300)) -
                0.5 * (residual.dot(ldlt.solve(residual)) + ldlt.vectorD().array().log().sum());
        }

        weights_ = (log_weight.array() - log_weight.maxCoeff()).exp().max(1e-15);
        weights_ /= weights_.sum();
        return estimate();
    }

    Estimate Imm::estimate() const
    {
        Estimate e;
        e.mean.setZero();
        e.covariance.setZero();
        e.probability = weights_;

        for (int i = 0; i < 2; ++i)
        {
            e.mean += weights_(i) * x_[i];
        }
        for (int i = 0; i < 2; ++i)
        {
            State d = x_[i] - e.mean;
            e.covariance += weights_(i) * (p_[i] + d * d.transpose());
        }
        return e;
    }

    std::vector<Estimate> Imm::forecast(double step, int count) const
    {
        if (!initialized_ || count <= 0 || count > 10000 || !std::isfinite(step) || step <= 0)
        {
            throw std::invalid_argument(
                "forecast requires initialized filter, "
                "positive step and count in [1,10000]");
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

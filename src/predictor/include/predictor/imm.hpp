#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>

namespace predictor
{

    using State       = Eigen::Matrix<double, 9, 1>;  // position, velocity, acceleration
    using Covariance  = Eigen::Matrix<double, 9, 9>;
    using Measurement = Eigen::Matrix<double, 6, 1>;

    struct Settings
    {
        double position_std = 0.15, velocity_std = 0.1;
        double cv_accel_density = 0.3, ca_jerk_density = 1.0;
        double stay_probability = 0.98;

        void validate() const;
    };

    struct Estimate
    {
        State           mean        = State::Zero();
        Covariance      covariance  = Covariance::Identity();
        Eigen::Vector2d probability = Eigen::Vector2d::Constant(0.5);
    };

    class Imm
    {
       public:
        explicit Imm(Settings settings = {});
        Estimate              update(const Measurement &measurement, double dt);
        std::vector<Estimate> forecast(double step, int count) const;
        Estimate              estimate() const;

       private:
        void predict(double dt);

        Settings                  settings_;
        bool                      initialized_ = false;
        std::array<State, 2>      x_;
        std::array<Covariance, 2> p_;
        Eigen::Vector2d           weights_ = Eigen::Vector2d::Constant(0.5);
    };

}  // namespace predictor

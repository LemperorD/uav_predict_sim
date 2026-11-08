#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace predictor
{

    // time, position xyz, velocity xyz, acceleration xyz; world frame, SI units.
    using TrajectorySample = std::array<double, 10>;
    using Vector3          = std::array<double, 3>;

    struct TrajectorySettings
    {
        std::string type = "figure8";
        Vector3     origin{0, 0, 2};
        Vector3     velocity{1, 0.4, 0};
        Vector3     acceleration{0.2, 0, 0};
        Vector3     amplitude{4, 3, 0.6};
        Vector3     frequency{0.35, 0.7, 0.5};  // Angular frequency, rad/s.
        Vector3     phase{0, 0, 0};             // Radians.
        double      radius         = 4;
        double      angular_speed  = 0.35;  // Signed; negative reverses the turn.
        double      circle_phase   = 0;
        double      vertical_speed = 0.2;

        void validate() const
        {
            if (type != "hover" && type != "straight" && type != "acceleration" &&
                type != "circle" && type != "helix" && type != "figure8" && type != "sinusoid")
            {
                throw std::invalid_argument(
                    "trajectory must be hover, straight, acceleration, "
                    "circle, helix, figure8 or sinusoid");
            }

            for (const auto& values : {origin, velocity, acceleration, amplitude, frequency, phase})
            {
                for (double value : values)
                {
                    if (!std::isfinite(value))
                    {
                        throw std::invalid_argument("trajectory vectors must be finite");
                    }
                }
            }

            if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(angular_speed) ||
                !std::isfinite(circle_phase) || !std::isfinite(vertical_speed))
            {
                throw std::invalid_argument(
                    "radius must be positive; circle/helix parameters must be finite");
            }
        }
    };

    // Caller validates settings once before sampling. All derivatives are analytic.
    inline TrajectorySample SampleTrajectory(const TrajectorySettings& s, double t)
    {
        TrajectorySample r{t, s.origin[0], s.origin[1], s.origin[2], 0, 0, 0, 0, 0, 0};

        if (s.type == "straight" || s.type == "acceleration")
        {
            for (int i = 0; i < 3; ++i)
            {
                double a = s.type == "acceleration" ? s.acceleration[i] : 0;
                r[1 + i] += s.velocity[i] * t + 0.5 * a * t * t;
                r[4 + i] = s.velocity[i] + a * t;
                r[7 + i] = a;
            }
        }
        else if (s.type == "circle" || s.type == "helix")
        {
            double angle = s.angular_speed * t + s.circle_phase;
            double x     = s.radius * std::cos(angle);
            double y     = s.radius * std::sin(angle);

            r[1] += x;
            r[2] += y;
            r[4] = -s.angular_speed * y;
            r[5] = s.angular_speed * x;
            r[7] = -s.angular_speed * s.angular_speed * x;
            r[8] = -s.angular_speed * s.angular_speed * y;

            if (s.type == "helix")
            {
                r[3] += s.vertical_speed * t;
                r[6] = s.vertical_speed;
            }
        }
        else if (s.type == "figure8" || s.type == "sinusoid")
        {
            for (int i = 0; i < 3; ++i)
            {
                // Figure eight locks y to twice x's frequency; z stays independent.
                double w     = s.type == "figure8" && i == 1 ? 2 * s.frequency[0] : s.frequency[i];
                double angle = w * t + s.phase[i];
                r[1 + i] += s.amplitude[i] * std::sin(angle);
                r[4 + i] = s.amplitude[i] * w * std::cos(angle);
                r[7 + i] = -s.amplitude[i] * w * w * std::sin(angle);
            }
        }

        return r;
    }

}  // namespace predictor

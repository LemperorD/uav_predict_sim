#include <iostream>
#include <limits>
#include <stdexcept>

#include "predictor/trajectory.hpp"

void Require(bool ok, const char* message) {
  if (!ok) {
    throw std::runtime_error(message);
  }
}

int main() {
  try {
    predictor::TrajectorySettings s;
    // Nondefault values exercise every axis, signed turn rate and phase.
    s.origin = {1, -2, 5};
    s.velocity = {-0.4, 0.2, 0.1};
    s.acceleration = {0.1, -0.2, 0.3};
    s.amplitude = {2, 3, 0.4};
    s.frequency = {0.8, 0.3, 0.4};
    s.phase = {0.2, -0.1, 0.3};
    s.radius = 2.5;
    s.angular_speed = -0.6;
    s.circle_phase = 0.4;
    s.vertical_speed = -0.1;
    for (const auto* type : {"hover", "straight", "acceleration", "circle",
                             "helix", "figure8", "sinusoid"}) {
      s.type = type;
      s.validate();
      for (double t : {0.0, 0.5, 3.0, 12.0}) {
        const double h = 1e-4;
        auto r = predictor::SampleTrajectory(s, t);
        auto lo = predictor::SampleTrajectory(s, t - h);
        auto hi = predictor::SampleTrajectory(s, t + h);
        for (int i = 0; i < 3; ++i) {
          Require(std::abs((hi[1 + i] - lo[1 + i]) / (2 * h) - r[4 + i]) < 1e-6,
                  "position derivative does not match velocity");
          Require(std::abs((hi[4 + i] - lo[4 + i]) / (2 * h) - r[7 + i]) < 1e-6,
                  "velocity derivative does not match acceleration");
        }
        if (s.type == "circle" || s.type == "helix") {
          Require(std::abs(std::hypot(r[1] - s.origin[0], r[2] - s.origin[1]) -
                           s.radius) < 1e-12,
                  "incorrect circle radius");
          Require(
              std::abs(r[3] - s.origin[2] -
                       (s.type == "helix" ? s.vertical_speed * t : 0)) < 1e-12,
              "incorrect altitude");
        }
      }
    }
    predictor::TrajectorySettings defaults;
    auto r = predictor::SampleTrajectory(defaults, 2.0);
    Require(std::abs(r[1] - 4 * std::sin(0.7)) < 1e-12 &&
                std::abs(r[2] - 3 * std::sin(1.4)) < 1e-12 &&
                std::abs(r[3] - 2 - 0.6 * std::sin(1.0)) < 1e-12,
            "default figure8 changed");
    for (int kind = 0; kind < 4; ++kind) {
      auto invalid = defaults;
      if (kind == 0) {
        invalid.type = "unknown";
      }
      if (kind == 1) {
        invalid.radius = 0;
      }
      if (kind == 2) {
        invalid.frequency[1] = std::numeric_limits<double>::quiet_NaN();
      }
      if (kind == 3) {
        invalid.angular_speed = std::numeric_limits<double>::infinity();
      }
      bool rejected = false;
      try {
        invalid.validate();
      } catch (const std::invalid_argument&) {
        rejected = true;
      }
      Require(rejected, "invalid trajectory parameters accepted");
    }
    std::cout << "Seven trajectory types: analytic derivatives and validation "
                 "passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

#include "uav_predict_sim/core.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
template <typename F> void rejects(F function) {
  bool rejected = false;
  try { function(); } catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, "invalid input accepted");
}

int main() {
  try {
    uav::State state;
    const uav::Vec3 acceleration(1, -2, 0.5);
    for (int i = 0; i < 1000; ++i) state = uav::integrate(state, acceleration, 0.01);
    require((state.position - 50 * acceleration).norm() < 1e-9, "constant acceleration position");
    require((state.velocity - 10 * acceleration).norm() < 1e-9, "constant acceleration velocity");
    state = uav::reference("hover", 0);
    for (int i = 0; i < 1000; ++i)
      state = uav::integrate(state, uav::control(state, uav::reference("hover", state.time), 5), 0.01);
    require((state.position - uav::Vec3(0, 0, 1)).norm() < 1e-12, "hover drift");
    for (auto model : {uav::Model::CV, uav::Model::CA}) {
      const auto trajectory = [model](double t) -> uav::Vec3 {
        return uav::Vec3(1, 2, 3) + t * uav::Vec3(2, -1, 0.4) +
               (model == uav::Model::CA ? 0.5 * t * t : 0.0) * uav::Vec3(1, 0.2, -0.1);
      };
      std::vector<uav::Observation> history, truth;
      for (int i = 0; i < 10; ++i) history.push_back({100 + i * 0.1, trajectory(i * 0.1)});
      for (int i = 1; i <= 20; ++i) truth.push_back({100.9 + i * 0.1, trajectory(0.9 + i * 0.1)});
      const auto prediction = uav::predict(history, model, 0.1, 20);
      const auto error = uav::evaluate(prediction, truth);
      require(error.ade < 1e-10 && error.fde < 1e-10, "analytic prediction mismatch");
      auto misaligned = truth;
      misaligned[0].time += 0.01;
      rejects([&] { uav::evaluate(prediction, misaligned); });
      history.back().time = history.front().time;
      rejects([&] { uav::predict(history, model, 0.1, 20); });
    }
    rejects([] { uav::integrate({}, uav::Vec3::Zero(), 0); });
    rejects([] { uav::predict({}, uav::Model::CA, 0.1, 20); });
    std::cout << "Numerical checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

#include <Eigen/Eigenvalues>
#include <iostream>
#include "predictor/factory.hpp"

void require(bool ok, const char *message)
{
  if (!ok) throw std::runtime_error(message);
}
template <class F>
void rejects(F f)
{
  try
  {
    f();
  }
  catch (const std::exception &)
  {
    return;
  }
  throw std::runtime_error("Expected rejection");
}
int main()
{
  try
  {
    predictor::PoseKF filter;
    rejects([&] { filter.predict(.1, 1); });
    predictor::Observation o;
    o.position = {0, 0, 2};
    rejects([&] { filter.update(o); });
    o.orientation = Eigen::Quaterniond::Identity();
    o.velocity    = Eigen::Vector3d::Constant(1e6);  // Must never enter the six-state update.
    filter.update(o);
    require(filter.diagnostics().velocity.norm() == 0,
            "Must initialize velocity without truth/measurement velocity");
    auto hover = filter.predict(.1, 10);
    require((hover.back() - o.position).norm() < 1e-12, "Level flight must cancel gravity");
    rejects([&] { filter.update(o); });
    rejects([&] { filter.predict(0, 1); });
    rejects([&] { filter.predict(.1, 10001); });
    filter.reset();
    require(!filter.diagnostics().position.allFinite(), "Reset must discard diagnostics");
    rejects([&] { filter.predict(.1, 1); });

    // A tilted attitude produces the analytic forward-Euler acceleration trajectory.
    o.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitY()));
    filter.update(o);
    const Eigen::Vector3d acceleration =
        9.81 * (*o.orientation * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ());
    const auto forecast = filter.predict(.1, 10);
    require((forecast.back() - o.position - .45 * acceleration).norm() < 1e-10,
            "Wrong thrust sign/frame or discretization");
    require((filter.predict(.1, 10).back() - forecast.back()).norm() == 0,
            "Forecast mutated online state");
    require(filter.diagnostics().velocity.norm() == 0, "Forecast changed velocity");
    predictor::PoseKFSettings down_settings;
    down_settings.world_up_sign = -1;
    predictor::PoseKF down(down_settings);
    down.update(o);
    require((down.predict(.1, 10).back() - o.position + .45 * acceleration).norm() < 1e-10,
            "Paper down-positive frame broken");

    filter.reset();
    o.orientation = Eigen::Quaterniond::Identity();
    for (int i = 0; i < 600; ++i)
    {
      o.timestamp = i * .1;
      o.position  = {o.timestamp, -.2 * o.timestamp, 2};
      filter.update(o);
      const auto            d = filter.diagnostics();
      const Eigen::Matrix3d p = d.pose_covariance->topLeftCorner<3, 3>();
      require(p.allFinite() && (p - p.transpose()).norm() < 1e-12 &&
                  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(p).eigenvalues().minCoeff() > 0,
              "Covariance must remain symmetric positive definite");
    }
    require((filter.diagnostics().velocity - Eigen::Vector3d(1, -.2, 0)).norm() < .002,
            "Position-only updates failed to infer constant velocity");
    require((filter.predict(.1, 10).back() - o.position - Eigen::Vector3d(1, -.2, 0)).norm() < .003,
            "Constant velocity forecast incorrect");
    const auto before = filter.diagnostics().position;
    o.timestamp += .1;
    o.orientation = Eigen::Quaterniond(0, 0, 0, 0);
    rejects([&] { filter.update(o); });
    require((filter.diagnostics().position - before).norm() == 0, "Rejected update changed state");
    auto baseline = predictor::makePredictor("kf");
    baseline->update(o);  // No usable attitude or velocity required by KF baseline.
    require(baseline->predict(.1, 2).size() == 2, "Factory KF failed");
    require(predictor::makePredictor("pose_kf") != nullptr, "Factory Pose-KF failed");
    rejects([] { predictor::makePredictor("pose_kf", {}, false); });
    rejects(
        []
        {
          predictor::PoseKFSettings s;
          s.measurement_variance = 0;
          predictor::PoseKF f(s);
        });
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

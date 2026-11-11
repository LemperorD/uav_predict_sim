#include <iostream>
#include "predictor/imm.hpp"

void require(bool ok, const char *message)
{
  if (!ok) throw std::runtime_error(message);
}
int main()
{
  try
  {
    predictor::Settings s;
    s.models = {"pose_kf"};
    predictor::Imm         pose(s);
    predictor::Measurement z;
    z << 1, 2, 3, .4, -.2, .1;
    const Eigen::Quaterniond q(Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitY()));
    bool                     rejected = false;
    try
    {
      pose.update(z, .02);
    }
    catch (const std::invalid_argument &)
    {
      rejected = true;
    }
    require(rejected, "Pose hypothesis silently accepted missing attitude");
    pose.update(z, q, .02, Eigen::Vector3d::Zero());
    const auto            before = pose.estimate();
    const Eigen::Vector3d a =
        s.pose_gravity * (q * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ());
    const auto future = pose.forecast(.05, 20).back();
    require((future.mean.head<3>() - z.head<3>() - z.tail<3>() - .475 * a).norm() < 1e-9,
            "Pose hypothesis position propagation/frame incorrect");
    require((future.mean.segment<3>(3) - z.tail<3>() - a).norm() < 1e-9,
            "Pose hypothesis velocity propagation incorrect");
    require((pose.estimate().mean - before.mean).norm() == 0, "Forecast advanced filter");
    s.models = {"cv", "ca", "pose_kf"};
    predictor::Imm fusion(s);
    for (int i = 0; i < 300; ++i)
    {
      const double t = i * .02;
      z.head<3>()    = Eigen::Vector3d(0, 0, 2) + .5 * t * t * a;
      z.tail<3>()    = t * a;
      const auto e   = fusion.update(z, q, .02);
      require(e.mean.allFinite() && e.covariance.allFinite() &&
                  std::abs(e.probability.sum() - 1) < 1e-12 && e.probability.minCoeff() >= 0,
              "Invalid fused state/probability");
      require((e.covariance - e.covariance.transpose()).norm() < 1e-9,
              "Asymmetric fused covariance");
      require(Eigen::SelfAdjointEigenSolver<predictor::Covariance>(e.covariance)
                      .eigenvalues()
                      .minCoeff() > -1e-8,
              "Indefinite fused covariance");
    }
    const auto e = fusion.estimate();
    require((e.mean.segment<3>(3) - z.tail<3>()).norm() < .05, "Fusion lost velocity tracking");
    s.pose_gravity = -1;
    rejected       = false;
    try
    {
      predictor::Imm bad(s);
    }
    catch (const std::invalid_argument &)
    {
      rejected = true;
    }
    require(rejected, "Invalid gravity accepted");
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

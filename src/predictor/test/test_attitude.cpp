#include <iostream>
#include <stdexcept>
#include "predictor/imm.hpp"

namespace
{
  void require(bool ok, const char *message)
  {
    if (!ok)
    {
      throw std::runtime_error(message);
    }
  }

  template <class F>
  void rejects(F f)
  {
    bool threw = false;
    try
    {
      f();
    }
    catch (const std::invalid_argument &)
    {
      threw = true;
    }
    require(threw, "invalid configuration/observation accepted");
  }

  void valid(const predictor::Estimate &e)
  {
    require(e.mean.allFinite() && e.covariance.allFinite(), "nonfinite estimate");
    require(std::abs(e.orientation.norm() - 1) < 1e-12, "quaternion not normalized");
    require(std::abs(e.probability.sum() - 1) < 1e-12 && e.probability.minCoeff() >= 0,
            "invalid model probabilities");
    require((e.covariance - e.covariance.transpose()).norm() < 1e-9, "asymmetric covariance");
    Eigen::SelfAdjointEigenSolver<predictor::Covariance> solver(e.covariance);
    require(solver.eigenvalues().minCoeff() > -1e-9, "indefinite covariance");
  }

  class CustomModel : public predictor::MotionModel
  {
   public:
    std::string           name() const override { return "custom"; }
    predictor::ModelState propagate(const predictor::ModelState &s, double dt) const override
    {
      auto out = s;
      out.mean.head<3>() += dt * s.mean.segment<3>(3);
      return out;
    }
    predictor::Covariance processNoise(double dt) const override
    {
      return predictor::Covariance::Identity() * dt * 0.1;
    }
  };
}  // namespace

int main()
{
  try
  {
    predictor::Settings s;
    s.models = {"body_velocity", "cv", "ca"};
    s.transition.resize(3, 3);
    s.transition << .98, .01, .01, .02, .97, .01, .02, .01, .97;
    s.initial_probability = Eigen::Vector3d(.4, .3, .3);
    predictor::Imm           filter(s), sign_filter(s);
    const Eigen::Quaterniond initial(Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitX()));
    const Eigen::Vector3d    axis = Eigen::Vector3d(0.2, -0.3, 1).normalized();
    predictor::Measurement   z    = predictor::Measurement::Zero();
    predictor::Estimate      e;
    for (int i = 0; i < 500; ++i)
    {
      auto q = initial * Eigen::Quaterniond(Eigen::AngleAxisd(i * .02, axis));
      e      = filter.update(z, q, .02);
      if (i % 2)
      {
        q.coeffs() *= -1;
      }
      const auto signed_e = sign_filter.update(z, q, .02);
      valid(e);
      require(e.orientation.angularDistance(signed_e.orientation) < 1e-9,
              "quaternion sign changed estimate");
      require((e.covariance - signed_e.covariance).norm() < 1e-7,
              "quaternion sign changed covariance");
      require(e.orientation.angularDistance(q) < .04, "attitude did not follow rotation across pi");
    }
    require((e.mean.tail<3>() - axis).norm() < .03,
            "attitude observations did not estimate angular velocity");
    const auto before = filter.estimate();
    const auto future = filter.forecast(.05, 20);
    valid(future.back());
    const auto expected =
        before.orientation * Eigen::Quaterniond(Eigen::AngleAxisd(
                                 before.mean.tail<3>().norm(), before.mean.tail<3>().normalized()));
    require(future.back().orientation.angularDistance(expected) < 1e-7,
            "attitude forecast inaccurate");
    require(filter.estimate().orientation.angularDistance(before.orientation) < 1e-14,
            "forecast modified online attitude");

    // A single attitude-coupled model must integrate the exact turning arc,
    // including a tilted rotation axis (body z mapped into world coordinates).
    s.models = {"body_velocity"};
    s.transition.resize(0, 0);
    s.initial_probability.resize(0);
    predictor::Imm turning(s);
    z << 0, 0, 0, 1, 0, 0;
    turning.update(z, initial, .02, Eigen::Vector3d(0, 0, 1));
    const auto            arc = turning.forecast(.05, 20).back();
    const Eigen::Vector3d expected_position(std::sin(1), std::cos(.7) * (1 - std::cos(1)),
                                            std::sin(.7) * (1 - std::cos(1)));
    require((arc.mean.head<3>() - expected_position).norm() < 1e-8,
            "body velocity model did not predict tilted arc");
    require(arc.probability.size() == 1 && arc.probability(0) == 1, "single model failed");
    valid(arc);
    for (int i = 1; i <= 200; ++i)
    {
      const double t = i * .02;
      const auto   q = initial * Eigen::Quaterniond(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitZ()));
      z.head<3>()    = Eigen::Vector3d(std::sin(t), std::cos(.7) * (1 - std::cos(t)),
                                       std::sin(.7) * (1 - std::cos(t)));
      z.tail<3>()    = q * initial.conjugate() * Eigen::Vector3d::UnitX();
      const auto tracked = turning.update(z, q, .02, Eigen::Vector3d(0, 0, 1));
      valid(tracked);
      require(tracked.orientation.angularDistance(q) < 1e-7 &&
                  (tracked.mean.tail<3>() - Eigen::Vector3d(0, 0, 1)).norm() < 1e-7,
              "direct angular velocity observation failed");
    }
    z << 0, 0, 0, 1, 0, 0;

    // Exact zero transitions and zero initial probabilities remain valid.
    s.models              = {"ca", "cv"};
    s.transition          = Eigen::Matrix2d::Identity();
    s.initial_probability = Eigen::Vector2d(1, 0);
    predictor::Imm isolated(s);
    isolated.update(z, initial, .02);
    valid(isolated.update(z, initial, .02));
    require(isolated.estimate().probability(1) == 0, "unreachable model gained probability");

    // Core accepts user-defined models without changing IMM mixing/update code.
    s.models = {"custom"};
    s.transition.resize(0, 0);
    s.initial_probability.resize(0);
    predictor::Imm custom(s, {std::make_shared<CustomModel>()});
    custom.update(z, .02);
    require(std::abs(custom.forecast(.1, 10).back().mean(0) - 1) < 1e-10, "custom model not used");

    rejects(
        []
        {
          predictor::Settings c;
          c.models = {};
          predictor::Imm f(c);
        });
    rejects(
        []
        {
          predictor::Settings c;
          c.models = {"missing"};
          predictor::Imm f(c);
        });
    rejects(
        []
        {
          predictor::Settings c;
          c.models = {"cv", "cv"};
          predictor::Imm f(c);
        });
    rejects(
        []
        {
          predictor::Settings c;
          c.transition = Eigen::Matrix3d::Identity();
          predictor::Imm f(c);
        });
    rejects(
        []
        {
          predictor::Settings c;
          c.transition = Eigen::Matrix2d::Constant(.3);
          predictor::Imm f(c);
        });
    rejects(
        []
        {
          predictor::Settings c;
          c.initial_probability = Eigen::Vector2d(-.1, 1.1);
          predictor::Imm f(c);
        });
    rejects([&] { filter.update(z, Eigen::Quaterniond(0, 0, 0, 0), .02); });
    rejects([&] { filter.update(z, Eigen::Quaterniond(1e308, 0, 0, 0), .02); });
    const auto normalized = filter.update(z, Eigen::Quaterniond(2, 0, 0, 0), .02);
    valid(normalized);
    std::cout << "Attitude, configurable models, and custom extension checks passed\n";
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

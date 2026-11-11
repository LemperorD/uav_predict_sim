#include "core/diagnostics.hpp"
#include "core/predictor.hpp"
#include "predictor/Pose-KF.hpp"
#include "predictor/factory.hpp"
#include "predictor/imm_predictor.hpp"

#include <iostream>
#include <limits>
#include <type_traits>

namespace
{
  void require(bool ok, const char *message)
  {
    if (!ok)
    {
      throw std::runtime_error(message);
    }
  }

  template <class Function>
  void rejects(Function function, const char *message)
  {
    try
    {
      function();
    }
    catch (const std::exception &)
    {
      return;
    }
    throw std::runtime_error(message);
  }

  // A completely different three-state, position-only algorithm with no diagnostics.
  class HoldPosition final : public predictor::Predictor
  {
   public:
    void update(const predictor::Observation &observation) override
    {
      observation.validate();
      position_ = observation.position;
    }
    predictor::PositionTrajectory predict(double step, int count) const override
    {
      validateHorizon(step, count);
      if (!position_)
      {
        throw std::logic_error("Not initialized");
      }
      return predictor::PositionTrajectory(count, *position_);
    }
    void reset() override { position_.reset(); }

   private:
    std::optional<Eigen::Vector3d> position_;
  };
}  // namespace

int main()
{
  static_assert(std::is_abstract_v<predictor::Predictor>);
  static_assert(std::has_virtual_destructor_v<predictor::Predictor>);
  static_assert(std::is_abstract_v<predictor::PoseKF>);
  static_assert(std::is_base_of_v<predictor::Predictor, predictor::PoseKF>);
  static_assert(
      std::is_same_v<decltype(std::declval<const predictor::Predictor &>().predict(.1, 2)),
                     std::vector<Eigen::Vector3d>>);
  try
  {
    std::unique_ptr<predictor::Predictor> minimal = std::make_unique<HoldPosition>();
    predictor::Observation                observation;
    observation.timestamp = 10;
    observation.position  = {1, 2, 3};
    minimal->update(observation);  // No velocity, attitude, covariance or model weights required.
    require(dynamic_cast<predictor::DiagnosticsProvider *>(minimal.get()) == nullptr,
            "Diagnostics must be optional");
    const auto held = minimal->predict(.1, 7);
    require(held.size() == 7 && held.back() == observation.position,
            "Position-only backend must work through the same interface");
    minimal->reset();
    rejects([&] { minimal->predict(.1, 1); }, "Reset must discard initialized state");

    predictor::Settings settings;
    auto                algorithm = predictor::makePredictor("imm", settings, true);
    predictor::Imm      reference(settings);
    rejects([&] { algorithm->predict(.1, 1); }, "Uninitialized prediction must fail");
    for (int i = 0; i < 30; ++i)
    {
      observation.timestamp = 100 + i * .04;
      observation.velocity  = Eigen::Vector3d(1, .3, -.1);
      observation.position  = Eigen::Vector3d(2, -1, 4) + i * .04 * *observation.velocity;
      observation.orientation =
          Eigen::Quaterniond(Eigen::AngleAxisd(.01 * i, Eigen::Vector3d::UnitZ()));
      algorithm->update(observation);
      predictor::Measurement z;
      z << observation.position, *observation.velocity;
      reference.update(z, *observation.orientation,
                       i ? observation.timestamp - (100 + (i - 1) * .04) : .01);
    }
    const auto positions = algorithm->predict(.05, 20);
    const auto expected  = reference.forecast(.05, 20);
    require(positions.size() == 20, "Prediction length must equal requested count");
    for (size_t i = 0; i < positions.size(); ++i)
    {
      require((positions[i] - expected[i].mean.head<3>()).norm() < 1e-10,
              "Adapter must preserve every IMM predicted position");
    }
    const auto repeated = algorithm->predict(.05, 20);
    require((positions.back() - repeated.back()).norm() == 0,
            "Predict must not advance online state");
    auto *diagnostics = dynamic_cast<predictor::DiagnosticsProvider *>(algorithm.get());
    require(diagnostics && diagnostics->diagnostics().forecast_endpoint_covariance.has_value(),
            "IMM diagnostics must remain available outside the base output");
    const auto before = diagnostics->diagnostics().position;
    rejects([&] { algorithm->update(observation); }, "Duplicate timestamps must fail");
    observation.timestamp -= 1;
    rejects([&] { algorithm->update(observation); }, "Reversed timestamps must fail");
    observation.timestamp += 2;
    observation.position.x() = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { algorithm->update(observation); }, "Nonfinite positions must fail");
    require((diagnostics->diagnostics().position - before).norm() == 0,
            "Rejected observations must not alter the filter");
    rejects([&] { algorithm->predict(0, 1); }, "Zero step must fail");
    rejects([&] { algorithm->predict(.1, 0); }, "Zero count must fail");
    rejects([&] { algorithm->predict(.1, 10001); }, "Excessive horizon must fail");
    algorithm->reset();
    rejects([&] { algorithm->predict(.1, 1); }, "Reset must require new observations");
    require(!diagnostics->diagnostics().position.allFinite(), "Reset must clear stale diagnostics");
    rejects([] { predictor::makePredictor("unimplemented"); }, "Unknown algorithms must fail");
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}

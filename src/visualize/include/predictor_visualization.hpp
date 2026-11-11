#pragma once

#include <stdexcept>
#include <string>
#include <vector>

struct DiagnosticCurve
{
  std::string label;
  size_t      metric_index;
  bool        square = false;
};
struct DiagnosticPanel
{
  std::string                  title;
  double                       maximum;
  std::vector<DiagnosticCurve> curves;
};

// A visualization backend declares panels without owning the 3-D scene or ROS
// subscriptions. New predictors extend this interface and register in the factory.
class PredictorVisualization
{
 public:
  virtual ~PredictorVisualization()                                      = default;
  virtual std::vector<DiagnosticPanel> panels(double error_max, double covariance_max,
                                              double velocity_max) const = 0;

 protected:
  std::vector<DiagnosticPanel> common(double error_max, double covariance_max) const
  {
    return {{"Position and forecast error",
             error_max,
             {{"position error (m)", 15}, {"horizon error (m)", 16}}},
            {"Position covariance",
             covariance_max,
             {{"Pxx (m2)", 10, true}, {"Pyy (m2)", 11, true}, {"Pzz (m2)", 12, true}}}};
  }
};
class ImmVisualization final : public PredictorVisualization
{
 public:
  explicit ImmVisualization(std::vector<std::string> models) : models_(std::move(models)) {}
  std::vector<DiagnosticPanel> panels(double e, double c, double) const override
  {
    auto            result = common(e, c);
    DiagnosticPanel probability{"IMM model probabilities", 1, {}};
    size_t          extra = 19;
    for (const auto &name : models_)
      probability.curves.push_back({"P(" + name + ")", name == "cv"   ? 13
                                                       : name == "ca" ? 14
                                                                      : extra++});
    result.push_back(probability);
    return result;
  }

 private:
  std::vector<std::string> models_;
};
class PoseKFVisualization final : public PredictorVisualization
{
 public:
  std::vector<DiagnosticPanel> panels(double e, double c, double v) const override
  {
    auto result = common(e, c);
    result.push_back({"Velocity estimation error", v, {{"velocity error (m/s)", 17}}});
    return result;
  }
};

#pragma once

#include "core/diagnostics.hpp"
#include "core/predictor.hpp"
#include "predictor/imm.hpp"

namespace predictor
{
  // Adapter: the IMM engine's 15-dimensional state never crosses Predictor::predict().
  class ImmPredictor final : public Predictor, public DiagnosticsProvider
  {
   public:
    explicit ImmPredictor(Settings settings = {}, bool use_attitude = true);
    void               update(const Observation &observation) override;
    PositionTrajectory predict(double step, int count) const override;
    void               reset() override;
    Diagnostics        diagnostics() const override;

   private:
    Settings              settings_;
    bool                  use_attitude_;
    Imm                   filter_;
    std::optional<double> last_timestamp_;
    mutable Diagnostics   diagnostics_;
  };
}  // namespace predictor

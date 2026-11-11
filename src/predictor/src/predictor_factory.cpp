#include "predictor/factory.hpp"
#include "predictor/imm_predictor.hpp"

namespace predictor
{
  std::unique_ptr<Predictor> makePredictor(const std::string &algorithm,
                                           const Settings &imm_settings, bool use_attitude)
  {
    if (algorithm == "imm")
    {
      return std::make_unique<ImmPredictor>(imm_settings, use_attitude);
    }
    throw std::invalid_argument("Unknown prediction_algorithm: " + algorithm);
  }
}  // namespace predictor

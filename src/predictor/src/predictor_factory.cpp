#include "predictor/factory.hpp"
#include "predictor/imm_predictor.hpp"

namespace predictor
{
  std::unique_ptr<Predictor> makePredictor(const std::string &algorithm,
                                           const Settings &imm_settings, bool use_attitude,
                                           const PoseKFSettings &pose_settings)
  {
    if (algorithm == "imm")
    {
      return std::make_unique<ImmPredictor>(imm_settings, use_attitude);
    }
    if (algorithm == "pose_kf" || algorithm == "kf")
    {
      if (algorithm == "pose_kf" && !use_attitude)
        throw std::invalid_argument("pose_kf requires use_attitude: true; use kf for the baseline");
      return std::make_unique<PoseKF>(pose_settings, algorithm == "pose_kf");
    }
    throw std::invalid_argument("Unknown prediction_algorithm: " + algorithm);
  }
}  // namespace predictor

// Replay one immutable observation CSV through all hypotheses. Future truth is
// accessed only after update/forecast, exclusively for scoring.
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "predictor/imm.hpp"

struct Sample
{
  double                 t;
  predictor::Measurement truth, observation;
  Eigen::Quaterniond     orientation;
};
int main(int argc, char **argv)
{
  try
  {
    if (argc != 5)
      throw std::runtime_error(
          "usage: imm_pose_replay observations.csv scores.csv velocity_std attitude_std");
    std::ifstream input(argv[1]);
    if (!input) throw std::runtime_error("Cannot open observation CSV");
    std::string line;
    std::getline(input, line);
    if (line != "t,px,py,pz,vx,vy,vz,mx,my,mz,mvx,mvy,mvz,qw,qx,qy,qz")
      throw std::runtime_error("Unexpected observation schema");
    std::vector<Sample> samples;
    while (std::getline(input, line))
    {
      std::stringstream   row(line);
      std::string         cell;
      std::vector<double> v;
      while (std::getline(row, cell, ',')) v.push_back(std::stod(cell));
      if (v.size() != 17) throw std::runtime_error("Bad observation row");
      Sample sample;
      sample.t = v[0];
      for (int k = 0; k < 6; ++k)
      {
        sample.truth(k)       = v[k + 1];
        sample.observation(k) = v[k + 7];
      }
      sample.orientation = Eigen::Quaterniond(v[13], v[14], v[15], v[16]);
      if (!std::isfinite(sample.t) || !sample.truth.allFinite() ||
          !sample.observation.allFinite() || !sample.orientation.coeffs().allFinite() ||
          sample.orientation.norm() < 1e-12 ||
          (!samples.empty() && std::abs(sample.t - samples.back().t - .02) > 1e-8))
        throw std::runtime_error("Replay requires finite observations at exactly 50 Hz");
      samples.push_back(sample);
    }
    if (samples.size() < 251) throw std::runtime_error("Replay requires at least 5 seconds");
    std::ofstream output(argv[2]);
    if (!output) throw std::runtime_error("Cannot open score CSV");
    output << std::setprecision(15);
    output << "t,algorithm,position_sq,velocity_sq,trajectory_sq,fde_05_sq,fde_1_sq,fde_2_sq,p_"
              "pose_kf,"
              "update_us,forecast_us\n";
    const std::vector<std::pair<std::string, std::vector<std::string>>> variants = {
        {"IMM_CV_CA", {"cv", "ca"}},
        {"IMM_CV_CA_POSE", {"cv", "ca", "pose_kf"}},
        {"IMM_CV_POSE", {"cv", "pose_kf"}},
        {"POSE_MODEL_PV", {"pose_kf"}}};
    for (const auto &variant : variants)
    {
      predictor::Settings settings;
      settings.models       = variant.second;
      settings.velocity_std = std::stod(argv[3]);
      settings.attitude_std = std::stod(argv[4]);
      // Same continuous acceleration density as CV; no per-scene tuning.
      settings.pose_accel_density = settings.cv_accel_density;
      predictor::Imm filter(settings);
      for (size_t i = 0; i < samples.size(); ++i)
      {
        const auto  &sample   = samples[i];
        const auto   begin    = std::chrono::steady_clock::now();
        const auto   estimate = filter.update(sample.observation, sample.orientation, .02);
        const double update_us =
            std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - begin)
                .count();
        if (sample.t < 3 || i % 10 || i + 100 >= samples.size()) continue;
        const auto   forecast_begin = std::chrono::steady_clock::now();
        const auto   future         = filter.forecast(.1, 20);
        const double forecast_us    = std::chrono::duration<double, std::micro>(
                                       std::chrono::steady_clock::now() - forecast_begin)
                                       .count();
        double                trajectory_sq = 0;
        std::array<double, 3> fde{};
        for (size_t k = 0; k < future.size(); ++k)
        {
          const double error =
              (future[k].mean.head<3>() - samples[i + (k + 1) * 5].truth.head<3>()).squaredNorm();
          trajectory_sq += error / future.size();
          if (k == 4) fde[0] = error;
          if (k == 9) fde[1] = error;
          if (k == 19) fde[2] = error;
        }
        double pose_probability = 0;
        for (size_t k = 0; k < settings.models.size(); ++k)
          if (settings.models[k] == "pose_kf") pose_probability = estimate.probability(k);
        output << sample.t << ',' << variant.first << ','
               << (estimate.mean.head<3>() - sample.truth.head<3>()).squaredNorm() << ','
               << (estimate.mean.segment<3>(3) - sample.truth.tail<3>()).squaredNorm() << ','
               << trajectory_sq << ',' << fde[0] << ',' << fde[1] << ',' << fde[2] << ','
               << pose_probability << ',' << update_us << ',' << forecast_us << '\n';
      }
    }
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

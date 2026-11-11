// Production PoseKF replay for diagnostics. No truth or w is supplied to the filter.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "predictor/Pose-KF.hpp"

int main(int argc, char **argv)
{
  try
  {
    if (argc != 4)
      throw std::runtime_error("usage: pose_kf_replay observations.csv output.csv gravity");
    std::ifstream input(argv[1]);
    std::ofstream output(argv[2]);
    if (!input || !output) throw std::runtime_error("Cannot open replay input/output");
    std::string line;
    std::getline(input, line);
    if (line != "t,mx,my,mz,qw,qx,qy,qz") throw std::runtime_error("Unexpected replay schema");
    predictor::PoseKFSettings settings;
    settings.gravity = std::stod(argv[3]);
    predictor::PoseKF filter(settings);
    output << std::setprecision(15)
           << "t,estimate_px,estimate_py,estimate_pz,estimate_vx,estimate_vy,estimate_vz,"
              "forecast_1s_x,forecast_1s_y,forecast_1s_z,std_x,std_y,std_z\n";
    size_t count = 0;
    while (std::getline(input, line))
    {
      std::stringstream   row(line);
      std::string         cell;
      std::vector<double> values;
      while (std::getline(row, cell, ',')) values.push_back(std::stod(cell));
      if (values.size() != 8) throw std::runtime_error("Invalid replay row");
      predictor::Observation observation;
      observation.timestamp   = values[0];
      observation.position    = Eigen::Vector3d(values[1], values[2], values[3]);
      observation.orientation = Eigen::Quaterniond(values[4], values[5], values[6], values[7]);
      filter.update(observation);
      const auto forecast = filter.predict(.05, 20);
      const auto d        = filter.diagnostics();
      output << observation.timestamp;
      for (double value : d.position) output << ',' << value;
      for (double value : d.velocity) output << ',' << value;
      for (double value : forecast.back()) output << ',' << value;
      for (int k = 0; k < 3; ++k) output << ',' << std::sqrt((*d.pose_covariance)(k, k));
      output << '\n';
      ++count;
    }
    if (!count) throw std::runtime_error("Empty replay");
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

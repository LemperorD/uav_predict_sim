#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main(int argc, char **argv)
{
  try
  {
    if (argc != 3)
    {
      throw std::runtime_error("usage: prediction_plot samples.csv output.png");
    }
    std::ifstream in(argv[1]);
    if (!in)
    {
      throw std::runtime_error("cannot open samples CSV");
    }
    std::string line;
    std::getline(in, line);
    std::vector<std::string> names;
    std::stringstream        header(line);
    std::string              name;
    while (std::getline(header, name, ','))
    {
      names.push_back(name);
    }
    if (names.size() < 19)
    {
      throw std::runtime_error("invalid sample header");
    }
    std::vector<std::vector<double>> columns(names.size());
    while (std::getline(in, line))
    {
      std::stringstream   s(line);
      std::string         cell;
      std::vector<double> row(names.size());
      for (size_t i = 0; i < names.size(); ++i)
      {
        if (!std::getline(s, cell, ','))
        {
          throw std::runtime_error("invalid sample row");
        }
        row[i] = std::stod(cell);
      }
      for (size_t i = 0; i < names.size(); ++i)
      {
        columns[i].push_back(row[i]);
      }
    }
    if (columns[0].empty())
    {
      throw std::runtime_error("no samples to plot");
    }
    auto &t = columns[0];
    plt::backend("Agg");
    plt::figure_size(1600, 1100);
    plt::subplot(2, 2, 1);
    plt::title("Trajectory (XY projection)");
    plt::named_plot("truth", columns[1], columns[2], "g-");
    plt::named_plot("observation", columns[4], columns[5], "k.");
    plt::named_plot("IMM", columns[7], columns[8], "b-");
    plt::xlabel("x (m)");
    plt::ylabel("y (m)");
    plt::legend();
    plt::grid(true);
    plt::subplot(2, 2, 2);
    plt::title("Position and time-aligned horizon error");
    plt::named_plot("position", t, columns[15]);
    plt::named_plot("horizon", t, columns[16]);
    plt::xlabel("time (s)");
    plt::ylabel("error (m)");
    plt::legend();
    plt::grid(true);
    plt::subplot(2, 2, 3);
    plt::title("Position covariance diagonal");
    for (int k = 0; k < 3; ++k)
    {
      auto variance = columns[10 + k];
      for (auto &v : variance)
      {
        v *= v;
      }
      plt::named_plot("P" + std::string(2, "xyz"[k]), t, variance);
    }
    plt::xlabel("time (s)");
    plt::ylabel("variance (m^2)");
    plt::legend();
    plt::grid(true);
    plt::subplot(2, 2, 4);
    plt::title("IMM model probabilities");
    for (size_t i = 0; i < names.size(); ++i)
    {
      if (names[i].rfind("p_", 0) == 0)
      {
        plt::named_plot(names[i].substr(2), t, columns[i]);
      }
    }
    plt::ylim(0, 1);
    plt::xlabel("time (s)");
    plt::legend();
    plt::grid(true);
    plt::tight_layout();
    auto parent = std::filesystem::path(argv[2]).parent_path();
    if (!parent.empty())
    {
      std::filesystem::create_directories(parent);
    }
    plt::save(argv[2]);
    plt::close();
  }
  catch (const std::exception &e)
  {
    if (PyErr_Occurred())
    {
      PyErr_Print();
    }
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}

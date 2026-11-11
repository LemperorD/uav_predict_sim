#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

struct Run
{
  std::string                                label;
  std::vector<std::string>                   names;
  std::map<std::string, std::vector<double>> columns;
  Run(const std::string &path, std::string name) : label(std::move(name))
  {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open samples CSV: " + path);
    std::string line, cell;
    std::getline(in, line);
    std::stringstream header(line);
    while (std::getline(header, cell, ','))
    {
      if (!cell.empty() && cell.back() == '\r') cell.pop_back();
      if (columns.count(cell)) throw std::runtime_error("duplicate CSV column: " + cell);
      names.push_back(cell);
      columns[cell] = {};
    }
    while (std::getline(in, line))
    {
      std::stringstream row(line);
      for (const auto &column : names)
      {
        if (!std::getline(row, cell, ','))
          throw std::runtime_error("invalid sample row in " + path);
        size_t       end   = 0;
        const double value = std::stod(cell, &end);
        if (std::isinf(value)) throw std::runtime_error("infinite CSV value");
        if (end != cell.size() && cell.substr(end) != "\r")
          throw std::runtime_error("invalid numeric CSV cell");
        columns[column].push_back(value);
      }
      if (std::getline(row, cell, ',')) throw std::runtime_error("extra sample columns");
    }
    if (get("t").empty()) throw std::runtime_error("no samples to plot");
    for (size_t i = 0; i < get("t").size(); ++i)
      if (!std::isfinite(get("t")[i]) || (i && get("t")[i] <= get("t")[i - 1]))
        throw std::runtime_error("CSV timestamps must be finite and strictly increasing");
    for (const auto &required :
         {"truth_x", "truth_y", "measurement_x", "measurement_y", "estimate_x", "estimate_y",
          "position_error", "horizon_error", "velocity_error", "std_x", "std_y", "std_z"})
      get(required);
  }
  const std::vector<double> &get(const std::string &name) const
  {
    const auto found = columns.find(name);
    if (found == columns.end()) throw std::runtime_error("missing CSV column: " + name);
    return found->second;
  }
};

int main(int argc, char **argv)
{
  try
  {
    if (argc < 3)
      throw std::runtime_error(
          "usage: prediction_plot samples.csv output.png [--profile imm|pose_kf|kf] "
          "[--label name] [--baseline name samples.csv]...");
    std::string                                      profile = "imm", label = "current";
    std::vector<std::pair<std::string, std::string>> baselines;
    for (int i = 3; i < argc; ++i)
    {
      const std::string option = argv[i];
      if ((option == "--profile" || option == "--label") && i + 1 < argc)
      {
        (option == "--profile" ? profile : label) = argv[++i];
      }
      else if (option == "--baseline" && i + 2 < argc)
      {
        const std::string name = argv[++i];
        baselines.emplace_back(name, argv[++i]);
      }
      else
        throw std::runtime_error("invalid plot option: " + option);
    }
    if (profile != "imm" && profile != "pose_kf" && profile != "kf")
      throw std::runtime_error("unknown plot profile: " + profile);
    std::vector<Run> runs;
    runs.emplace_back(argv[1], label);
    for (const auto &baseline : baselines) runs.emplace_back(baseline.second, baseline.first);
    const auto &run = runs.front();
    const auto &t   = run.get("t");
    plt::backend("Agg");
    plt::figure_size(1600, 1100);
    plt::subplot(2, 2, 1);
    plt::title("Trajectory (XY projection)");
    plt::named_plot("truth", run.get("truth_x"), run.get("truth_y"), "k--");
    plt::named_plot("observation", run.get("measurement_x"), run.get("measurement_y"), "k.");
    for (const auto &r : runs) plt::named_plot(r.label, r.get("estimate_x"), r.get("estimate_y"));
    plt::xlabel("x (m)");
    plt::ylabel("y (m)");
    plt::legend();
    plt::grid(true);
    plt::subplot(2, 2, 2);
    plt::title("Position and time-aligned horizon error");
    for (const auto &r : runs)
    {
      plt::named_plot(r.label + " position", r.get("t"), r.get("position_error"));
      plt::named_plot(r.label + " horizon", r.get("t"), r.get("horizon_error"), "--");
    }
    plt::xlabel("time (s)");
    plt::ylabel("error (m)");
    plt::legend();
    plt::grid(true);
    plt::subplot(2, 2, 3);
    plt::title(label + " position covariance diagonal");
    for (const auto &axis : {"x", "y", "z"})
    {
      auto variance = run.get(std::string("std_") + axis);
      for (auto &v : variance) v *= v;
      plt::named_plot(std::string("P") + axis + axis, t, variance);
    }
    plt::xlabel("time (s)");
    plt::ylabel("variance (m^2)");
    plt::legend();
    plt::grid(true);
    plt::subplot(2, 2, 4);
    if (runs.size() > 1 || profile != "imm")
    {
      plt::title("Velocity estimation error");
      for (const auto &r : runs) plt::named_plot(r.label, r.get("t"), r.get("velocity_error"));
      plt::ylabel("error (m/s)");
      plt::legend();
    }
    else
    {
      plt::title("Model probabilities (optional diagnostics)");
      bool available = false;
      for (const auto &name : run.names)
        if (name.rfind("p_", 0) == 0)
        {
          const auto &values = run.get(name);
          bool        finite = false;
          for (double value : values) finite = finite || std::isfinite(value);
          if (finite)
          {
            plt::named_plot(name.substr(2), t, values);
            available = true;
          }
        }
      plt::ylim(0, 1);
      if (available) plt::legend();
    }
    plt::xlabel("time (s)");
    plt::grid(true);
    plt::tight_layout();
    auto parent = std::filesystem::path(argv[2]).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    plt::save(argv[2]);
    plt::close();
  }
  catch (const std::exception &e)
  {
    if (Py_IsInitialized() && PyErr_Occurred()) PyErr_Print();
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}

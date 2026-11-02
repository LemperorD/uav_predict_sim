// SPDX-License-Identifier: GPL-3.0-only
#include "uav_predict_sim/simulation/experiment.hpp"
#include <iostream>
#include <stdexcept>

int main(int argc,char** argv) {
  try {
    uav::Config config;
    std::filesystem::path output="outputs/helix";
    bool plotting=false,viewing=false;
    for(int i=1;i<argc;++i) {
      const std::string option=argv[i];
      if(option=="--help") {
        std::cout<<"Usage: uav_sim [--config FILE] [--scene hover|vertical|helix|eight]\n"
                 <<"               [--output NEW_DIRECTORY] [--plot] [--view]\n"
                 <<"Options apply in order; default scene helix, duration 12s.\n";
        return 0;
      }
      if(option=="--plot") { plotting=true; continue; }
      if(option=="--view") { viewing=true; continue; }
      if(i+1>=argc) throw std::invalid_argument("missing value for "+option);
      const std::string value=argv[++i];
      if(option=="--config") config=uav::load_config(value);
      else if(option=="--scene") config.trajectory.scene=value;
      else if(option=="--output") output=value;
      else throw std::invalid_argument("unknown option: "+option);
    }
#ifndef UAV_ENABLE_PLOTTING
    if(plotting) throw std::runtime_error("rebuild with -DUAV_ENABLE_PLOTTING=ON");
#endif
#ifndef UAV_ENABLE_VIEWER
    if(viewing) throw std::runtime_error("rebuild with -DUAV_ENABLE_VIEWER=ON");
#endif
    if(std::filesystem::exists(output)) throw std::runtime_error("output directory already exists");
    const auto experiment=uav::run(config);
    uav::save(config,experiment,output);
    uav::print_summary(experiment);
    std::cout<<"Saved "<<std::filesystem::absolute(output)<<'\n';
#ifdef UAV_ENABLE_PLOTTING
    if(plotting) uav::plot(experiment,output);
#endif
#ifdef UAV_ENABLE_VIEWER
    if(viewing) uav::view(experiment,config.vehicle);
#endif
    return 0;
  } catch(const std::exception& e) { std::cerr<<"Error: "<<e.what()<<'\n'; return 1; }
}

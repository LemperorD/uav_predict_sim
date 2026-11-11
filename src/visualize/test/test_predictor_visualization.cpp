#include <stdexcept>
#include "predictor_visualization.hpp"
int main()
{
  ImmVisualization imm({"body_velocity", "ca", "cv"});
  const auto       panels = imm.panels(2, 3, 4);
  if (panels.size() != 3 || panels[2].curves[0].metric_index != 19 ||
      panels[2].curves[1].metric_index != 14 || panels[2].curves[2].metric_index != 13)
    throw std::runtime_error(
        "IMM profile must preserve metrics wire indices with reordered models");
  PoseKFVisualization pose;
  const auto          p = pose.panels(2, 3, 4);
  if (p[2].curves.size() != 1 || p[2].curves[0].metric_index != 17 || p[2].maximum != 4 ||
      !p[1].curves[0].square || p[1].curves[0].metric_index != 10)
    throw std::runtime_error("Pose-KF must show velocity error and square standard deviations");
}

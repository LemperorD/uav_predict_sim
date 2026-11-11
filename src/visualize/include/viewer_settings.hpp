#pragma once

#include <rclcpp/rclcpp.hpp>
#include "viewer_theme.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

// Read and validate display settings before opening a window or creating recording files.
struct ViewerSettings : ViewerTheme
{
  explicit ViewerSettings(rclcpp::Node &node)
      : ViewerTheme(makeViewerTheme(node.declare_parameter<std::string>("theme", "light")))
  {
    auto vector = [&](const std::string &name, std::vector<double> value)
    {
      const auto result = node.declare_parameter(name, value);
      if (result.size() != value.size())
      {
        throw std::invalid_argument(name + " has an invalid number of components");
      }
      for (double component : result)
      {
        if (!std::isfinite(component))
        {
          throw std::invalid_argument(name + " must contain finite values");
        }
      }
      return result;
    };
    auto positive = [&](const std::string &name, double value)
    {
      const double result = node.declare_parameter(name, value);
      if (!std::isfinite(result) || result <= 0)
      {
        throw std::invalid_argument(name + " must be finite and positive");
      }
      return result;
    };
    drone_scale       = positive("drone_scale", 2.);
    drone_axis_length = positive("drone_axis_length", .45);
    show_drone        = node.declare_parameter("show_drone", true);
    show_drone_axes   = node.declare_parameter("show_drone_axes", true);
    drone_pose_source = node.declare_parameter<std::string>("drone_pose_source", "truth");
    if (drone_pose_source != "truth" && drone_pose_source != "estimate")
    {
      throw std::invalid_argument("drone_pose_source must be truth or estimate");
    }
    eye             = vector("camera_eye", {8., -10., 8.});
    target          = vector("camera_target", {0., 0., 2.});
    const double dx = eye[0] - target[0], dy = eye[1] - target[1];
    if (std::hypot(dx, dy) < 1e-6)
    {
      throw std::invalid_argument("camera direction must not be parallel to the Z up axis");
    }
    focal_length = positive("camera_focal_length", 850.);
    near_clip    = positive("camera_near_clip", .1);
    far_clip     = positive("camera_far_clip", 1000.);
    if (far_clip <= near_clip)
    {
      throw std::invalid_argument("camera_far_clip must exceed camera_near_clip");
    }
    grid_extent  = positive("grid_extent", 10.);
    grid_spacing = positive("grid_spacing", 1.);
    if (grid_extent / grid_spacing > 1000)
    {
      throw std::invalid_argument("grid_extent / grid_spacing must not exceed 1000");
    }
    line_width       = positive("trajectory_line_width", 2.);
    observation_size = positive("observation_point_size", 2.);
    estimate_size    = positive("estimate_point_size", 9.);
    error_max        = positive("error_plot_max", 1.);
    covariance_max   = positive("covariance_plot_max", .001);
    max_track_points = node.declare_parameter("max_track_points", 20000);
    if (max_track_points <= 0)
    {
      throw std::invalid_argument("max_track_points must be positive");
    }
    show_grid        = node.declare_parameter("show_grid", true);
    show_legend      = node.declare_parameter("show_legend", true);
    show_uncertainty = node.declare_parameter("show_uncertainty", true);
  }

  double              drone_scale, drone_axis_length;
  bool                show_drone, show_drone_axes;
  std::string         drone_pose_source;
  std::vector<double> eye, target;
  double              focal_length, near_clip, far_clip, grid_extent, grid_spacing;
  double              line_width, observation_size, estimate_size, error_max, covariance_max;
  int                 max_track_points;
  bool                show_grid, show_legend, show_uncertainty;
};

#pragma once

#include <pangolin/gl/colour.h>

#include <array>
#include <stdexcept>
#include <string>

// Complete palettes keep text, plots and the quadrotor readable in either theme.
struct ViewerTheme
{
  pangolin::Colour background{.04, .055, .08}, plot_background{.06, .08, .12};
  pangolin::Colour plot_axis{.5, .5, .5}, plot_ticks{.3, .3, .3}, text{.9, .94, 1.};
  pangolin::Colour grid{.2, .25, .3}, truth{.2, .9, .4}, observation{.55, .55, .6};
  pangolin::Colour estimate{.2, .65, 1.}, forecast{1., .6, .1}, uncertainty{.3, .7, 1.};
  pangolin::Colour position_error{.2, .7, 1.}, horizon_error{1., .6, .1};
  pangolin::Colour drone_body{.75, .8, .85}, drone_front{1., .3, .05};
  std::array<pangolin::Colour, 3> covariance = {
      pangolin::Colour(.9, .3, .9), pangolin::Colour(.7, .9, .2), pangolin::Colour(.2, .65, 1.)};
  std::array<pangolin::Colour, 3> models = covariance;
};

inline ViewerTheme makeViewerTheme(const std::string &name)
{
  ViewerTheme palette;
  if (name == "light")
  {
    palette.background      = pangolin::Colour(1., 1., 1.);
    palette.plot_background = pangolin::Colour(.96, .97, .98);
    palette.text            = pangolin::Colour(.08, .1, .14);
    palette.plot_axis       = pangolin::Colour(.3, .33, .38);
    palette.plot_ticks      = pangolin::Colour(.75, .78, .82);
    palette.grid            = pangolin::Colour(.78, .8, .83);
    palette.truth           = pangolin::Colour(0., .48, .2);
    palette.observation     = pangolin::Colour(.4, .43, .47);
    palette.estimate        = pangolin::Colour(0., .36, .78);
    palette.forecast        = pangolin::Colour(.78, .31, 0.);
    palette.uncertainty     = pangolin::Colour(.15, .45, .8);
    palette.position_error  = palette.estimate;
    palette.horizon_error   = palette.forecast;
    palette.drone_body      = pangolin::Colour(.3, .36, .42);
    palette.drone_front     = pangolin::Colour(.85, .25, 0.);
    palette.covariance      = {pangolin::Colour(.67, .15, .67), pangolin::Colour(.38, .48, .02),
                               palette.estimate};
    palette.models          = palette.covariance;
  }
  else if (name != "dark")
  {
    throw std::invalid_argument("theme must be light or dark");
  }
  return palette;
}

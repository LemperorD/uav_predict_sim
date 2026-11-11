#include "viewer_theme.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

float luminance(const pangolin::Colour &color)
{
  auto linear = [](float value)
  { return value <= .04045f ? value / 12.92f : std::pow((value + .055f) / 1.055f, 2.4f); };
  return .2126f * linear(color.r) + .7152f * linear(color.g) + .0722f * linear(color.b);
}

float contrast(const pangolin::Colour &a, const pangolin::Colour &b)
{
  const float x = luminance(a), y = luminance(b);
  return (std::max(x, y) + .05f) / (std::min(x, y) + .05f);
}

int main()
{
  for (const auto *name : {"light", "dark"})
  {
    const auto palette = makeViewerTheme(name);
    if (contrast(palette.text, palette.background) < 7)
    {
      std::cerr << name << ": dashboard text must remain readable\n";
      return 1;
    }
    for (const auto &series : {palette.truth, palette.observation, palette.estimate,
                               palette.forecast, palette.position_error, palette.horizon_error})
    {
      if (contrast(series, palette.background) < 3 || contrast(series, palette.plot_background) < 3)
      {
        std::cerr << name << ": trajectory/plot series must contrast with the background\n";
        return 1;
      }
    }
  }
  try
  {
    makeViewerTheme("invalid");
    std::cerr << "Invalid theme must be rejected\n";
    return 1;
  }
  catch (const std::invalid_argument &)
  {
  }
  return 0;
}

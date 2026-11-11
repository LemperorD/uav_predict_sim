#include "drone_pose.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition, const char *message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

int main()
{
  try
  {
    DronePose state;
    require(!state.valid, "Model must stay hidden before a valid pose");
    geometry_msgs::msg::Pose pose;
    pose.position.x = 3;
    pose.position.y = -2;
    pose.position.z = 4;
    // A non-unit quaternion representing 90 degrees yaw: body +X becomes world +Y.
    pose.orientation.w = 2;
    pose.orientation.z = 2;
    require(state.update(pose), "Non-unit finite quaternion must be normalized");
    require((state.transform * Eigen::Vector4d(1, 0, 0, 1) - Eigen::Vector4d(3, -1, 4, 1)).norm() <
                1e-12,
            "Yaw or world translation is incorrect");
    const auto yaw     = state.transform;
    pose.orientation.w = -2;
    pose.orientation.z = -2;
    require(state.update(pose) && (state.transform - yaw).norm() < 1e-12,
            "q and -q must render identically");
    pose.orientation.w = 1;
    pose.orientation.x = 1;
    pose.orientation.z = 0;
    require(state.update(pose), "Roll must be accepted");
    require(
        (state.transform.block<3, 3>(0, 0) * Eigen::Vector3d::UnitY() - Eigen::Vector3d::UnitZ())
                .norm() < 1e-12,
        "Roll must rotate body +Y to world +Z");
    const auto previous = state.transform;
    pose.orientation.w  = 0;
    pose.orientation.x  = 0;
    require(!state.update(pose), "Zero quaternion must be rejected");
    pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
    require(!state.update(pose), "Nonfinite quaternion must be rejected");
    pose.orientation.w = 1;
    pose.position.x    = std::numeric_limits<double>::infinity();
    require(!state.update(pose), "Nonfinite position must be rejected");
    require((state.transform - previous).norm() == 0,
            "Rejected input must preserve last valid pose");
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}

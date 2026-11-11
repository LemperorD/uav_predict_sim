#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>

struct DronePose
{
  bool update(const geometry_msgs::msg::Pose &pose)
  {
    const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond    q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                            pose.orientation.z);
    const double          norm = q.norm();
    if (!position.allFinite() || !q.coeffs().allFinite() || !std::isfinite(norm) || norm < 1e-12)
    {
      return false;
    }
    q.normalize();
    transform.setIdentity();
    transform.block<3, 3>(0, 0) = q.toRotationMatrix();
    transform.block<3, 1>(0, 3) = position;
    valid                       = true;
    return true;
  }

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  bool            valid     = false;
};

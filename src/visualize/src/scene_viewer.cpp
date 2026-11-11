#include <pangolin/pangolin.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{
  using Point = std::array<double, 3>;

  struct Track
  {
    std::deque<Point> points;
  };

  void drawPoints(const std::vector<Point> &points)
  {
    glBegin(GL_POINTS);
    for (const auto &point : points)
    {
      glVertex3dv(point.data());
    }
    glEnd();
  }
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto       node            = std::make_shared<rclcpp::Node>("scene_viewer");
    const auto odometry_topics = node->declare_parameter<std::vector<std::string>>(
        "odometry_topics", {"/drone_0_visual_slam/odom"});
    const auto cloud_topics = node->declare_parameter<std::vector<std::string>>(
        "cloud_topics", {"/map_generator/global_cloud"});
    const int max_track_points = node->declare_parameter("max_track_points", 20000);
    const int max_cloud_points = node->declare_parameter("max_cloud_points", 200000);
    const int fps              = node->declare_parameter("fps", 30);
    if (max_track_points <= 0 || max_cloud_points <= 0 || fps <= 0 || fps > 120)
    {
      throw std::invalid_argument("point limits must be positive and fps must be in [1, 120]");
    }

    // All callbacks and drawing run on the window thread. Topics must share world coordinates.
    std::vector<Track>              tracks(odometry_topics.size());
    std::vector<std::vector<Point>> clouds(cloud_topics.size());
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>       odometry_subs;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subs;
    for (size_t i = 0; i < odometry_topics.size(); ++i)
    {
      if (odometry_topics[i].empty())
      {
        continue;
      }
      odometry_subs.push_back(node->create_subscription<nav_msgs::msg::Odometry>(
          odometry_topics[i], rclcpp::SensorDataQoS(),
          [&, i](const nav_msgs::msg::Odometry &message)
          {
            const auto &p = message.pose.pose.position;
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            {
              return;
            }
            auto &points = tracks[i].points;
            points.push_back({p.x, p.y, p.z});
            if (points.size() > static_cast<size_t>(max_track_points))
            {
              points.pop_front();
            }
          }));
    }
    for (size_t i = 0; i < cloud_topics.size(); ++i)
    {
      if (cloud_topics[i].empty())
      {
        continue;
      }
      cloud_subs.push_back(node->create_subscription<sensor_msgs::msg::PointCloud2>(
          cloud_topics[i], rclcpp::SensorDataQoS(),
          [&, i](const sensor_msgs::msg::PointCloud2 &message)
          {
            // PCL map publishers use FLOAT32 xyz fields. Reject incompatible layouts.
            for (const auto *name : {"x", "y", "z"})
            {
              bool valid = false;
              for (const auto &field : message.fields)
              {
                valid |= field.name == name &&
                         field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                         field.count == 1 && field.offset + sizeof(float) <= message.point_step;
              }
              if (!valid)
              {
                RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                                     "Point cloud requires FLOAT32 xyz fields");
                return;
              }
            }
            if (message.is_bigendian || message.point_step == 0 ||
                message.row_step != static_cast<size_t>(message.width) * message.point_step ||
                message.data.size() != static_cast<size_t>(message.row_step) * message.height)
            {
              return;
            }
            if (message.data.empty())
            {
              clouds[i].clear();
              return;
            }
            std::vector<Point> points;
            const size_t       count = static_cast<size_t>(message.width) * message.height;
            const size_t       stride =
                std::max(size_t{1}, (count + max_cloud_points - 1) / max_cloud_points);
            sensor_msgs::PointCloud2ConstIterator<float> x(message, "x"), y(message, "y"),
                z(message, "z");
            for (size_t j = 0; j < count; ++j, ++x, ++y, ++z)
            {
              if (j % stride == 0 && std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z))
              {
                points.push_back({*x, *y, *z});
              }
            }
            clouds[i] = std::move(points);
          }));
    }

    pangolin::CreateWindowAndBind("UAV scene | world / meters", 1280, 800);
    pangolin::OpenGlRenderState camera(
        pangolin::ProjectionMatrix(1280, 800, 700, 700, 640, 400, .1, 1000),
        pangolin::ModelViewLookAt(22, -30, 25, 0, 0, 1, pangolin::AxisZ));
    auto &view =
        pangolin::CreateDisplay().SetBounds(0, 1, 0, 1).SetHandler(new pangolin::Handler3D(camera));
    constexpr std::array<std::array<float, 3>, 6> colors = {{{.2f, .9f, .4f},
                                                             {.2f, .65f, 1.f},
                                                             {1.f, .6f, .1f},
                                                             {.9f, .3f, .5f},
                                                             {.7f, .5f, 1.f},
                                                             {.2f, .9f, .9f}}};
    while (rclcpp::ok() && !pangolin::ShouldQuit())
    {
      const auto next = std::chrono::steady_clock::now() + std::chrono::microseconds(1000000 / fps);
      rclcpp::spin_some(node);
      glClearColor(.04, .055, .08, 1);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glEnable(GL_DEPTH_TEST);
      view.Activate(camera);
      glColor3f(.2, .25, .3);
      glLineWidth(1);
      glBegin(GL_LINES);
      for (int i = -50; i <= 50; ++i)
      {
        glVertex3f(i, -50, 0);
        glVertex3f(i, 50, 0);
        glVertex3f(-50, i, 0);
        glVertex3f(50, i, 0);
      }
      glEnd();
      glColor3f(.55, .6, .7);
      glPointSize(2);
      for (const auto &cloud : clouds)
      {
        drawPoints(cloud);
      }
      for (size_t i = 0; i < tracks.size(); ++i)
      {
        glColor3fv(colors[i % colors.size()].data());
        glLineWidth(2);
        glBegin(GL_LINE_STRIP);
        for (const auto &point : tracks[i].points)
        {
          glVertex3dv(point.data());
        }
        glEnd();
        if (!tracks[i].points.empty())
        {
          glPointSize(10);
          glBegin(GL_POINTS);
          glVertex3dv(tracks[i].points.back().data());
          glEnd();
        }
      }
      pangolin::FinishFrame();
      std::this_thread::sleep_until(next);
    }
  }
  catch (const std::exception &error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("scene_viewer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

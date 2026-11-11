#include <pangolin/display/default_font.h>
#include <pangolin/image/image_io.h>
#include <pangolin/pangolin.h>
#include <pangolin/plot/plotter.h>

#include "viewer_settings.hpp"

#include <array>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>

using Point = std::array<double, 3>;

void line(const std::deque<Point> &points, const pangolin::Colour &color)
{
  glColor3f(color.r, color.g, color.b);
  glBegin(GL_LINE_STRIP);
  for (auto &p : points)
  {
    glVertex3dv(p.data());
  }
  glEnd();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto      node                = std::make_shared<rclcpp::Node>("prediction_viewer");
    const int width               = node->declare_parameter("window_width", 1280),
              height              = node->declare_parameter("window_height", 800);
    const int            fps      = node->declare_parameter("video_fps", 15);
    const bool           record   = node->declare_parameter("record_video", true);
    const double         duration = node->declare_parameter("duration", 30.0);
    const ViewerSettings style(*node);
    const double         error_max = style.error_max, covariance_max = style.covariance_max;
    const auto           output =
        std::filesystem::path(node->declare_parameter<std::string>("output_dir", "outputs/demo"));
    auto frames =
        std::filesystem::path(node->declare_parameter<std::string>("frame_dir", "frames"));
    if (frames.is_relative())
    {
      frames = output / frames;
    }
    if (width <= 0 || height <= 0 || width % 2 || height % 2 || fps <= 0 || fps > 120)
    {
      throw std::invalid_argument("invalid video dimensions/fps");
    }
    std::ofstream frame_times;
    if (record)
    {
      std::filesystem::create_directories(frames);
      auto times_path = std::filesystem::path(
          node->declare_parameter<std::string>("frame_timestamps_file", "frame_times.csv"));
      if (times_path.is_relative())
      {
        times_path = output / times_path;
      }
      if (!times_path.parent_path().empty())
      {
        std::filesystem::create_directories(times_path.parent_path());
      }
      frame_times.open(times_path);
      if (!frame_times)
      {
        throw std::runtime_error("cannot open frame timestamps file");
      }
      frame_times << "frame,elapsed_s\n" << std::setprecision(12);
    }
    pangolin::CreateWindowAndBind("UAV trajectory prediction | IMM-KF", width, height);
    glEnable(GL_DEPTH_TEST);
    pangolin::OpenGlRenderState camera(
        pangolin::ProjectionMatrix(width * .62, height, style.focal_length, style.focal_length,
                                   width * .31, height * .5, style.near_clip, style.far_clip),
        pangolin::ModelViewLookAt(style.eye[0], style.eye[1], style.eye[2], style.target[0],
                                  style.target[1], style.target[2], pangolin::AxisZ));
    auto &view = pangolin::CreateDisplay()
                     .SetBounds(0, 1, 0, .62)
                     .SetHandler(new pangolin::Handler3D(camera));
    pangolin::DataLog errors, sigma, prob;
    errors.SetLabels({"time (s)", "position error (m)", "horizon error (m)"});
    sigma.SetLabels({"time (s)", "Pxx (m2)", "Pyy (m2)", "Pzz (m2)"});
    const auto model_names =
        node->declare_parameter<std::vector<std::string>>("motion_models", {"cv", "ca"});
    std::vector<std::string> probability_labels = {"time (s)"};
    for (const auto &name : model_names)
    {
      probability_labels.push_back("P(" + name + ")");
    }
    prob.SetLabels(probability_labels);
    pangolin::Plotter error_plot(&errors, 0, duration, 0, error_max, 5, error_max / 4),
        cov_plot(&sigma, 0, duration, 0, covariance_max, 5, covariance_max / 4),
        prob_plot(&prob, 0, duration, 0, 1, 5, .2);
    std::array<pangolin::Plotter *, 3> plots = {&error_plot, &cov_plot, &prob_plot};
    for (auto *p : plots)
    {
      p->ClearSeries();
      p->SetBackgroundColour(style.plot_background);
      p->SetAxisColour(style.plot_axis);
      p->SetTickColour(style.plot_ticks);
    }
    error_plot.AddSeries("$0", "$1", pangolin::DrawingModeLine, style.position_error,
                         "position error (m)");
    error_plot.AddSeries("$0", "$2", pangolin::DrawingModeLine, style.horizon_error,
                         "horizon error (m)");
    for (int i = 1; i <= 3; ++i)
    {
      cov_plot.AddSeries("$0", "$" + std::to_string(i) + "*$" + std::to_string(i),
                         pangolin::DrawingModeLine, style.covariance[i - 1],
                         "P" + std::string(2, "xyz"[i - 1]) + " (m2)");
    }
    for (size_t i = 0; i < model_names.size(); ++i)
    {
      prob_plot.AddSeries("$0", "$" + std::to_string(i + 1), pangolin::DrawingModeLine,
                          style.models[model_names[i] == "cv"   ? 0
                                       : model_names[i] == "ca" ? 1
                                                                : 2],
                          probability_labels[i + 1]);
    }
    error_plot.SetBounds(.67, 1, .62, 1);
    cov_plot.SetBounds(.34, .67, .62, 1);
    prob_plot.SetBounds(0, .34, .62, 1);
    for (auto *p : plots)
    {
      pangolin::DisplayBase().AddDisplay(*p);
    }
    std::deque<Point> truth, measure, estimate, future;
    Point             stddev       = {0, 0, 0};
    double            current_time = 0;
    auto              metrics      = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "metrics", 100,
        [&](const std_msgs::msg::Float64MultiArray &m)
        {
          if (m.data.size() < 19)
          {
            return;
          }
          auto &d      = m.data;
          current_time = d[0];
          truth.push_back({d[1], d[2], d[3]});
          measure.push_back({d[4], d[5], d[6]});
          estimate.push_back({d[7], d[8], d[9]});
          stddev = {d[10], d[11], d[12]};
          if (truth.size() > static_cast<size_t>(style.max_track_points))
          {
            truth.pop_front();
            measure.pop_front();
            estimate.pop_front();
          }
          errors.Log(d[0], d[15], d[16]);
          sigma.Log(d[0], d[10], d[11], d[12]);
          std::vector<float> probabilities = {static_cast<float>(d[0])};
          for (const auto &name : model_names)
          {
            size_t index = name == "cv" ? 13 : 14;
            if (name != "cv" && name != "ca")
            {
              index = 19;
              for (const auto &candidate : model_names)
              {
                if (candidate == name)
                {
                  break;
                }
                if (candidate != "cv" && candidate != "ca")
                {
                  ++index;
                }
              }
            }
            if (index >= d.size())
            {
              return;
            }
            probabilities.push_back(d[index]);
          }
          prob.Log(probabilities);
        });
    auto path = node->create_subscription<nav_msgs::msg::Path>(
        "prediction", 10,
        [&](const nav_msgs::msg::Path &m)
        {
          future.clear();
          for (auto &p : m.poses)
          {
            future.push_back({p.pose.position.x, p.pose.position.y, p.pose.position.z});
          }
        });
    auto       next            = std::chrono::steady_clock::now();
    int        frame           = 0;
    const auto recording_start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !pangolin::ShouldQuit())
    {
      rclcpp::spin_some(node);
      glClearColor(style.background.r, style.background.g, style.background.b, 1);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      view.Activate(camera);
      glEnable(GL_DEPTH_TEST);
      glLineWidth(1);
      if (style.show_grid)
      {
        glColor3f(style.grid.r, style.grid.g, style.grid.b);
        glBegin(GL_LINES);
        const int steps = static_cast<int>(style.grid_extent / style.grid_spacing);
        for (int i = -steps; i <= steps; ++i)
        {
          const double position = i * style.grid_spacing;
          glVertex3d(position, -style.grid_extent, 0);
          glVertex3d(position, style.grid_extent, 0);
          glVertex3d(-style.grid_extent, position, 0);
          glVertex3d(style.grid_extent, position, 0);
        }
        glEnd();
      }
      glLineWidth(style.line_width);
      line(truth, style.truth);
      line(estimate, style.estimate);
      line(future, style.forecast);
      glColor3f(style.observation.r, style.observation.g, style.observation.b);
      glPointSize(style.observation_size);
      glBegin(GL_POINTS);
      for (auto &p : measure)
      {
        glVertex3dv(p.data());
      }
      glEnd();
      if (!estimate.empty())
      {
        // Position marginal 2-sigma radii (axis-aligned; cross-correlation is
        // logged in odometry).
        auto p = estimate.back();
        if (style.show_uncertainty)
        {
          glColor3f(style.uncertainty.r, style.uncertainty.g, style.uncertainty.b);
          glBegin(GL_LINES);
          for (int k = 0; k < 3; ++k)
          {
            auto a = p, b = p;
            a[k] -= 2 * stddev[k];
            b[k] += 2 * stddev[k];
            glVertex3dv(a.data());
            glVertex3dv(b.data());
          }
          glEnd();
        }
        glColor3f(style.estimate.r, style.estimate.g, style.estimate.b);
        glPointSize(style.estimate_size);
        glBegin(GL_POINTS);
        glVertex3dv(p.data());
        glEnd();
      }
      // Render plots before capture, so the recording includes the entire
      // dashboard.
      pangolin::DisplayBase().Render();
      glViewport(0, 0, width, height);
      glDisable(GL_DEPTH_TEST);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadIdentity();
      glOrtho(0, width, 0, height, -1, 1);
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glLoadIdentity();
      if (style.show_legend)
      {
        glColor3f(style.text.r, style.text.g, style.text.b);
        pangolin::default_font().Text("IMM-KF | Trajectory prediction").Draw(20, height - 32);
        glColor3f(style.truth.r, style.truth.g, style.truth.b);
        pangolin::default_font().Text("Truth").Draw(20, height - 62);
        glColor3f(style.observation.r, style.observation.g, style.observation.b);
        pangolin::default_font().Text("Observation").Draw(100, height - 62);
        glColor3f(style.estimate.r, style.estimate.g, style.estimate.b);
        pangolin::default_font()
            .Text(style.show_uncertainty ? "Estimate + 2 sigma" : "Estimate")
            .Draw(240, height - 62);
        glColor3f(style.forecast.r, style.forecast.g, style.forecast.b);
        pangolin::default_font().Text("Forecast").Draw(460, height - 62);
        glColor3f(style.text.r, style.text.g, style.text.b);
        pangolin::default_font()
            .Text("t = %.2f s | World frame / meters", current_time)
            .Draw(20, height - 92);
      }
      glPopMatrix();
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      glMatrixMode(GL_MODELVIEW);
      if (record)
      {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - recording_start)
                .count();
        frame_times << frame << ',' << elapsed << '\n';
        std::vector<unsigned char> pixels(width * height * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
        pangolin::Image<unsigned char> image(pixels.data(), width, height, width * 3);
        std::ostringstream             name;
        name << "frame_" << std::setw(6) << std::setfill('0') << frame++ << ".png";
        pangolin::SaveImage(image, pangolin::PixelFormatFromString("RGB24"),
                            (frames / name.str()).string(), false);
      }
      pangolin::FinishFrame();
      next += std::chrono::microseconds(1000000 / fps);
      std::this_thread::sleep_until(next);
      if (next < std::chrono::steady_clock::now() - std::chrono::seconds(1))
      {
        next = std::chrono::steady_clock::now();
      }
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("viewer"), "%s", e.what());
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
  return 0;
}

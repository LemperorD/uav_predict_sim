#include <pangolin/display/default_font.h>
#include <pangolin/image/image_io.h>
#include <pangolin/pangolin.h>
#include <pangolin/plot/plotter.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "drone_model.hpp"
#include "predictor_visualization.hpp"
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
    if (std::all_of(p.begin(), p.end(), [](double value) { return std::isfinite(value); }))
    {
      glVertex3dv(p.data());
    }
  }
  glEnd();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto      node             = std::make_shared<rclcpp::Node>("prediction_viewer");
    const int initial_width    = node->declare_parameter("window_width", 1280),
              initial_height   = node->declare_parameter("window_height", 800);
    int                  width = initial_width, height = initial_height;
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
    if (width <= 0 || height <= 0 || fps <= 0 || fps > 120)
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
      frame_times << "frame,elapsed_s,width,height\n" << std::setprecision(12);
    }
    const auto algorithm = node->declare_parameter<std::string>("prediction_algorithm", "imm");
    pangolin::CreateWindowAndBind("UAV trajectory prediction | " + algorithm, width, height);
    glEnable(GL_DEPTH_TEST);
    std::unique_ptr<DroneModel> drone;
    if (style.show_drone)
    {
      auto mesh_file = node->declare_parameter<std::string>("drone_mesh_file", "");
      if (mesh_file.empty())
      {
        mesh_file = ament_index_cpp::get_package_share_directory("prediction_visualize") +
                    "/meshes/hummingbird.ply";
      }
      drone = std::make_unique<DroneModel>(mesh_file);
    }
    DronePose                                                drone_pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr drone_odometry;
    if (style.show_drone)
    {
      const auto pose_topic = style.drone_pose_source == "truth" ? "odometry" : "estimate";
      drone_odometry        = node->create_subscription<nav_msgs::msg::Odometry>(
          pose_topic, rclcpp::SensorDataQoS(),
          [&](const nav_msgs::msg::Odometry &message)
          {
            if (!drone_pose.update(message.pose.pose))
            {
              RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                                          "Ignoring invalid drone position/quaternion");
            }
          });
    }
    pangolin::OpenGlRenderState camera(
        pangolin::ProjectionMatrix(width * .62, height, style.focal_length, style.focal_length,
                                   width * .31, height * .5, style.near_clip, style.far_clip),
        pangolin::ModelViewLookAt(style.eye[0], style.eye[1], style.eye[2], style.target[0],
                                  style.target[1], style.target[2], pangolin::AxisZ));
    auto &view = pangolin::CreateDisplay()
                     .SetBounds(0, 1, 0, .62)
                     .SetHandler(new pangolin::Handler3D(camera));
    const auto model_names =
        node->declare_parameter<std::vector<std::string>>("motion_models", {"cv", "ca"});
    const auto profile  = node->declare_parameter<std::string>("plot_profile", "auto");
    const auto selected = profile == "auto" ? algorithm : profile;
    std::unique_ptr<PredictorVisualization> visualization;
    if (selected == "imm")
      visualization = std::make_unique<ImmVisualization>(model_names);
    else if (selected == "pose_kf" || selected == "kf")
      visualization = std::make_unique<PoseKFVisualization>();
    else
      throw std::invalid_argument("Unknown plot_profile: " + selected);
    const double velocity_max = node->declare_parameter("velocity_error_plot_max", 2.0);
    if (!std::isfinite(velocity_max) || velocity_max <= 0)
      throw std::invalid_argument("velocity_error_plot_max must be positive and finite");
    const auto panels = visualization->panels(error_max, covariance_max, velocity_max);
    std::vector<std::unique_ptr<pangolin::DataLog>> logs;
    std::vector<std::unique_ptr<pangolin::Plotter>> plots;
    for (size_t i = 0; i < panels.size(); ++i)
    {
      const auto &panel = panels[i];
      logs.push_back(std::make_unique<pangolin::DataLog>());
      std::vector<std::string> labels = {"time (s)"};
      for (const auto &curve : panel.curves) labels.push_back(curve.label);
      logs.back()->SetLabels(labels);
      plots.push_back(std::make_unique<pangolin::Plotter>(logs.back().get(), 0, duration, 0,
                                                          panel.maximum, 5, panel.maximum / 4));
      auto &plot = *plots.back();
      plot.ClearSeries();
      plot.SetBackgroundColour(style.plot_background);
      plot.SetAxisColour(style.plot_axis);
      plot.SetTickColour(style.plot_ticks);
      for (size_t j = 0; j < panel.curves.size(); ++j)
      {
        const auto color = i == 0   ? (j == 0 ? style.position_error : style.horizon_error)
                           : i == 1 ? style.covariance[j % 3]
                                    : style.models[j % 3];
        plot.AddSeries("$0", "$" + std::to_string(j + 1), pangolin::DrawingModeLine, color,
                       panel.curves[j].label);
      }
      plot.SetBounds(1.0 - double(i + 1) / panels.size(), 1.0 - double(i) / panels.size(), .62, 1);
      pangolin::DisplayBase().AddDisplay(plot);
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
          for (size_t i = 0; i < panels.size(); ++i)
          {
            std::vector<float> values = {static_cast<float>(d[0])};
            for (const auto &curve : panels[i].curves)
            {
              const double value = curve.metric_index < d.size()
                                                         ? d[curve.metric_index]
                                                         : std::numeric_limits<double>::quiet_NaN();
              values.push_back(static_cast<float>(curve.square ? value * value : value));
            }
            logs[i]->Log(values);
          }
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
    // Drain queued samples each frame; spinning only once per subscription makes
    // the 100 Hz trajectory lag behind the latest pose at the video frame rate.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto       next            = std::chrono::steady_clock::now();
    int        frame           = 0;
    const auto recording_start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !pangolin::ShouldQuit())
    {
      executor.spin_all(std::chrono::milliseconds(5));
      const auto viewport = pangolin::DisplayBase().v;
      if (viewport.w <= 0 || viewport.h <= 0)
      {
        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        next = std::chrono::steady_clock::now();
        continue;
      }
      if (width != viewport.w || height != viewport.h)
      {
        width  = viewport.w;
        height = viewport.h;
        // Preserve the user's orbit/pan and vertical field of view when resizing.
        const double focal = style.focal_length * height / initial_height;
        camera.SetProjectionMatrix(pangolin::ProjectionMatrix(view.v.w, view.v.h, focal, focal,
                                                              view.v.w * .5, view.v.h * .5,
                                                              style.near_clip, style.far_clip));
      }
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
      if (!estimate.empty() && std::all_of(estimate.back().begin(), estimate.back().end(),
                                           [](double value) { return std::isfinite(value); }))
      {
        // Position marginal 2-sigma radii (axis-aligned; cross-correlation is
        // logged in odometry).
        auto p = estimate.back();
        if (style.show_uncertainty &&
            std::all_of(stddev.begin(), stddev.end(),
                        [](double value) { return std::isfinite(value); }))
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
      if (drone)
      {
        drone->draw(drone_pose, style.drone_scale, style.drone_body, style.drone_front,
                    style.show_drone_axes, style.drone_axis_length);
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
        const double hud_scale = std::min(1.0, width / 1280.0);
        const double text_top  = height / hud_scale;
        glScaled(hud_scale, hud_scale, 1);
        // Draw(x,y) keeps Pangolin text at a fixed pixel size. Draw in our
        // transformed local coordinates so the whole legend scales together.
        auto draw_text = [](const pangolin::GlText &text, double x, double y)
        {
          glPushMatrix();
          glTranslated(x, y, 0);
          text.Draw();
          glPopMatrix();
        };
        glColor3f(style.text.r, style.text.g, style.text.b);
        draw_text(pangolin::default_font().Text("Trajectory prediction"), 20, text_top - 32);
        glColor3f(style.truth.r, style.truth.g, style.truth.b);
        draw_text(pangolin::default_font().Text("Truth"), 20, text_top - 62);
        glColor3f(style.observation.r, style.observation.g, style.observation.b);
        draw_text(pangolin::default_font().Text("Observation"), 100, text_top - 62);
        glColor3f(style.estimate.r, style.estimate.g, style.estimate.b);
        draw_text(pangolin::default_font().Text(style.show_uncertainty ? "Estimate + 2 sigma"
                                                                       : "Estimate"),
                  240, text_top - 62);
        glColor3f(style.forecast.r, style.forecast.g, style.forecast.b);
        draw_text(pangolin::default_font().Text("Forecast"), 460, text_top - 62);
        glColor3f(style.text.r, style.text.g, style.text.b);
        draw_text(pangolin::default_font().Text("t = %.2f s | World frame / meters", current_time),
                  20, text_top - 92);
        if (style.show_drone)
        {
          draw_text(pangolin::default_font().Text("Drone: %s | body axes: X red, Y green, Z blue",
                                                  style.drone_pose_source.c_str()),
                    20, text_top - 120);
        }
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
        frame_times << frame << ',' << elapsed << ',' << width << ',' << height << '\n';
        std::vector<unsigned char> pixels(static_cast<size_t>(width) * height * 3);
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

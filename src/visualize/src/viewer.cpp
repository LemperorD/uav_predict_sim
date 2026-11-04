#include <array>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <nav_msgs/msg/path.hpp>
#include <pangolin/display/default_font.h>
#include <pangolin/image/image_io.h>
#include <pangolin/pangolin.h>
#include <pangolin/plot/plotter.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>

using Point = std::array<double, 3>;
void line(const std::deque<Point> &points, float r, float g, float b) {
  glColor3f(r, g, b);
  glBegin(GL_LINE_STRIP);
  for (auto &p : points)
    glVertex3dv(p.data());
  glEnd();
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>("prediction_viewer");
    const int width = node->declare_parameter("window_width", 1280),
              height = node->declare_parameter("window_height", 800);
    const int fps = node->declare_parameter("video_fps", 15);
    const bool record = node->declare_parameter("record_video", true);
    const double duration = node->declare_parameter("duration", 30.0);
    const double error_max = node->declare_parameter("error_plot_max", 1.0);
    const double covariance_max =
        node->declare_parameter("covariance_plot_max", 0.001);
    const auto output = std::filesystem::path(
        node->declare_parameter<std::string>("output_dir", "outputs/demo"));
    auto frames = std::filesystem::path(
        node->declare_parameter<std::string>("frame_dir", "frames"));
    if (frames.is_relative())
      frames = output / frames;
    if (width <= 0 || height <= 0 || width % 2 || height % 2 || fps <= 0 ||
        fps > 120)
      throw std::invalid_argument("invalid video dimensions/fps");
    if (record)
      std::filesystem::create_directories(frames);
    pangolin::CreateWindowAndBind("UAV IMM-KF | truth green - observation gray "
                                  "- estimate blue - forecast orange",
                                  width, height);
    glEnable(GL_DEPTH_TEST);
    pangolin::OpenGlRenderState camera(
        pangolin::ProjectionMatrix(width * .62, height, 850, 850, width * .31,
                                   height * .5, .1, 1000),
        pangolin::ModelViewLookAt(8, -10, 8, 0, 0, 2, pangolin::AxisZ));
    auto &view = pangolin::CreateDisplay()
                     .SetBounds(0, 1, 0, .62)
                     .SetHandler(new pangolin::Handler3D(camera));
    pangolin::DataLog errors, sigma, prob;
    errors.SetLabels({"time (s)", "position error (m)", "horizon error (m)"});
    sigma.SetLabels({"time (s)", "Pxx (m2)", "Pyy (m2)", "Pzz (m2)"});
    prob.SetLabels({"time (s)", "P(CV)", "P(CA)"});
    pangolin::Plotter error_plot(&errors, 0, duration, 0, error_max, 5,
                                 error_max / 4),
        cov_plot(&sigma, 0, duration, 0, covariance_max, 5, covariance_max / 4),
        prob_plot(&prob, 0, duration, 0, 1, 5, .2);
    std::array<pangolin::Plotter *, 3> plots = {&error_plot, &cov_plot,
                                                &prob_plot};
    for (auto *p : plots) {
      p->ClearSeries();
      p->SetBackgroundColour(pangolin::Colour(.06, .08, .12));
    }
    error_plot.AddSeries("$0", "$1", pangolin::DrawingModeLine,
                         pangolin::Colour(.2, .7, 1), "position error (m)");
    error_plot.AddSeries("$0", "$2", pangolin::DrawingModeLine,
                         pangolin::Colour(1, .6, .1), "horizon error (m)");
    for (int i = 1; i <= 3; ++i)
      cov_plot.AddSeries(
          "$0", "$" + std::to_string(i) + "*$" + std::to_string(i),
          pangolin::DrawingModeLine, pangolin::Colour::Unspecified(),
          "P" + std::string(2, "xyz"[i - 1]) + " (m2)");
    prob_plot.AddSeries("$0", "$1", pangolin::DrawingModeLine,
                        pangolin::Colour(.2, .9, .4), "P(CV)");
    prob_plot.AddSeries("$0", "$2", pangolin::DrawingModeLine,
                        pangolin::Colour(.9, .3, .8), "P(CA)");
    error_plot.SetBounds(.67, 1, .62, 1);
    cov_plot.SetBounds(.34, .67, .62, 1);
    prob_plot.SetBounds(0, .34, .62, 1);
    for (auto *p : plots)
      pangolin::DisplayBase().AddDisplay(*p);
    std::deque<Point> truth, measure, estimate, future;
    Point stddev = {0, 0, 0};
    double current_time = 0;
    auto metrics = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "metrics", 100, [&](const std_msgs::msg::Float64MultiArray &m) {
          if (m.data.size() != 19)
            return;
          auto &d = m.data;
          current_time = d[0];
          truth.push_back({d[1], d[2], d[3]});
          measure.push_back({d[4], d[5], d[6]});
          estimate.push_back({d[7], d[8], d[9]});
          stddev = {d[10], d[11], d[12]};
          if (truth.size() > 20000) {
            truth.pop_front();
            measure.pop_front();
            estimate.pop_front();
          }
          errors.Log(d[0], d[15], d[16]);
          sigma.Log(d[0], d[10], d[11], d[12]);
          prob.Log(d[0], d[13], d[14]);
        });
    auto path = node->create_subscription<nav_msgs::msg::Path>(
        "prediction", 10, [&](const nav_msgs::msg::Path &m) {
          future.clear();
          for (auto &p : m.poses)
            future.push_back(
                {p.pose.position.x, p.pose.position.y, p.pose.position.z});
        });
    auto next = std::chrono::steady_clock::now();
    int frame = 0;
    while (rclcpp::ok() && !pangolin::ShouldQuit()) {
      rclcpp::spin_some(node);
      glClearColor(.04, .055, .08, 1);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      view.Activate(camera);
      glEnable(GL_DEPTH_TEST);
      glLineWidth(1);
      glColor3f(.2, .25, .3);
      glBegin(GL_LINES);
      for (int i = -10; i <= 10; ++i) {
        glVertex3f(i, -10, 0);
        glVertex3f(i, 10, 0);
        glVertex3f(-10, i, 0);
        glVertex3f(10, i, 0);
      }
      glEnd();
      glLineWidth(2);
      line(truth, .2, .9, .4);
      line(estimate, .2, .65, 1);
      line(future, 1, .6, .1);
      glColor3f(.55, .55, .6);
      glPointSize(2);
      glBegin(GL_POINTS);
      for (auto &p : measure)
        glVertex3dv(p.data());
      glEnd();
      if (!estimate.empty()) {
        // Position marginal 2-sigma radii (axis-aligned; cross-correlation is
        // logged in odometry).
        auto p = estimate.back();
        glColor3f(.3, .7, 1);
        glBegin(GL_LINES);
        for (int k = 0; k < 3; ++k) {
          auto a = p, b = p;
          a[k] -= 2 * stddev[k];
          b[k] += 2 * stddev[k];
          glVertex3dv(a.data());
          glVertex3dv(b.data());
        }
        glEnd();
        glPointSize(9);
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
      glColor3f(.9, .94, 1);
      pangolin::default_font()
          .Text("IMM-KF | EGO kinematic simulation")
          .Draw(20, height - 32);
      glColor3f(.2, .9, .4);
      pangolin::default_font().Text("Truth").Draw(20, height - 62);
      glColor3f(.6, .6, .65);
      pangolin::default_font().Text("Observation").Draw(100, height - 62);
      glColor3f(.2, .65, 1);
      pangolin::default_font()
          .Text("Estimate + 2 sigma")
          .Draw(240, height - 62);
      glColor3f(1, .6, .1);
      pangolin::default_font().Text("Forecast").Draw(460, height - 62);
      glColor3f(.8, .85, .9);
      pangolin::default_font()
          .Text("t = %.2f s | World frame / meters", current_time)
          .Draw(20, height - 92);
      glPopMatrix();
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      glMatrixMode(GL_MODELVIEW);
      if (record) {
        std::vector<unsigned char> pixels(width * height * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE,
                     pixels.data());
        pangolin::Image<unsigned char> image(pixels.data(), width, height,
                                             width * 3);
        std::ostringstream name;
        name << "frame_" << std::setw(6) << std::setfill('0') << frame++
             << ".png";
        pangolin::SaveImage(image, pangolin::PixelFormatFromString("RGB24"),
                            (frames / name.str()).string(), false);
      }
      pangolin::FinishFrame();
      next += std::chrono::microseconds(1000000 / fps);
      std::this_thread::sleep_until(next);
      if (next < std::chrono::steady_clock::now() - std::chrono::seconds(1))
        next = std::chrono::steady_clock::now();
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("viewer"), "%s", e.what());
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok())
    rclcpp::shutdown();
  return 0;
}

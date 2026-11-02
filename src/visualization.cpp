#include "uav_predict_sim/presentation.hpp"
#include <pangolin/pangolin.h>
#include <algorithm>
#include <chrono>
#include <thread>

namespace uav {
namespace {
void vertex(const Vec3& point) { glVertex3d(point.x(), point.y(), point.z()); }
void line(const Vec3& a, const Vec3& b) { vertex(a); vertex(b); }
}  // namespace

void view(const Experiment& experiment) {
  pangolin::CreateWindowAndBind("UAV Predict Sim - point mass playback", 1200, 800);
  glEnable(GL_DEPTH_TEST);
  auto camera = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(1200, 800, 600, 600, 600, 400, 0.1, 1000),
      pangolin::ModelViewLookAt(10, -12, 10, 0, 0, 1, pangolin::AxisZ));
  pangolin::CreatePanel("ui").SetBounds(0, 1, 0, pangolin::Attach::Pix(210));
  auto& display = pangolin::CreateDisplay().SetBounds(0, 1, pangolin::Attach::Pix(210), 1)
      .SetHandler(new pangolin::Handler3D(camera));
  const int last = static_cast<int>(experiment.states.size()) - 1;
  pangolin::Var<bool> play("ui.Play", true, true);
  pangolin::Var<int> frame("ui.Sample", 0, 0, last);
  pangolin::Var<bool> observations("ui.Observations", true, true);
  pangolin::Var<bool> show_cv("ui.CV", true, true);
  pangolin::Var<bool> show_ca("ui.CA", true, true);
  auto previous = std::chrono::steady_clock::now();
  double playback_time = 0;
  while (!pangolin::ShouldQuit()) {
    const auto now = std::chrono::steady_clock::now();
    if (frame.GuiChanged()) playback_time = experiment.states[static_cast<int>(frame)].time;
    if (play) playback_time += std::chrono::duration<double>(now - previous).count();
    previous = now;
    const double dt = experiment.states[1].time - experiment.states[0].time;
    const int index = std::clamp(static_cast<int>(playback_time / dt), 0, last);
    frame = index;
    if (index == last) play = false;
    const auto& current = experiment.states[index];

    glClearColor(0.96f, 0.97f, 0.98f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    display.Activate(camera);
    glColor3f(0.8f, 0.8f, 0.8f);
    glBegin(GL_LINES);
    for (int i = -10; i <= 10; ++i) {
      line(Vec3(i, -10, 0), Vec3(i, 10, 0));
      line(Vec3(-10, i, 0), Vec3(10, i, 0));
    }
    glEnd();
    glLineWidth(2);
    glColor3f(0.1f, 0.1f, 0.1f);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= index; ++i) vertex(experiment.states[i].position);
    glEnd();
    if (observations) {
      glColor3f(0.2f, 0.65f, 0.3f); glPointSize(3);
      glBegin(GL_POINTS);
      for (const auto& sample : experiment.observations)
        if (sample.time <= current.time) vertex(sample.position);
      glEnd();
    }
    for (auto model : {Model::CV, Model::CA}) {
      if ((model == Model::CV && !show_cv) || (model == Model::CA && !show_ca)) continue;
      const Forecast* latest = nullptr;
      for (const auto& forecast : experiment.forecasts)
        if (forecast.model == model && forecast.origin <= current.time) latest = &forecast;
      // After the final evaluated origin, hide expired forecasts.
      if (latest && current.time <= latest->points.back().time) {
        if (model == Model::CV) glColor3f(0.15f, 0.4f, 0.95f);
        else glColor3f(0.95f, 0.4f, 0.1f);
        glBegin(GL_LINE_STRIP);
        for (const auto& sample : latest->points) vertex(sample.position);
        glEnd();
      }
    }
    // Position marker only: the current model has no attitude state.
    glColor3f(0.8f, 0.1f, 0.2f); glLineWidth(4);
    glBegin(GL_LINES);
    for (int axis = 0; axis < 3; ++axis) {
      Vec3 offset = Vec3::Zero(); offset[axis] = 0.18;
      line(current.position - offset, current.position + offset);
    }
    glEnd(); glLineWidth(1);
    pangolin::FinishFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
}  // namespace uav

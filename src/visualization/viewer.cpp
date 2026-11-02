// SPDX-License-Identifier: GPL-3.0-only
#include "uav_predict_sim/simulation/experiment.hpp"
#include <pangolin/pangolin.h>
#include <pangolin/display/default_font.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace uav {
namespace {
void vertex(const Vec3& p) { glVertex3d(p.x(),p.y(),p.z()); }
void segment(const Vec3& a,const Vec3& b) { vertex(a); vertex(b); }
std::string number(double value,int precision=2) {
  std::ostringstream s; s<<std::fixed<<std::setprecision(precision)<<value; return s.str();
}
void drone(const State& state,double arm) {
  Eigen::Matrix4d transform=Eigen::Matrix4d::Identity();
  transform.topLeftCorner<3,3>()=state.rotation(); transform.topRightCorner<3,1>()=state.position;
  glPushMatrix(); glMultMatrixd(transform.data());
  const Vec3 motors[]={Vec3(arm,0,0),Vec3(-arm,0,0),Vec3(0,arm,0),Vec3(0,-arm,0)};
  glLineWidth(5);
  for(int i=0;i<4;++i) {
    if(i==0) glColor3f(0.85f,0.15f,0.1f); else glColor3f(0.2f,0.2f,0.25f);
    glBegin(GL_LINES); segment(Vec3::Zero(),motors[i]); glEnd();
    glLineWidth(2); glBegin(GL_LINE_LOOP);
    for(int k=0;k<32;++k) {
      const double angle=6.283185307179586*k/32;
      vertex(motors[i]+Vec3(0.062*std::cos(angle),0.062*std::sin(angle),0));
    }
    glEnd();
    pangolin::default_font().Text("%d",i+1).Draw(motors[i].x(),motors[i].y(),0.07f);
    glLineWidth(5);
  }
  glLineWidth(2); glBegin(GL_LINES);
  glColor3f(1,0,0); segment(Vec3::Zero(),Vec3(0.45,0,0));
  glColor3f(0,0.7f,0); segment(Vec3::Zero(),Vec3(0,0.45,0));
  glColor3f(0,0.3f,1); segment(Vec3::Zero(),Vec3(0,0,0.5));
  glEnd(); glLineWidth(1); glPopMatrix();
}
}
void view(const Experiment& e,const VehicleParams& vehicle) {
  if(e.frames.size()<2) throw std::invalid_argument("not enough frames for replay");
  pangolin::CreateWindowAndBind("UAV Predict Sim - SO3 quadrotor replay",1280,850);
  glEnable(GL_DEPTH_TEST);
  const Vec3 center=e.frames.front().state.position;
  pangolin::OpenGlRenderState camera(
      pangolin::ProjectionMatrix(1280,850,650,650,640,425,0.05,1000),
      pangolin::ModelViewLookAt(center.x()+9,center.y()-12,center.z()+8,
                              center.x(),center.y(),center.z(),pangolin::AxisZ));
  pangolin::CreatePanel("ui").SetBounds(0,1,0,pangolin::Attach::Pix(240));
  auto& display=pangolin::CreateDisplay().SetBounds(0,1,pangolin::Attach::Pix(240),1)
      .SetHandler(new pangolin::Handler3D(camera));
  const int last=static_cast<int>(e.frames.size())-1;
  pangolin::Var<bool> play("ui.Play",true,true), next("ui.Step 10ms",false,false), follow("ui.Follow",false,true);
  pangolin::Var<int> frame("ui.Sample",0,0,last);
  pangolin::Var<double> speed("ui.Speed",1,0.1,5);
  pangolin::Var<bool> show_reference("ui.Reference gray",true,true), show_observations("ui.Observations green",false,true),
      show_cv("ui.CV blue",true,true),show_ca("ui.CA orange",true,true);
  pangolin::Var<std::string> time_text("ui.Time s","0"),height_text("ui.Height m","0"),
      velocity_text("ui.Speed m_s","0"),tilt_text("ui.Tilt deg","0"),constraint_text("ui.State","nominal"),
      origin_text("ui.Forecast origin","none");
  auto previous=std::chrono::steady_clock::now();
  double playback=0;
  const double dt=e.frames[1].state.time-e.frames[0].state.time;
  while(!pangolin::ShouldQuit()) {
    const auto now=std::chrono::steady_clock::now();
    const bool scrub=frame.GuiChanged();
    if(scrub) playback=e.frames[std::clamp(static_cast<int>(frame),0,last)].state.time;
    if(pangolin::Pushed(next)) { play=false; playback=std::min(e.frames.back().state.time,playback+0.01); }
    if(play && !scrub) playback+=std::chrono::duration<double>(now-previous).count()*static_cast<double>(speed);
    previous=now;
    playback=std::clamp(playback,0.0,e.frames.back().state.time);
    const int index=std::clamp(static_cast<int>(std::round(playback/dt)),0,last);
    frame=index; if(index==last) play=false;
    const auto& current=e.frames[index]; const auto& state=current.state;
    time_text=number(state.time,3); height_text=number(state.position.z()); velocity_text=number(state.velocity.norm());
    tilt_text=number(current.tilt*57.29577951308232);
    constraint_text=current.allocation.saturated?"motor saturated":current.state_feasible?"nominal":"state limit";
    if(follow) camera.SetModelViewMatrix(pangolin::ModelViewLookAt(state.position.x()+7,state.position.y()-9,state.position.z()+5,
        state.position.x(),state.position.y(),state.position.z(),pangolin::AxisZ));
    glClearColor(0.96f,0.97f,0.98f,1); glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    display.Activate(camera);
    glColor3f(0.8f,0.8f,0.8f); glBegin(GL_LINES);
    for(int k=-15;k<=15;++k) { segment(Vec3(k,-15,0),Vec3(k,15,0)); segment(Vec3(-15,k,0),Vec3(15,k,0)); }
    glEnd();
    if(show_reference) {
      glColor3f(0.65f,0.65f,0.65f); glBegin(GL_LINE_STRIP);
      for(std::size_t i=0;i<e.frames.size();i+=10) vertex(e.frames[i].reference.position);
      glEnd();
    }
    glColor3f(0.05f,0.05f,0.05f); glLineWidth(2); glBegin(GL_LINE_STRIP);
    for(int i=0;i<=index;i+=5) vertex(e.frames[i].state.position);
    vertex(state.position); glEnd();
    if(show_observations) {
      glPointSize(3); glColor3f(0.1f,0.65f,0.2f); glBegin(GL_POINTS);
      for(const auto& sample:e.observations) if(sample.time<=state.time) vertex(sample.position);
      glEnd();
    }
    origin_text="none";
    for(auto model:{PredictionModel::CV,PredictionModel::CA}) {
      if((model==PredictionModel::CV && !show_cv) || (model==PredictionModel::CA && !show_ca)) continue;
      const Forecast* latest=nullptr;
      for(const auto& f:e.forecasts) if(f.model==model && f.origin<=state.time) latest=&f;
      if(!latest || latest->predicted.empty() || state.time>latest->predicted.back().time) continue;
      origin_text=number(latest->origin,2)+" s";
      if(model==PredictionModel::CV) glColor3f(0.1f,0.35f,0.95f); else glColor3f(0.95f,0.4f,0.05f);
      glBegin(GL_LINE_STRIP);
      for(const auto& point:latest->predicted) vertex(point.position);
      glEnd();
    }
    drone(state,vehicle.arm);
    pangolin::FinishFrame(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
} // namespace uav

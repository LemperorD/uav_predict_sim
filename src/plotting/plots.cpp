// SPDX-License-Identifier: GPL-3.0-only
// Python.h must precede system headers.
#include "matplotlibcpp.h"
#include "uav_predict_sim/simulation/experiment.hpp"
#include <algorithm>
#include <cmath>

namespace uav {
namespace {
namespace plt=matplotlibcpp;
void line(const std::vector<double>& x,const std::vector<double>& y,const std::string& label,
          const std::string& color="",const std::string& style="-") {
  std::map<std::string,std::string> kw{{"label",label},{"linestyle",style}};
  if(!color.empty()) kw["color"]=color;
  if(!plt::plot(x,y,kw)) { PyErr_Print(); throw std::runtime_error("matplotlib plot failed"); }
}
void axes(const char* title,const char* xlabel,const char* ylabel) {
  plt::title(title); plt::xlabel(xlabel); plt::ylabel(ylabel); plt::grid(true); plt::legend();
}
void finish(const std::filesystem::path& directory,const char* file) {
  plt::save((directory/file).string()); plt::close();
}
void equal_3d(const Vec3& low,const Vec3& high) {
  const Vec3 center=(low+high)/2;
  const double r=std::max(0.5,(high-low).maxCoeff()*0.55);
  plt::xlim(center.x()-r,center.x()+r); plt::ylim(center.y()-r,center.y()+r);
  PyObject* ax=PyObject_CallObject(plt::detail::_interpreter::get().s_python_function_gca,
                                 plt::detail::_interpreter::get().s_python_empty_tuple);
  if(!ax) throw std::runtime_error("missing 3D axes");
  PyObject* z=PyObject_CallMethod(ax,"set_zlim","(dd)",center.z()-r,center.z()+r);
  PyObject* shape=Py_BuildValue("(ddd)",1.0,1.0,1.0);
  PyObject* aspect=PyObject_CallMethod(ax,"set_box_aspect","(O)",shape);
  const bool ok=z && aspect;
  Py_XDECREF(z); Py_XDECREF(aspect); Py_DECREF(shape); Py_DECREF(ax);
  if(!ok) { PyErr_Print(); throw std::runtime_error("3D aspect configuration failed"); }
}
long current_figure() {
  PyObject* module=PyImport_ImportModule("matplotlib.pyplot");
  PyObject* figure=module?PyObject_CallMethod(module,"gcf",nullptr):nullptr;
  PyObject* number=figure?PyObject_GetAttrString(figure,"number"):nullptr;
  const long id=number?PyLong_AsLong(number):-1;
  Py_XDECREF(number); Py_XDECREF(figure); Py_XDECREF(module);
  if(id<0) throw std::runtime_error("cannot determine matplotlib figure number");
  return id;
}
}
void plot(const Experiment& e,const std::filesystem::path& directory) {
  if(e.frames.empty() || e.forecasts.empty()) throw std::invalid_argument("empty plotting input");
  const Config config=load_config(directory/"config.yaml");
  plt::backend("Agg");
  std::vector<double> t,x[3],r[3],v[3],a[3],omega[3],tilt,heading,yaw,error,thrust,actual_thrust,rpm[4];
  Vec3 low=e.frames.front().state.position,high=low;
  const std::size_t stride=std::max<std::size_t>(1,e.frames.size()/2000);
  for(std::size_t i=0;i<e.frames.size();i+=stride) {
    const auto& f=e.frames[i]; t.push_back(f.state.time);
    for(int k=0;k<3;++k) {
      x[k].push_back(f.state.position[k]); r[k].push_back(f.reference.position[k]);
      v[k].push_back(f.state.velocity[k]); a[k].push_back(f.acceleration[k]);
      omega[k].push_back(f.state.omega[k]);
    }
    for(int k=0;k<4;++k) rpm[k].push_back(f.state.rpm[k]);
    low=low.cwiseMin(f.state.position); high=high.cwiseMax(f.state.position);
    tilt.push_back(f.tilt); heading.push_back(f.reference.heading);
    const Mat3 rotation=f.state.rotation(); yaw.push_back(std::atan2(rotation(1,0),rotation(0,0)));
    error.push_back(f.position_error); thrust.push_back(f.control.requested.thrust);
    actual_thrust.push_back(rotor_wrench(f.state.rpm,config.vehicle).thrust);
  }
  plt::figure_size(1000,800);
  // figure_size creates a numbered figure; pass its actual id to plot3.
  const long figure=current_figure();
  plt::plot3(r[0],r[1],r[2],{{"label","Reference"},{"color","0.6"},{"linestyle","--"}},figure);
  plt::plot3(x[0],x[1],x[2],{{"label","Actual 6-DoF flight"},{"color","black"}},figure);
  const double origin=e.frames.back().state.time*0.55;
  // matplotlib-cpp exposes borrowed NumPy views; mplot3d retains these arrays
  // until drawing. Keep each method's buffers alive until after save().
  std::vector<double> prediction_buffers[2][3];
  for(auto model:{PredictionModel::CV,PredictionModel::CA}) {
    const Forecast* selected=nullptr;
    for(const auto& f:e.forecasts)
      if(f.model==model && f.status=="complete" && (!selected || std::abs(f.origin-origin)<std::abs(selected->origin-origin))) selected=&f;
    if(!selected) continue;
    auto& points=prediction_buffers[model==PredictionModel::CV?0:1];
    for(const auto& point:selected->predicted) {
      for(int k=0;k<3;++k) points[k].push_back(point.position[k]);
      low=low.cwiseMin(point.position); high=high.cwiseMax(point.position);
    }
    plt::plot3(points[0],points[1],points[2],{{"label",std::string(name(model))+" forecast, t="+std::to_string(selected->origin).substr(0,4)+"s"},
        {"color",model==PredictionModel::CV?"tab:blue":"tab:orange"}},figure);
  }
  equal_3d(low,high);
  plt::title("Three-dimensional quadrotor flight and position-only forecasts");
  plt::xlabel("x [m]"); plt::ylabel("y [m]"); plt::set_zlabel("z [m]"); plt::legend();
  finish(directory,"trajectory3d.png");

  const char* coordinates[]={"x","y","z"};
  plt::figure_size(1200,1000);
  for(int k=0;k<3;++k) {
    plt::subplot(3,2,2*k+1); line(t,r[k],"Reference","0.65","--"); line(t,x[k],"Actual","black");
    axes((std::string(coordinates[k])+" position").c_str(),"Time [s]","Position [m]");
    plt::subplot(3,2,2*k+2); line(t,v[k],"Velocity","tab:blue"); line(t,a[k],"Acceleration","tab:orange");
    axes((std::string(coordinates[k])+" motion").c_str(),"Time [s]","m/s or m/s^2");
  }
  plt::subplots_adjust({{"hspace",0.4},{"wspace",0.3}}); finish(directory,"tracking.png");

  plt::figure_size(1200,1000);
  plt::subplot(3,2,1); line(t,tilt,"Actual tilt"); line(t,std::vector<double>(t.size(),config.limits.max_tilt),"Tilt limit","red","--");
  axes("Tilt","Time [s]","rad");
  plt::subplot(3,2,2); line(t,heading,"Reference heading"); line(t,yaw,"Actual Euler yaw");
  axes("Heading parameter and measured yaw","Time [s]","rad");
  plt::subplot(3,2,3);
  for(int k=0;k<3;++k) line(t,omega[k],coordinates[k]);
  axes("Body angular velocity","Time [s]","rad/s");
  plt::subplot(3,2,4); line(t,thrust,"Commanded"); line(t,actual_thrust,"Actual","black");
  axes("Collective thrust and motor lag","Time [s]","N");
  plt::subplot(3,2,5);
  for(int k=0;k<4;++k) line(t,rpm[k],"Rotor "+std::to_string(k+1));
  axes("Rotor speeds (+ layout)","Time [s]","RPM");
  plt::subplot(3,2,6); line(t,error,"Tracking error","black");
  axes("Actual versus reference position","Time [s]","m");
  plt::subplots_adjust({{"hspace",0.4},{"wspace",0.3}}); finish(directory,"dynamics.png");

  plt::figure_size(1200,500);
  for(auto model:{PredictionModel::CV,PredictionModel::CA}) {
    std::vector<double> lead,rmse,origins,ade;
    for(std::size_t i=0;i<e.forecasts.front().predicted.size();++i) {
      double sum=0; std::size_t n=0;
      for(const auto& f:e.forecasts) if(f.model==model && f.status=="complete") {
        sum+=(f.predicted[i].position-f.truth[i].position).squaredNorm(); ++n;
      }
      lead.push_back((i+1)*config.prediction_step); rmse.push_back(std::sqrt(sum/n));
    }
    for(const auto& f:e.forecasts) if(f.model==model && f.status=="complete") { origins.push_back(f.origin); ade.push_back(f.ade); }
    const std::string color=model==PredictionModel::CV?"tab:blue":"tab:orange";
    plt::subplot(1,2,1); line(lead,rmse,name(model),color);
    axes("Complete-horizon prediction error","Prediction lead [s]","3D position RMSE [m]");
    plt::subplot(1,2,2); line(origins,ade,name(model),color);
    axes("Position-only prediction accuracy","Origin time [s]","ADE [m]");
  }
  plt::subplots_adjust({{"wspace",0.3},{"bottom",0.15}}); finish(directory,"prediction_errors.png");
}
} // namespace uav

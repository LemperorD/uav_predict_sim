// SPDX-License-Identifier: GPL-3.0-only
#include "uav_predict_sim/prediction/predictor.hpp"
#include <Eigen/QR>
#include <cmath>
#include <stdexcept>

namespace uav {
const char* name(PredictionModel model) { return model==PredictionModel::CV?"cv":"ca"; }
std::vector<Observation> predict(const std::vector<Observation>& history,
    PredictionModel model,double period,int steps) {
  const int degree=model==PredictionModel::CV?1:2;
  if(history.size()<static_cast<std::size_t>(degree+1) || !std::isfinite(period) || period<=0 || steps<1)
    throw std::invalid_argument("invalid prediction window or horizon");
  for(std::size_t i=0;i<history.size();++i)
    if(!std::isfinite(history[i].time) || !history[i].position.allFinite() ||
       (i>0 && history[i].time<=history[i-1].time)) throw std::invalid_argument("invalid history");
  const double origin=history.back().time, scale=origin-history.front().time;
  Eigen::MatrixXd a(history.size(),degree+1), b(history.size(),3);
  for(std::size_t i=0;i<history.size();++i) {
    const double t=(history[i].time-origin)/scale;
    a(i,0)=1; a(i,1)=t; if(degree==2) a(i,2)=t*t/2;
    b.row(i)=history[i].position.transpose();
  }
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(a);
  if(qr.rank()!=degree+1) throw std::invalid_argument("rank deficient history");
  const Eigen::MatrixXd coefficients=qr.solve(b);
  std::vector<Observation> result;
  result.reserve(steps);
  for(int i=1;i<=steps;++i) {
    const double t=i*period/scale;
    Vec3 position=coefficients.row(0).transpose()+t*coefficients.row(1).transpose();
    if(degree==2) position+=t*t/2*coefficients.row(2).transpose();
    result.push_back({origin+i*period,position});
  }
  return result;
}
} // namespace uav

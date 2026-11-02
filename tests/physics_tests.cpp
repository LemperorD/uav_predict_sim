#include "uav_predict_sim/dynamics/quadrotor.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

void check(bool value, const char* text) { if(!value) throw std::runtime_error(text); }
int main() {
  try {
    using namespace uav;
    VehicleParams p; Quadrotor quad(p);
    const auto hover=allocate({p.mass*p.gravity, Vec3::Zero()},p);
    check(!hover.saturated,"hover allocation");
    State s; s.position.z()=2; s.rpm=hover.commanded_rpm;
    const State initial=s;
    for(int i=0;i<5000;++i) s=quad.step(s,hover.commanded_rpm,0.001);
    check((s.position-initial.position).norm()<1e-10 && s.velocity.norm()<1e-10,"hover drift");
    check(s.omega.norm()<1e-10,"hover attitude drift");
    for(int axis=0;axis<3;++axis) {
      Wrench w{p.mass*p.gravity,Vec3::Zero()}; w.torque[axis]=0.01;
      const auto a=allocate(w,p);
      check(!a.saturated && (a.achievable.vector()-w.vector()).norm()<1e-12,"mixer inverse/sign");
      State test=initial; test.rpm=a.commanded_rpm;
      test=quad.step(test,a.commanded_rpm,0.001);
      check(test.omega[axis]>0,"positive torque angular response");
    }
    const Vec4 command=hover.commanded_rpm*1.1;
    s=initial;
    for(int i=0;i<100;++i) s=quad.step(s,command,0.001);
    const Vec4 expected=command+(initial.rpm-command)*std::exp(-0.1/p.motor_tau);
    check((s.rpm-expected).norm()<1e-5,"motor exponential response");
    check(s.position.z()>initial.position.z(),"collective thrust must climb");
    State tilted=initial;
    tilted.attitude=Eigen::AngleAxisd(0.2,Vec3::UnitY());
    check(quad.acceleration(tilted).x()>0,"tilt must produce horizontal acceleration");
    VehicleParams free=p; free.min_rpm=0; Quadrotor free_quad(free);
    State falling; falling.position.z()=10;
    for(int i=0;i<1000;++i) falling=free_quad.step(falling,Vec4::Zero(),0.001);
    check(std::abs(falling.position.z()-(10-0.5*p.gravity))<1e-10,"free fall analytic position");
    State rotating; rotating.position.z()=100; rotating.omega=Vec3(0.1,0.2,0.3);
    State coarse=rotating, fine=rotating, exact=rotating;
    for(int i=0;i<100;++i) coarse=free_quad.step(coarse,Vec4::Zero(),0.002);
    for(int i=0;i<200;++i) fine=free_quad.step(fine,Vec4::Zero(),0.001);
    for(int i=0;i<400;++i) exact=free_quad.step(exact,Vec4::Zero(),0.0005);
    check((coarse.rotation().transpose()*coarse.rotation()-Mat3::Identity()).norm()<1e-12,"SO3 orthogonality");
    check(std::abs(coarse.rotation().determinant()-1)<1e-12,"SO3 determinant");
    check((fine.rotation()-exact.rotation()).norm()<1e-11,"rotation convergence");
    const auto impossible=allocate({-1,Vec3::Zero()},p);
    check(impossible.saturated && (impossible.raw_squared_rpm.array()<0).all(),"negative allocation visibility");
    std::cout<<"Physics numerical checks passed\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}

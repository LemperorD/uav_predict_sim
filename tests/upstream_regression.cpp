#include "uav_predict_sim/control/so3.hpp"
#include "uav_predict_sim/trajectory/reference.hpp"
#include <quadrotor_simulator/Quadrotor.h>
#include <so3_control/SO3Control.h>
#include <iostream>
#include <stdexcept>

void check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
int main() {
  try {
    using namespace uav;
    VehicleParams p; p.drag=0.1*3.14159265*p.arm*p.arm;
    Quadrotor model(p);
    State s; s.position=Vec3(0.1,-0.2,3); s.velocity=Vec3(0.5,-0.2,0.1);
    s.omega=Vec3(0.1,0.2,0.3); s.rpm=allocate({p.mass*p.gravity,Vec3(0.001,-0.002,0.0001)},p).commanded_rpm;
    const auto target=reference({"eight",1,0.3},0.4);
    SO3Control original_control;
    Gains g;
    original_control.setMass(p.mass); original_control.setGravity(p.gravity);
    original_control.setPosition(s.position); original_control.setVelocity(s.velocity);
    original_control.setAcc(model.acceleration(s));
    original_control.calculateControl(target.position,target.velocity,target.acceleration,
        target.heading,target.heading_rate,g.position,g.velocity);
    const auto compatible=control(s,target,s.rpm,model,g,{},true);
    check((compatible.force_world-original_control.getComputedForce()).norm()<1e-12,"upstream force mismatch");
    check((compatible.desired.rotation-original_control.getComputedOrientation().toRotationMatrix()).norm()<1e-12,
          "upstream attitude mismatch");

    QuadrotorSimulator::Quadrotor original;
    original.setExternalForce(Vec3::Zero()); original.setExternalMoment(Vec3::Zero());
    auto os=original.getState();
    os.x=s.position; os.v=s.velocity; os.R=s.rotation(); os.omega=s.omega; os.motor_rpm=s.rpm;
    original.setState(os); original.setInput(s.rpm[0],s.rpm[1],s.rpm[2],s.rpm[3]);
    const Vec4 held=s.rpm;
    for(int i=0;i<20;++i) { original.step(0.0005); s=model.step(s,held,0.0005); }
    os=original.getState();
    const double position_error=(s.position-os.x).norm(), rotation_error=(s.rotation()-os.R).norm();
    std::cout<<"upstream 10ms position difference="<<position_error<<", rotation="<<rotation_error<<'\n';
    check(position_error<1e-7 && (s.velocity-os.v).norm()<1e-7 && rotation_error<1e-7 &&
          (s.omega-os.omega).norm()<1e-7,"upstream short-time dynamics mismatch");
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}

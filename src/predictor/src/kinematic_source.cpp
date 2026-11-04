#include <rclcpp/rclcpp.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fstream>
#include <sstream>
#include <array>
#include <cmath>
#include <algorithm>
#include <thread>

using Row = std::array<double,10>; // time, position xyz, velocity xyz, acceleration xyz
class Source : public rclcpp::Node {
 public:
  Source() : Node("kinematic_source") {
    rate_=declare_parameter("sample_rate",50.0);
    duration_=declare_parameter("duration",30.0);
    trajectory_=declare_parameter<std::string>("trajectory","figure8");
    auto input=declare_parameter<std::string>("input_csv","");
    if(!std::isfinite(rate_) || rate_<=0 || !std::isfinite(duration_) || duration_<=0)
      throw std::invalid_argument("sample_rate and duration must be positive");
    if(trajectory_!="figure8" && trajectory_!="straight" && trajectory_!="acceleration")
      throw std::invalid_argument("trajectory must be figure8, straight or acceleration");
    if(!input.empty()) {
      std::ifstream file(input); if(!file) throw std::runtime_error("cannot read input_csv: "+input);
      std::string line; std::getline(file,line);
      if(line!="t,px,py,pz,vx,vy,vz,ax,ay,az") throw std::runtime_error("invalid input_csv header");
      while(std::getline(file,line)) {
        std::replace(line.begin(),line.end(),',',' '); std::istringstream in(line); Row r;
        for(double& v:r) if(!(in>>v) || !std::isfinite(v)) throw std::runtime_error("invalid CSV value");
        std::string extra; if(in>>extra) throw std::runtime_error("too many CSV columns");
        if(r[0]<0 || (!rows_.empty() && r[0]<=rows_.back()[0])) throw std::runtime_error("CSV times must increase");
        rows_.push_back(r);
      }
      if(rows_.size()<2 || rows_.front()[0]!=0 || rows_.back()[0]<duration_)
        throw std::runtime_error("CSV must cover [0,duration] with at least two rows");
    }
    pub_=create_publisher<quadrotor_msgs::msg::PositionCommand>("command",10);
    done_=create_publisher<std_msgs::msg::Bool>("simulation_done",rclcpp::QoS(1).transient_local());
    start_=std::chrono::steady_clock::now();
    timer_=create_wall_timer(std::chrono::duration<double>(1/rate_),[this]{tick();});
  }
  bool finished() const {return finished_;}
 private:
  Row sample(double t) {
    if(!rows_.empty()) {
      auto hi=std::upper_bound(rows_.begin(),rows_.end(),t,[](double v,const Row& r){return v<r[0];});
      if(hi==rows_.end()) return rows_.back();
      auto lo=hi-1; double a=(t-(*lo)[0])/((*hi)[0]-(*lo)[0]); Row r;
      for(int i=0;i<10;++i) r[i]=(*lo)[i]*(1-a)+(*hi)[i]*a;
      return r;
    }
    if(trajectory_=="straight") return {t,t,0.4*t,2,1,0.4,0,0,0,0};
    if(trajectory_=="acceleration") return {t,0.1*t*t,0.2*t,2,0.2*t,0.2,0,0.2,0,0};
    return {t,4*sin(.35*t),3*sin(.7*t),2+.6*sin(.5*t),
      1.4*cos(.35*t),2.1*cos(.7*t),.3*cos(.5*t),
      -.49*sin(.35*t),-1.47*sin(.7*t),-.15*sin(.5*t)};
  }
  void tick() {
    double t=std::chrono::duration<double>(std::chrono::steady_clock::now()-start_).count();
    if(t>=duration_) {
      std_msgs::msg::Bool b; b.data=true; done_->publish(b); finished_=true; timer_->cancel(); return;
    }
    auto r=sample(t); quadrotor_msgs::msg::PositionCommand c;
    c.header.stamp=now(); c.header.frame_id="world";
    c.position.x=r[1]; c.position.y=r[2]; c.position.z=r[3];
    c.velocity.x=r[4]; c.velocity.y=r[5]; c.velocity.z=r[6];
    c.acceleration.x=r[7]; c.acceleration.y=r[8]; c.acceleration.z=r[9];
    c.yaw=atan2(r[5],r[4]); pub_->publish(c);
  }
  double rate_,duration_; bool finished_=false; std::string trajectory_; std::vector<Row> rows_;
  std::chrono::steady_clock::time_point start_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_;
};
int main(int argc,char**argv) {
  rclcpp::init(argc,argv);
  try {auto node=std::make_shared<Source>(); while(rclcpp::ok()&&!node->finished()) {rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(1));}}
  catch(const std::exception& e) {RCLCPP_ERROR(rclcpp::get_logger("source"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown(); return 0;
}

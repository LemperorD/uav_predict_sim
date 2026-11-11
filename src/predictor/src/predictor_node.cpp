#include <algorithm>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "core/diagnostics.hpp"
#include "predictor/factory.hpp"

class PredictionNode : public rclcpp::Node
{
 public:
  PredictionNode() : Node("predictor_node")
  {
    predictor::Settings s;
    s.position_std          = declare_parameter("position_noise_std", 0.15);
    s.velocity_std          = declare_parameter("velocity_noise_std", 0.1);
    s.cv_accel_density      = declare_parameter("cv_accel_density", 0.3);
    s.ca_jerk_density       = declare_parameter("ca_jerk_density", 1.0);
    s.stay_probability      = declare_parameter("stay_probability", 0.98);
    s.attitude_std          = declare_parameter("attitude_noise_std", 0.05);
    s.angular_velocity_std  = declare_parameter("angular_velocity_noise_std", 0.1);
    s.angular_accel_density = declare_parameter("angular_accel_density", 0.3);
    s.models = declare_parameter<std::vector<std::string>>("motion_models", {"cv", "ca"});
    const auto transition =
        declare_parameter<std::vector<double>>("model_transition", std::vector<double>{});
    const auto initial =
        declare_parameter<std::vector<double>>("model_initial_probability", std::vector<double>{});
    const int n = s.models.size();
    if (!transition.empty())
    {
      if (transition.size() != static_cast<size_t>(n * n))
      {
        throw std::invalid_argument("model_transition must have N*N entries in row order");
      }
      s.transition.resize(n, n);
      for (int i = 0; i < n; ++i)
      {
        for (int j = 0; j < n; ++j)
        {
          s.transition(i, j) = transition[i * n + j];
        }
      }
    }
    if (!initial.empty())
    {
      s.initial_probability = Eigen::Map<const Eigen::VectorXd>(initial.data(), initial.size());
    }
    use_attitude_         = declare_parameter("use_attitude", true);
    use_angular_velocity_ = declare_parameter("use_angular_velocity", false);
    if (use_angular_velocity_ && !use_attitude_)
    {
      throw std::invalid_argument("use_angular_velocity requires use_attitude");
    }
    const auto algorithm = declare_parameter<std::string>("prediction_algorithm", "imm");
    predictor::PoseKFSettings pose_settings;
    pose_settings.process_variance     = declare_parameter("pose_kf_process_variance", 0.01);
    pose_settings.measurement_variance = declare_parameter("pose_kf_measurement_variance", 0.1);
    pose_settings.gravity              = declare_parameter("pose_kf_gravity", 9.81);
    pose_settings.world_up_sign        = declare_parameter("pose_kf_world_up_sign", 1.0);
    filter_   = predictor::makePredictor(algorithm, s, use_attitude_, pose_settings);
    settings_ = s;

    noise_ = declare_parameter("inject_noise", true);
    rng_.seed(declare_parameter("noise_seed", 42));

    step_    = declare_parameter("prediction_step", 0.05);
    count_   = declare_parameter("prediction_steps", 20);
    max_gap_ = declare_parameter("max_time_gap", 1.0);
    if (!std::isfinite(step_) || step_ <= 0 || count_ <= 0 || count_ > 10000 ||
        !std::isfinite(max_gap_) || max_gap_ <= 0)
    {
      throw std::invalid_argument("invalid prediction step/count or max_time_gap");
    }

    auto dir = std::filesystem::path(declare_parameter<std::string>("output_dir", "outputs/demo"));
    std::filesystem::create_directories(dir);
    auto sample_path =
        std::filesystem::path(declare_parameter<std::string>("samples_file", "samples.csv"));
    auto prediction_path = std::filesystem::path(
        declare_parameter<std::string>("predictions_file", "predictions.csv"));
    if (sample_path.is_relative())
    {
      sample_path = dir / sample_path;
    }
    if (prediction_path.is_relative())
    {
      prediction_path = dir / prediction_path;
    }
    if (!sample_path.parent_path().empty())
    {
      std::filesystem::create_directories(sample_path.parent_path());
    }
    if (!prediction_path.parent_path().empty())
    {
      std::filesystem::create_directories(prediction_path.parent_path());
    }

    samples_.open(sample_path);
    predictions_.open(prediction_path);
    if (!samples_ || !predictions_)
    {
      throw std::runtime_error("cannot open output CSV files");
    }

    samples_ << std::setprecision(12);
    predictions_ << std::setprecision(12);
    samples_ << "t,truth_x,truth_y,truth_z,measurement_x,measurement_y,measurement_"
                "z,estimate_x,estimate_y,estimate_z,std_x,std_y,std_z,p_cv,p_ca,"
                "position_error,horizon_error,velocity_error,position_nees";
    for (const auto &name : settings_.models)
    {
      if (name != "cv" && name != "ca")
      {
        samples_ << ",p_" << name;
      }
    }
    samples_ << ",truth_vx,truth_vy,truth_vz,measurement_vx,measurement_vy,measurement_vz,"
                "estimate_vx,estimate_vy,estimate_vz,measurement_qw,measurement_qx,measurement_qy,"
                "measurement_qz\n";
    predictions_ << "origin_t,target_t,pred_x,pred_y,pred_z,truth_x,truth_y,"
                    "truth_z,error,std_x,std_y,std_z\n";

    estimate_pub_ = create_publisher<nav_msgs::msg::Odometry>("estimate", 10);
    path_pub_     = create_publisher<nav_msgs::msg::Path>("prediction", 10);
    probability_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>("model_probabilities", 10);
    metrics_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("metrics", 10);
    done_        = create_subscription<std_msgs::msg::Bool>("simulation_done",
                                                     rclcpp::QoS(1).transient_local(),
                                                     [this](const std_msgs::msg::Bool &message)
                                                     {
                                                       if (message.data)
                                                       {
                                                         finished_ = true;
                                                         samples_.flush();
                                                         predictions_.flush();
                                                       }
                                                     });
    sub_         = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", rclcpp::QoS(50),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { receive(*m); });
  }

 private:
  struct Pending
  {
    double          origin, target;
    Eigen::Vector3d p, std;
  };

  void receive(const nav_msgs::msg::Odometry &m)
  {
    if (finished_)
    {
      return;
    }

    const double stamp = rclcpp::Time(m.header.stamp).seconds();
    if (!std::isfinite(stamp))
    {
      return;
    }

    Eigen::Matrix<double, 6, 1> truth;  // ROS position/velocity observations, not algorithm state.
    truth << m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z,
        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z;
    if (!truth.allFinite())
    {
      RCLCPP_WARN(get_logger(), "Skipping nonfinite odometry");
      return;
    }

    if (start_stamp_ < 0)
    {
      start_stamp_ = stamp;
    }
    double t = stamp - start_stamp_, dt = last_stamp_ < 0 ? 0.01 : stamp - last_stamp_;
    if (dt <= 0)
    {
      RCLCPP_WARN(get_logger(), "Skipping duplicate/out-of-order stamp");
      return;
    }
    if (dt > max_gap_)
    {
      filter_->reset();
      pending_.clear();
      RCLCPP_WARN(get_logger(), "Reset predictor after time gap %.3f", dt);
    }

    Eigen::Matrix<double, 6, 1> z = truth;
    if (noise_)
    {
      for (int i = 0; i < 6; ++i)
      {
        z(i) += normal_(rng_) * (i < 3 ? settings_.position_std : settings_.velocity_std);
      }
    }

    Eigen::Quaterniond             orientation(m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                                               m.pose.pose.orientation.y, m.pose.pose.orientation.z);
    std::optional<Eigen::Vector3d> omega;
    if (use_angular_velocity_)
    {
      omega = Eigen::Vector3d(m.twist.twist.angular.x, m.twist.twist.angular.y,
                              m.twist.twist.angular.z);
    }
    if ((use_attitude_ && (!orientation.coeffs().allFinite() ||
                           !std::isfinite(orientation.norm()) || orientation.norm() < 1e-12)) ||
        (omega && !omega->allFinite()))
    {
      RCLCPP_WARN(get_logger(), "Skipping invalid attitude/angular velocity");
      return;
    }
    if (use_attitude_)
    {
      orientation.normalize();
      if (noise_)
      {
        Eigen::Vector3d rotation_noise;
        for (int k = 0; k < 3; ++k)
        {
          rotation_noise(k) = normal_(rng_) * settings_.attitude_std;
          if (omega)
          {
            (*omega)(k) += normal_(rng_) * settings_.angular_velocity_std;
          }
        }
        const double angle = rotation_noise.norm();
        if (angle > 0)
        {
          orientation =
              orientation * Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_noise / angle));
        }
      }
    }
    predictor::Observation observation;
    observation.timestamp = stamp;
    observation.position  = z.head<3>();
    observation.velocity  = z.tail<3>();
    if (use_attitude_)
    {
      observation.orientation      = orientation;
      observation.angular_velocity = omega;
    }
    filter_->update(observation);

    double horizon_error = std::numeric_limits<double>::quiet_NaN();
    while (!pending_.empty() && pending_.front().target <= t)
    {
      auto p = pending_.front();
      pending_.pop_front();
      if (last_stamp_ < 0 || p.target < last_stamp_ - start_stamp_)
      {
        continue;
      }
      double          a      = (p.target - (last_stamp_ - start_stamp_)) / dt;
      Eigen::Vector3d target = (1 - a) * last_truth_ + a * truth.head<3>();
      horizon_error          = (p.p - target).norm();
      predictions_ << p.origin << ',' << p.target;
      for (double v : p.p)
      {
        predictions_ << ',' << v;
      }
      for (double v : target)
      {
        predictions_ << ',' << v;
      }
      predictions_ << ',' << horizon_error;
      for (double v : p.std)
      {
        predictions_ << ',' << v;
      }
      predictions_ << '\n';
    }

    const auto future = filter_->predict(step_, count_);
    if (future.size() != static_cast<size_t>(count_) ||
        !std::all_of(future.begin(), future.end(), [](const auto &p) { return p.allFinite(); }))
    {
      throw std::runtime_error(
          "Predictor must return exactly prediction_steps finite XYZ positions");
    }
    predictor::Diagnostics diagnostic;
    if (const auto *provider = dynamic_cast<const predictor::DiagnosticsProvider *>(filter_.get()))
    {
      diagnostic = provider->diagnostics();
    }
    Eigen::Vector3d forecast_std = Eigen::Vector3d::Constant(predictor::Diagnostics::unavailable);
    if (diagnostic.forecast_endpoint_covariance)
    {
      forecast_std = diagnostic.forecast_endpoint_covariance->diagonal().cwiseMax(0).cwiseSqrt();
    }
    pending_.push_back({t, t + step_ * count_, future.back(), forecast_std});

    // Estimated state is optional diagnostics, independent of the position-only prediction output.
    if (diagnostic.position.allFinite())
    {
      nav_msgs::msg::Odometry odom;
      odom.header               = m.header;
      odom.child_frame_id       = "prediction";
      odom.pose.pose.position.x = diagnostic.position.x();
      odom.pose.pose.position.y = diagnostic.position.y();
      odom.pose.pose.position.z = diagnostic.position.z();
      const auto q              = diagnostic.orientation.value_or(Eigen::Quaterniond::Identity());
      odom.pose.pose.orientation.w = q.w();
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.twist.twist.linear.x    = diagnostic.velocity.x();
      odom.twist.twist.linear.y    = diagnostic.velocity.y();
      odom.twist.twist.linear.z    = diagnostic.velocity.z();
      odom.twist.twist.angular.x   = diagnostic.angular_velocity.x();
      odom.twist.twist.angular.y   = diagnostic.angular_velocity.y();
      odom.twist.twist.angular.z   = diagnostic.angular_velocity.z();
      for (int i = 0; i < 6; ++i)
      {
        for (int j = 0; j < 6; ++j)
        {
          odom.pose.covariance[6 * i + j]  = diagnostic.pose_covariance
                                                 ? (*diagnostic.pose_covariance)(i, j)
                                                 : predictor::Diagnostics::unavailable;
          odom.twist.covariance[6 * i + j] = diagnostic.twist_covariance
                                                 ? (*diagnostic.twist_covariance)(i, j)
                                                 : predictor::Diagnostics::unavailable;
        }
      }
      estimate_pub_->publish(odom);
    }

    nav_msgs::msg::Path path;
    path.header = m.header;
    for (int i = 0; i < count_; ++i)
    {
      geometry_msgs::msg::PoseStamped p;
      p.header = m.header;
      p.header.stamp =
          rclcpp::Time(m.header.stamp) + rclcpp::Duration::from_seconds((i + 1) * step_);
      p.pose.position.x = future[i].x();
      p.pose.position.y = future[i].y();
      p.pose.position.z = future[i].z();
      // Path requires PoseStamped; identity is a neutral placeholder, not predicted attitude.
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }
    path_pub_->publish(path);

    const Eigen::Vector3d error  = diagnostic.position - truth.head<3>();
    Eigen::Vector3d position_std = Eigen::Vector3d::Constant(predictor::Diagnostics::unavailable);
    double          nees         = predictor::Diagnostics::unavailable;
    if (diagnostic.pose_covariance)
    {
      const Eigen::Matrix3d covariance = diagnostic.pose_covariance->topLeftCorner<3, 3>();
      position_std                     = covariance.diagonal().cwiseMax(0).cwiseSqrt();
      nees                             = error.dot(covariance.ldlt().solve(error));
    }

    std_msgs::msg::Float64MultiArray metrics;
    metrics.data.push_back(t);
    for (double v : truth.head<3>())
    {
      metrics.data.push_back(v);
    }
    for (double v : z.head<3>())
    {
      metrics.data.push_back(v);
    }
    for (double v : diagnostic.position)
    {
      metrics.data.push_back(v);
    }
    for (double v : position_std)
    {
      metrics.data.push_back(v);
    }
    const auto probability = [&](const std::string &name)
    {
      if (diagnostic.model_probabilities.empty())
      {
        return predictor::Diagnostics::unavailable;
      }
      for (const auto &entry : diagnostic.model_probabilities)
      {
        if (entry.name == name)
        {
          return entry.probability;
        }
      }
      return 0.0;
    };
    metrics.data.push_back(probability("cv"));
    metrics.data.push_back(probability("ca"));
    metrics.data.push_back(error.norm());
    metrics.data.push_back(horizon_error);
    metrics.data.push_back((diagnostic.velocity - truth.tail<3>()).norm());
    metrics.data.push_back(nees);
    for (const auto &name : settings_.models)
    {
      if (name != "cv" && name != "ca")
      {
        metrics.data.push_back(probability(name));
      }
    }
    metrics_pub_->publish(metrics);
    std_msgs::msg::Float64MultiArray probabilities;
    probabilities.layout.dim.resize(1);
    auto &dimension = probabilities.layout.dim[0];
    for (const auto &entry : diagnostic.model_probabilities)
    {
      if (!dimension.label.empty())
      {
        dimension.label += ",";
      }
      dimension.label += entry.name;
      probabilities.data.push_back(entry.probability);
    }
    dimension.size   = probabilities.data.size();
    dimension.stride = dimension.size;
    probability_pub_->publish(probabilities);

    for (size_t i = 0; i < metrics.data.size(); ++i)
    {
      if (i)
      {
        samples_ << ',';
      }
      samples_ << metrics.data[i];
    }
    for (double v : truth.tail<3>()) samples_ << ',' << v;
    for (double v : z.tail<3>()) samples_ << ',' << v;
    for (double v : diagnostic.velocity) samples_ << ',' << v;
    if (observation.orientation)
      samples_ << ',' << orientation.w() << ',' << orientation.x() << ',' << orientation.y() << ','
               << orientation.z();
    else
      samples_ << ",nan,nan,nan,nan";
    samples_ << '\n';
    if (++received_ % 100 == 0)
    {
      samples_.flush();
      predictions_.flush();
    }

    last_truth_ = truth.head<3>();
    last_stamp_ = stamp;
  }

  predictor::Settings                                  settings_;
  std::unique_ptr<predictor::Predictor>                filter_;
  bool                                                 noise_;
  bool                                                 use_attitude_, use_angular_velocity_;
  bool                                                 finished_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr done_;
  std::mt19937                                         rng_;
  std::normal_distribution<double>                     normal_;
  double              step_, max_gap_, start_stamp_ = -1, last_stamp_ = -1;
  int                 count_, received_ = 0;
  Eigen::Vector3d     last_truth_ = Eigen::Vector3d::Zero();
  std::deque<Pending> pending_;
  std::ofstream       samples_, predictions_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          estimate_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr probability_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr metrics_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<PredictionNode>());
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("predictor"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
  return 0;
}

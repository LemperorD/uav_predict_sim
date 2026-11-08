#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include <vector>

#include "predictor/trajectory.hpp"

using Row = predictor::TrajectorySample;

class Source : public rclcpp::Node
{
   public:
    Source() : Node("kinematic_source")
    {
        rate_          = declare_parameter("sample_rate", 50.0);
        duration_      = declare_parameter("duration", 30.0);
        settings_.type = declare_parameter<std::string>("trajectory", "figure8");

        auto vector_parameter = [this](const std::string &name, predictor::Vector3 defaults)
        {
            auto values = declare_parameter<std::vector<double>>(
                name, std::vector<double>(defaults.begin(), defaults.end()));
            if (values.size() != 3)
            {
                throw std::invalid_argument(name + " must contain exactly three values");
            }
            return predictor::Vector3{values[0], values[1], values[2]};
        };

        settings_.origin   = vector_parameter("trajectory_origin", settings_.origin);
        settings_.velocity = vector_parameter(
            "trajectory_velocity",
            settings_.type == "acceleration" ? predictor::Vector3{0, 0.2, 0} : settings_.velocity);
        settings_.acceleration =
            vector_parameter("trajectory_acceleration", settings_.acceleration);
        settings_.amplitude = vector_parameter("trajectory_amplitude", settings_.amplitude);
        settings_.frequency = vector_parameter("trajectory_frequency", settings_.frequency);
        settings_.phase     = vector_parameter("trajectory_phase", settings_.phase);
        settings_.radius    = declare_parameter("trajectory_radius", settings_.radius);
        settings_.angular_speed =
            declare_parameter("trajectory_angular_speed", settings_.angular_speed);
        settings_.circle_phase =
            declare_parameter("trajectory_circle_phase", settings_.circle_phase);
        settings_.vertical_speed =
            declare_parameter("trajectory_vertical_speed", settings_.vertical_speed);

        auto input = declare_parameter<std::string>("input_csv", "");
        if (!std::isfinite(rate_) || rate_ <= 0 || !std::isfinite(duration_) || duration_ <= 0)
        {
            throw std::invalid_argument("sample_rate and duration must be positive");
        }

        // CSV overrides the built-in trajectory selection.
        if (input.empty())
        {
            settings_.validate();
        }

        if (!input.empty())
        {
            std::ifstream file(input);
            if (!file)
            {
                throw std::runtime_error("cannot read input_csv: " + input);
            }

            std::string line;
            std::getline(file, line);
            if (line != "t,px,py,pz,vx,vy,vz,ax,ay,az")
            {
                throw std::runtime_error("invalid input_csv header");
            }

            while (std::getline(file, line))
            {
                std::replace(line.begin(), line.end(), ',', ' ');
                std::istringstream in(line);
                Row                r;
                for (double &v : r)
                {
                    if (!(in >> v) || !std::isfinite(v))
                    {
                        throw std::runtime_error("invalid CSV value");
                    }
                }

                std::string extra;
                if (in >> extra)
                {
                    throw std::runtime_error("too many CSV columns");
                }
                if (r[0] < 0 || (!rows_.empty() && r[0] <= rows_.back()[0]))
                {
                    throw std::runtime_error("CSV times must increase");
                }
                rows_.push_back(r);
            }

            if (rows_.size() < 2 || rows_.front()[0] != 0 || rows_.back()[0] < duration_)
            {
                throw std::runtime_error("CSV must cover [0,duration] with at least two rows");
            }
        }

        pub_  = create_publisher<quadrotor_msgs::msg::PositionCommand>("command", 10);
        done_ = create_publisher<std_msgs::msg::Bool>("simulation_done",
                                                      rclcpp::QoS(1).transient_local());

        start_ = std::chrono::steady_clock::now();
        timer_ = create_wall_timer(std::chrono::duration<double>(1 / rate_), [this] { tick(); });
    }

    bool finished() const { return finished_; }

   private:
    Row sample(double t)
    {
        if (!rows_.empty())
        {
            auto hi = std::upper_bound(rows_.begin(), rows_.end(), t,
                                       [](double v, const Row &r) { return v < r[0]; });
            if (hi == rows_.end())
            {
                return rows_.back();
            }

            auto   lo = hi - 1;
            double a  = (t - (*lo)[0]) / ((*hi)[0] - (*lo)[0]);
            Row    r;
            for (int i = 0; i < 10; ++i)
            {
                r[i] = (*lo)[i] * (1 - a) + (*hi)[i] * a;
            }
            return r;
        }

        return predictor::SampleTrajectory(settings_, t);
    }

    void tick()
    {
        double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
        if (t >= duration_)
        {
            std_msgs::msg::Bool b;
            b.data = true;
            done_->publish(b);
            done_->wait_for_all_acked(std::chrono::seconds(1));
            finished_ = true;
            timer_->cancel();
            return;
        }

        auto                                 r = sample(t);
        quadrotor_msgs::msg::PositionCommand c;
        c.header.stamp    = now();
        c.header.frame_id = "world";

        c.position.x = r[1];
        c.position.y = r[2];
        c.position.z = r[3];

        c.velocity.x = r[4];
        c.velocity.y = r[5];
        c.velocity.z = r[6];

        c.acceleration.x = r[7];
        c.acceleration.y = r[8];
        c.acceleration.z = r[9];

        c.yaw = atan2(r[5], r[4]);
        pub_->publish(c);
    }

    double                                                             rate_, duration_;
    bool                                                               finished_ = false;
    predictor::TrajectorySettings                                      settings_;
    std::vector<Row>                                                   rows_;
    std::chrono::steady_clock::time_point                              start_;
    rclcpp::TimerBase::SharedPtr                                       timer_;
    rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                  done_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<Source>();
        while (rclcpp::ok() && !node->finished())
        {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("source"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

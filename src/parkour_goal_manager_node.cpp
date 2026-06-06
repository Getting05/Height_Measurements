#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace height_measurements {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct GoalPoint {
  double x = 0.0;
  double y = 0.0;
};

struct GoalConfig {
  std::string frame_id = "map";
  std::vector<GoalPoint> goals;
};

struct PoseState {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::string frame_id;
};

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double wrap_to_pi(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

GoalConfig load_goal_config(const std::string &path) {
  const YAML::Node root = YAML::LoadFile(path);
  GoalConfig config;
  if (root["frame_id"]) {
    config.frame_id = root["frame_id"].as<std::string>();
  }

  const YAML::Node goals = root["goals"];
  if (!goals || !goals.IsSequence()) {
    throw std::runtime_error("goal YAML must contain a sequence field: goals");
  }

  config.goals.reserve(goals.size());
  for (std::size_t i = 0; i < goals.size(); ++i) {
    const YAML::Node goal = goals[i];
    if (!goal.IsSequence() || goal.size() < 2) {
      throw std::runtime_error("each goal must be a sequence with at least x/y");
    }
    GoalPoint point;
    point.x = goal[0].as<double>();
    point.y = goal[1].as<double>();
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::runtime_error("goal coordinates must be finite");
    }
    config.goals.push_back(point);
  }

  if (config.goals.empty()) {
    throw std::runtime_error("goal YAML must contain at least one goal");
  }
  return config;
}

} // namespace

class ParkourGoalManagerNode : public rclcpp::Node {
public:
  ParkourGoalManagerNode() : Node("parkour_goal_manager_node") {
    goal_points_path_ = declare_parameter<std::string>("goal_points_path", "");
    odom_topic_ =
        declare_parameter<std::string>("odom_topic", "/aft_mapped_in_map");
    goal_yaw_topic_ =
        declare_parameter<std::string>("goal_yaw_topic", "/parkour/goal_yaw");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    goal_reach_threshold_ =
        declare_parameter<double>("goal_reach_threshold", 0.2);
    reach_goal_delay_ = declare_parameter<double>("reach_goal_delay", 0.1);
    base_to_odom_x_ = declare_parameter<double>("base_to_odom_x", 0.16266);
    base_to_odom_y_ = declare_parameter<double>("base_to_odom_y", 0.0);
    base_to_odom_z_ = declare_parameter<double>("base_to_odom_z", 0.11703);

    if (goal_points_path_.empty()) {
      throw std::runtime_error("goal_points_path parameter is required");
    }
    if (publish_rate_ <= 0.0) {
      throw std::runtime_error("publish_rate must be positive");
    }
    if (goal_reach_threshold_ < 0.0) {
      throw std::runtime_error("goal_reach_threshold must be non-negative");
    }
    if (reach_goal_delay_ < 0.0) {
      throw std::runtime_error("reach_goal_delay must be non-negative");
    }

    const GoalConfig config = load_goal_config(goal_points_path_);
    if (config.frame_id != map_frame_) {
      throw std::runtime_error("goal YAML frame_id (" + config.frame_id +
                               ") differs from map_frame parameter (" +
                               map_frame_ + ")");
    }
    goals_ = config.goals;

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, qos,
        std::bind(&ParkourGoalManagerNode::odom_callback, this,
                  std::placeholders::_1));
    goal_yaw_pub_ =
        create_publisher<std_msgs::msg::Float32MultiArray>(goal_yaw_topic_, qos);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_));
    timer_period_seconds_ = 1.0 / publish_rate_;
    timer_ =
        create_wall_timer(period, std::bind(&ParkourGoalManagerNode::on_timer, this));
    last_stats_time_ = now();

    RCLCPP_INFO(get_logger(),
                "Loaded %zu map-frame goals from %s; subscribing odom=%s, "
                "publishing goal_yaw=%s at %.2f Hz",
                goals_.size(), goal_points_path_.c_str(), odom_topic_.c_str(),
                goal_yaw_topic_.c_str(), publish_rate_);
    RCLCPP_INFO(get_logger(),
                "Goal reach threshold=%.3f m, delay=%.3f s, "
                "base_to_odom=(%.5f, %.5f, %.5f)",
                goal_reach_threshold_, reach_goal_delay_, base_to_odom_x_,
                base_to_odom_y_, base_to_odom_z_);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    PoseState pose;
    pose.yaw = quaternion_to_yaw(msg->pose.pose.orientation);
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    pose.x = msg->pose.pose.position.x -
             (c * base_to_odom_x_ - s * base_to_odom_y_);
    pose.y = msg->pose.pose.position.y -
             (s * base_to_odom_x_ + c * base_to_odom_y_);
    pose.frame_id = msg->header.frame_id;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      latest_pose_ = pose;
      have_pose_ = true;
    }

    if (!pose.frame_id.empty() && pose.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Odometry frame_id is %s, expected map_frame %s. Goal yaw assumes "
          "odometry is already expressed in the goal map frame.",
          pose.frame_id.c_str(), map_frame_.c_str());
    }
  }

  double distance_to_goal(const PoseState &pose, const GoalPoint &goal) const {
    const double dx = goal.x - pose.x;
    const double dy = goal.y - pose.y;
    return std::hypot(dx, dy);
  }

  float delta_yaw_to_goal(const PoseState &pose, const GoalPoint &goal) const {
    const double goal_yaw = std::atan2(goal.y - pose.y, goal.x - pose.x);
    return static_cast<float>(wrap_to_pi(goal_yaw - pose.yaw));
  }

  void update_goal_progress(const PoseState &pose) {
    if (all_goals_complete_) {
      return;
    }

    const GoalPoint &goal = goals_[current_goal_idx_];
    const double distance = distance_to_goal(pose, goal);
    if (distance < goal_reach_threshold_) {
      reach_goal_timer_ += timer_period_seconds_;
      if (reach_goal_timer_ >= reach_goal_delay_) {
        if (current_goal_idx_ + 1 < goals_.size()) {
          ++current_goal_idx_;
          reach_goal_timer_ = 0.0;
          RCLCPP_INFO(get_logger(), "Reached goal, switching to goal %zu/%zu",
                      current_goal_idx_ + 1, goals_.size());
        } else {
          all_goals_complete_ = true;
          reach_goal_timer_ = 0.0;
          RCLCPP_INFO(get_logger(), "All goals reached");
        }
      }
    } else {
      reach_goal_timer_ = 0.0;
    }
  }

  std::array<float, 3> compute_goal_yaw(const PoseState &pose) const {
    if (all_goals_complete_) {
      return {0.0f, 0.0f, 0.0f};
    }

    const std::size_t goal_idx =
        std::min(current_goal_idx_, goals_.size() - 1);
    const std::size_t next_idx = std::min(goal_idx + 1, goals_.size() - 1);
    return {0.0f, delta_yaw_to_goal(pose, goals_[goal_idx]),
            delta_yaw_to_goal(pose, goals_[next_idx])};
  }

  void publish_goal_yaw(const std::array<float, 3> &goal_yaw) {
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "goal_yaw";
    msg.layout.dim[0].size = goal_yaw.size();
    msg.layout.dim[0].stride = goal_yaw.size();
    msg.layout.data_offset = 0;
    msg.data.assign(goal_yaw.begin(), goal_yaw.end());
    goal_yaw_pub_->publish(msg);
  }

  void on_timer() {
    PoseState pose;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      if (!have_pose_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Waiting for odometry on %s", odom_topic_.c_str());
        return;
      }
      pose = latest_pose_;
    }

    update_goal_progress(pose);
    const std::array<float, 3> goal_yaw = compute_goal_yaw(pose);
    publish_goal_yaw(goal_yaw);

    const rclcpp::Time current = now();
    if ((current - last_stats_time_).seconds() >= 2.0) {
      if (all_goals_complete_) {
        RCLCPP_INFO(get_logger(), "goal_yaw=[0.0000, 0.0000, 0.0000], complete");
      } else {
        const double distance =
            distance_to_goal(pose, goals_[current_goal_idx_]);
        RCLCPP_INFO(get_logger(),
                    "goal %zu/%zu distance=%.3f reach_timer=%.3f "
                    "goal_yaw=[%.4f, %.4f, %.4f]",
                    current_goal_idx_ + 1, goals_.size(), distance,
                    reach_goal_timer_, goal_yaw[0], goal_yaw[1], goal_yaw[2]);
      }
      last_stats_time_ = current;
    }
  }

  std::string goal_points_path_;
  std::string odom_topic_;
  std::string goal_yaw_topic_;
  std::string map_frame_;
  double publish_rate_ = 50.0;
  double goal_reach_threshold_ = 0.2;
  double reach_goal_delay_ = 0.1;
  double base_to_odom_x_ = 0.16266;
  double base_to_odom_y_ = 0.0;
  double base_to_odom_z_ = 0.11703;
  double timer_period_seconds_ = 0.02;

  std::vector<GoalPoint> goals_;
  std::size_t current_goal_idx_ = 0;
  double reach_goal_timer_ = 0.0;
  bool all_goals_complete_ = false;

  std::mutex pose_mutex_;
  PoseState latest_pose_;
  bool have_pose_ = false;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr goal_yaw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_stats_time_;
};

} // namespace height_measurements

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<height_measurements::ParkourGoalManagerNode>());
  } catch (const std::exception &exc) {
    RCLCPP_ERROR(rclcpp::get_logger("parkour_goal_manager_node"), "%s",
                 exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

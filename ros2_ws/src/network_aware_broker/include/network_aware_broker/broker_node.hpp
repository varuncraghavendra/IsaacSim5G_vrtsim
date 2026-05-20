#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "network_aware_broker/broker_types.hpp"
#include "network_aware_broker/deterministic_log.hpp"
#include "network_aware_broker/metrics_estimator.hpp"
#include "network_aware_broker/pose_cache.hpp"
#include "network_aware_broker/telemetry_fuser.hpp"
#include "network_aware_broker/time_broker.hpp"
#include "network_aware_msgs/msg/clock_state.hpp"
#include "network_aware_msgs/msg/network_metrics.hpp"
#include "network_aware_msgs/msg/robot_telemetry.hpp"
#include "network_aware_msgs/msg/teleop_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace network_aware_broker
{

class BrokerNode : public rclcpp::Node
{
public:
  explicit BrokerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_teleop(const network_aware_msgs::msg::TeleopCommand::SharedPtr msg);
  void on_metrics(const network_aware_msgs::msg::NetworkMetrics::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_step_complete(const std_msgs::msg::Bool::SharedPtr msg);
  void on_timer();

  void publish_clock_state(double publish_age_ms);
  void log_teleop_event(const QueuedCommand & cmd);
  void log_metrics_event(const NetworkEstimate & estimate, const std::string & source);
  void log_release_event(const QueuedCommand & cmd);

  // Pipeline pieces
  TimeBroker broker_;
  MetricsEstimator metrics_estimator_;
  PoseCache pose_cache_;
  TelemetryFuser telemetry_fuser_;
  DeterministicLog log_;

  // Cached state
  NetworkEstimate estimate_;
  RobotState robot_state_;
  std::string last_metrics_source_{"none"};

  // Continuous-publish state: this is what the timer publishes every tick.
  // Decoupling the publish rate from the arrival rate is what makes the
  // robot controllable -- Isaac always sees a fresh Twist.
  QueuedCommand current_cmd_{};
  double current_cmd_set_at_{0.0};
  bool have_current_cmd_{false};
  bool estop_latched_{false};
  double max_command_hold_ms_{600.0};

  rclcpp::Subscription<network_aware_msgs::msg::TeleopCommand>::SharedPtr teleop_sub_;
  rclcpp::Subscription<network_aware_msgs::msg::NetworkMetrics>::SharedPtr metrics_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr step_complete_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr adaptive_cmd_pub_;
  rclcpp::Publisher<network_aware_msgs::msg::ClockState>::SharedPtr clock_state_pub_;
  rclcpp::Publisher<network_aware_msgs::msg::RobotTelemetry>::SharedPtr telemetry_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace network_aware_broker

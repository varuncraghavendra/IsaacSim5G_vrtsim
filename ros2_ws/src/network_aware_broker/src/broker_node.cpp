#include "network_aware_broker/broker_node.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace network_aware_broker
{

namespace
{
builtin_interfaces::msg::Time stamp_from_now(rclcpp::Node & node)
{
  const auto t = node.now();
  builtin_interfaces::msg::Time msg;
  const auto ns = t.nanoseconds();
  msg.sec = static_cast<int32_t>(ns / 1000000000LL);
  msg.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return msg;
}
}  // namespace

BrokerNode::BrokerNode(const rclcpp::NodeOptions & options)
: Node("network_aware_broker", options),
  metrics_estimator_(0.2)
{
  const auto teleop_topic = declare_parameter<std::string>(
    "teleop_topic", "/network/teleop_command");
  // NOTE: default is /network/cmd_vel (NOT /cmd_vel) so the Isaac bridge is
  // the only publisher on /cmd_vel and can apply its own watchdog cleanly.
  const auto adaptive_cmd_topic = declare_parameter<std::string>(
    "adaptive_cmd_topic", "/network/cmd_vel");
  const auto metrics_topic = declare_parameter<std::string>(
    "metrics_topic", "/network/metrics");
  const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
  const auto step_complete_topic = declare_parameter<std::string>(
    "step_complete_topic", "/network/step_complete");
  const auto clock_state_topic = declare_parameter<std::string>(
    "clock_state_topic", "/network/clock_state");
  const auto telemetry_topic = declare_parameter<std::string>(
    "telemetry_topic", "/network/robot_telemetry");
  const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 100.0);
  const auto log_csv_path = declare_parameter<std::string>("log_csv_path", "");
  const auto deterministic_replay_enabled = declare_parameter<bool>(
    "deterministic_replay_enabled", false);

  broker_.set_default_rtf(declare_parameter<double>("default_rtf", 1.0));
  broker_.set_physics_step_ms(declare_parameter<double>("physics_step_ms", 20.0));
  broker_.set_network_step_ms(declare_parameter<double>("network_step_ms", 10.0));
  broker_.set_release_lookahead_ms(declare_parameter<double>("release_lookahead_ms", 0.0));
  broker_.set_max_command_age_ms(declare_parameter<double>("max_command_age_ms", 500.0));
  broker_.set_max_queue_size(
    static_cast<std::size_t>(declare_parameter<int>("max_queue_size", 256)));
  // The step-complete gate is an experimental barrier that only makes sense
  // in deterministic-replay mode. Defaulted off so live teleop releases as
  // soon as the latency-aware deadline is met.
  broker_.set_step_complete_gate(declare_parameter<bool>("use_step_complete_gate", false));
  broker_.set_hold_last_command_ms(declare_parameter<double>("stale_hold_last_command_ms", 100.0));
  broker_.set_deterministic_replay_enabled(deterministic_replay_enabled);

  max_command_hold_ms_ = declare_parameter<double>("max_command_hold_ms", 600.0);

  pose_cache_.set_max_size(256);
  log_.open(log_csv_path);

  adaptive_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(adaptive_cmd_topic, 20);
  clock_state_pub_ = create_publisher<network_aware_msgs::msg::ClockState>(clock_state_topic, 20);
  telemetry_pub_ = create_publisher<network_aware_msgs::msg::RobotTelemetry>(telemetry_topic, 20);

  teleop_sub_ = create_subscription<network_aware_msgs::msg::TeleopCommand>(
    teleop_topic, 50, std::bind(&BrokerNode::on_teleop, this, std::placeholders::_1));
  metrics_sub_ = create_subscription<network_aware_msgs::msg::NetworkMetrics>(
    metrics_topic, 20, std::bind(&BrokerNode::on_metrics, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 20, std::bind(&BrokerNode::on_odom, this, std::placeholders::_1));
  step_complete_sub_ = create_subscription<std_msgs::msg::Bool>(
    step_complete_topic, 20,
    std::bind(&BrokerNode::on_step_complete, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&BrokerNode::on_timer, this));

  estimate_.rtf = broker_.current_rtf();
  current_cmd_set_at_ = now().seconds();

  RCLCPP_INFO(
    get_logger(),
    "broker started: teleop=%s -> %s at %.0f Hz | hold=%.0f ms | est_gate=%s",
    teleop_topic.c_str(), adaptive_cmd_topic.c_str(),
    publish_rate_hz, max_command_hold_ms_,
    broker_.step_complete_gate_enabled() ? "on" : "off");
}

void BrokerNode::on_teleop(const network_aware_msgs::msg::TeleopCommand::SharedPtr msg)
{
  QueuedCommand cmd;
  if (msg->stamp.sec == 0 && msg->stamp.nanosec == 0) {
    cmd.stamp_sec = now().seconds();
  } else {
    cmd.stamp_sec = static_cast<double>(msg->stamp.sec)
                  + 1e-9 * static_cast<double>(msg->stamp.nanosec);
  }

  cmd.seq = msg->seq;
  cmd.linear_x = msg->linear_x;
  cmd.angular_z = msg->angular_z;
  cmd.desired_rtf = msg->desired_rtf > 0.0 ? msg->desired_rtf : broker_.current_rtf();
  cmd.emergency_stop = msg->emergency_stop;
  cmd.source = msg->source;

  broker_.push_command(cmd);
  log_teleop_event(cmd);
}

void BrokerNode::on_metrics(const network_aware_msgs::msg::NetworkMetrics::SharedPtr msg)
{
  NetworkEstimate sample;
  sample.one_way_latency_ms = msg->one_way_latency_ms;
  sample.round_trip_latency_ms = msg->round_trip_latency_ms;
  sample.jitter_ms = msg->jitter_ms;
  sample.packet_error_rate = msg->packet_error_rate;
  sample.snr_db = msg->snr_db;
  sample.rtf = msg->rtf > 0.0 ? msg->rtf : broker_.current_rtf();
  sample.los = msg->los;
  sample.path_loss_db = msg->path_loss_db;

  estimate_ = metrics_estimator_.update(sample);
  broker_.update_network_estimate(estimate_);
  last_metrics_source_ = msg->source;
  log_metrics_event(estimate_, msg->source);
}

void BrokerNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_state_.x = msg->pose.pose.position.x;
  robot_state_.y = msg->pose.pose.position.y;
  robot_state_.speed = msg->twist.twist.linear.x;

  const auto & q = msg->pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  robot_state_.yaw = std::atan2(siny_cosp, cosy_cosp);

  TimedPose pose;
  pose.stamp_sec = now().seconds();
  pose.x = robot_state_.x;
  pose.y = robot_state_.y;
  pose.yaw = robot_state_.yaw;
  pose.speed = robot_state_.speed;
  pose_cache_.push(pose);
}

void BrokerNode::on_step_complete(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    broker_.notify_step_complete(now().seconds());
  }
}

void BrokerNode::on_timer()
{
  const double now_sec = now().seconds();

  // 1. Drain every command whose release-deadline has passed. The most recent
  //    one wins -- if four commands released this tick we publish only the
  //    freshest. Each release is logged so the CSV captures latency-aware
  //    release order.
  while (auto ready = broker_.try_pop_ready(now_sec)) {
    log_release_event(*ready);

    if (ready->emergency_stop) {
      // Latch an e-stop. Stays latched until a non-estop command arrives.
      QueuedCommand zero = *ready;
      zero.linear_x = 0.0;
      zero.angular_z = 0.0;
      current_cmd_ = zero;
      estop_latched_ = true;
    } else {
      current_cmd_ = *ready;
      estop_latched_ = false;
    }
    current_cmd_set_at_ = now_sec;
    have_current_cmd_ = true;
  }

  // 2. Decide what to publish. CONTINUOUS PUBLISHING is the key fix here:
  //    every tick we emit something, so Isaac's Twist subscriber always has
  //    a fresh value. If the upstream operator goes silent for longer than
  //    max_command_hold_ms_ we emit a zero (broker-side watchdog).
  geometry_msgs::msg::Twist out;
  const double age_ms = (now_sec - current_cmd_set_at_) * 1000.0;

  if (!have_current_cmd_) {
    // Boot-up: publish zero until the first command lands.
  } else if (estop_latched_) {
    // out stays zero
  } else if (age_ms <= max_command_hold_ms_) {
    out.linear.x = current_cmd_.linear_x;
    out.angular.z = current_cmd_.angular_z;
  } else {
    // Watchdog expired -- publish zero and forget the current command so we
    // don't spam zero-release log entries.
    have_current_cmd_ = false;
  }
  adaptive_cmd_pub_->publish(out);

  // 3. Telemetry + clock state for the dashboard / external consumers.
  auto telemetry = telemetry_fuser_.build(
    robot_state_, estimate_, broker_.broker_state());
  telemetry.stamp = stamp_from_now(*this);
  telemetry_pub_->publish(telemetry);

  publish_clock_state(age_ms);
}

void BrokerNode::publish_clock_state(double publish_age_ms)
{
  network_aware_msgs::msg::ClockState msg;
  msg.stamp = stamp_from_now(*this);
  msg.rtf = static_cast<float>(broker_.current_rtf());
  msg.estimated_latency_ms = static_cast<float>(broker_.estimated_latency_ms());
  msg.estimated_jitter_ms = static_cast<float>(broker_.estimated_jitter_ms());
  msg.release_lookahead_ms = static_cast<float>(broker_.release_lookahead_ms());
  msg.physics_step_ms = static_cast<float>(broker_.physics_step_ms());
  msg.network_step_ms = static_cast<float>(broker_.network_step_ms());
  msg.queue_depth = static_cast<uint32_t>(broker_.queue_depth());
  msg.step_gate_enabled = broker_.step_complete_gate_enabled();
  msg.deterministic_replay_enabled = broker_.deterministic_replay_enabled();
  if (estop_latched_) {
    msg.broker_state = "estop_latched";
  } else if (!have_current_cmd_) {
    msg.broker_state = "idle_watchdog";
  } else if (publish_age_ms > max_command_hold_ms_) {
    msg.broker_state = "holding_last";
  } else {
    msg.broker_state = broker_.broker_state();
  }
  clock_state_pub_->publish(msg);
}

void BrokerNode::log_teleop_event(const QueuedCommand & cmd)
{
  if (!log_.enabled()) {
    return;
  }
  std::ostringstream oss;
  oss << now().seconds() << ",teleop_rx," << cmd.seq << ","
      << cmd.linear_x << "," << cmd.angular_z << ","
      << estimate_.one_way_latency_ms << "," << estimate_.round_trip_latency_ms << ","
      << estimate_.jitter_ms << "," << estimate_.packet_error_rate << ","
      << estimate_.snr_db << "," << broker_.current_rtf() << ","
      << (estimate_.los ? "true" : "false") << "," << estimate_.path_loss_db << ","
      << cmd.source;
  log_.write_line(oss.str());
}

void BrokerNode::log_metrics_event(
  const NetworkEstimate & estimate, const std::string & source)
{
  if (!log_.enabled()) {
    return;
  }
  std::ostringstream oss;
  oss << now().seconds() << ",metrics_rx,0,0,0,"
      << estimate.one_way_latency_ms << "," << estimate.round_trip_latency_ms << ","
      << estimate.jitter_ms << "," << estimate.packet_error_rate << ","
      << estimate.snr_db << "," << estimate.rtf << ","
      << (estimate.los ? "true" : "false") << "," << estimate.path_loss_db << ","
      << source;
  log_.write_line(oss.str());
}

void BrokerNode::log_release_event(const QueuedCommand & cmd)
{
  if (!log_.enabled()) {
    return;
  }
  std::ostringstream oss;
  oss << now().seconds() << ",cmd_release," << cmd.seq << ","
      << cmd.linear_x << "," << cmd.angular_z << ","
      << estimate_.one_way_latency_ms << "," << estimate_.round_trip_latency_ms << ","
      << estimate_.jitter_ms << "," << estimate_.packet_error_rate << ","
      << estimate_.snr_db << "," << broker_.current_rtf() << ","
      << (estimate_.los ? "true" : "false") << "," << estimate_.path_loss_db << ","
      << cmd.source;
  log_.write_line(oss.str());
}

}  // namespace network_aware_broker

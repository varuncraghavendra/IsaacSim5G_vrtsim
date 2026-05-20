#include <chrono>

#include "network_aware_msgs/msg/clock_state.hpp"
#include "rclcpp/rclcpp.hpp"

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
#include "rosgraph_msgs/msg/clock.hpp"

class RtfClockNode : public rclcpp::Node
{
public:
  RtfClockNode()
  : Node("rtf_clock_node")
  {
    rtf_ = declare_parameter<double>("rtf", 0.95);
    period_ms_ = declare_parameter<int>("period_ms", 10);
    release_lookahead_ms_ = declare_parameter<double>("release_lookahead_ms", 12.0);
    physics_step_ms_ = declare_parameter<double>("physics_step_ms", 20.0);
    network_step_ms_ = declare_parameter<double>("network_step_ms", 10.0);
    deterministic_replay_enabled_ = declare_parameter<bool>("deterministic_replay_enabled", false);

    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    state_pub_ = create_publisher<network_aware_msgs::msg::ClockState>("/network/clock_state_dev", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms_),
      std::bind(&RtfClockNode::on_timer, this));
  }

private:
  void on_timer()
  {
    elapsed_sim_sec_ += (static_cast<double>(period_ms_) / 1000.0) * rtf_;

    const auto sec = static_cast<int32_t>(elapsed_sim_sec_);
    const auto nsec = static_cast<uint32_t>((elapsed_sim_sec_ - static_cast<double>(sec)) * 1e9);

    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock.sec = sec;
    clock_msg.clock.nanosec = nsec;
    clock_pub_->publish(clock_msg);

    network_aware_msgs::msg::ClockState state;
    state.stamp = stamp_from_now(*this);
    state.rtf = static_cast<float>(rtf_);
    state.estimated_latency_ms = 0.0F;
    state.estimated_jitter_ms = 0.0F;
    state.release_lookahead_ms = static_cast<float>(release_lookahead_ms_);
    state.physics_step_ms = static_cast<float>(physics_step_ms_);
    state.network_step_ms = static_cast<float>(network_step_ms_);
    state.queue_depth = 0;
    state.step_gate_enabled = true;
    state.deterministic_replay_enabled = deterministic_replay_enabled_;
    state.broker_state = "clock_only";
    state_pub_->publish(state);
  }

  double rtf_{0.95};
  int period_ms_{10};
  double release_lookahead_ms_{12.0};
  double physics_step_ms_{20.0};
  double network_step_ms_{10.0};
  bool deterministic_replay_enabled_{false};
  double elapsed_sim_sec_{0.0};

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<network_aware_msgs::msg::ClockState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RtfClockNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

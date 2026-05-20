#pragma once

#include <deque>
#include <optional>
#include <string>

#include "network_aware_broker/broker_types.hpp"

namespace network_aware_broker
{

class TimeBroker
{
public:
  TimeBroker();

  void set_default_rtf(double rtf);
  void set_release_lookahead_ms(double lookahead_ms);
  void set_max_command_age_ms(double max_age_ms);
  void set_max_queue_size(std::size_t max_queue_size);
  void set_step_complete_gate(bool enabled);
  void set_hold_last_command_ms(double hold_ms);
  void set_physics_step_ms(double step_ms);
  void set_network_step_ms(double step_ms);
  void set_deterministic_replay_enabled(bool enabled);

  void update_network_estimate(const NetworkEstimate & estimate);
  void push_command(const QueuedCommand & cmd);
  void notify_step_complete(double now_sec);

  std::optional<QueuedCommand> try_pop_ready(double now_sec);

  std::size_t queue_depth() const;
  double estimated_latency_ms() const;
  double estimated_jitter_ms() const;
  double current_rtf() const;
  double release_lookahead_ms() const;
  double physics_step_ms() const;
  double network_step_ms() const;
  bool step_complete_gate_enabled() const;
  bool deterministic_replay_enabled() const;
  std::string broker_state() const;

private:
  double compute_release_time_sec(double now_sec) const;
  bool is_stale(const QueuedCommand & cmd, double now_sec) const;

  std::deque<QueuedCommand> queue_;
  NetworkEstimate estimate_;
  double default_rtf_{1.0};
  double release_lookahead_ms_{10.0};
  double max_command_age_ms_{250.0};
  double hold_last_command_ms_{100.0};
  double physics_step_ms_{20.0};
  double network_step_ms_{10.0};
  std::size_t max_queue_size_{256};
  bool step_complete_gate_enabled_{true};
  bool step_completed_since_last_release_{true};
  bool deterministic_replay_enabled_{false};
  std::string broker_state_{"idle"};
};

}  // namespace network_aware_broker

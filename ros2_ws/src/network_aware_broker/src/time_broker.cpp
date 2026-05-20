#include "network_aware_broker/time_broker.hpp"

#include <algorithm>
#include <cmath>

namespace network_aware_broker
{

TimeBroker::TimeBroker() = default;

void TimeBroker::set_default_rtf(double rtf)
{
  default_rtf_ = rtf;
}

void TimeBroker::set_release_lookahead_ms(double lookahead_ms)
{
  release_lookahead_ms_ = lookahead_ms;
}

void TimeBroker::set_max_command_age_ms(double max_age_ms)
{
  max_command_age_ms_ = max_age_ms;
}

void TimeBroker::set_max_queue_size(std::size_t max_queue_size)
{
  max_queue_size_ = max_queue_size;
}

void TimeBroker::set_step_complete_gate(bool enabled)
{
  step_complete_gate_enabled_ = enabled;
}

void TimeBroker::set_hold_last_command_ms(double hold_ms)
{
  hold_last_command_ms_ = hold_ms;
}

void TimeBroker::set_physics_step_ms(double step_ms)
{
  physics_step_ms_ = step_ms;
}

void TimeBroker::set_network_step_ms(double step_ms)
{
  network_step_ms_ = step_ms;
}

void TimeBroker::set_deterministic_replay_enabled(bool enabled)
{
  deterministic_replay_enabled_ = enabled;
}

void TimeBroker::update_network_estimate(const NetworkEstimate & estimate)
{
  estimate_ = estimate;
  if (estimate_.rtf <= 0.0) {
    estimate_.rtf = default_rtf_;
  }
}

void TimeBroker::push_command(const QueuedCommand & cmd)
{
  QueuedCommand staged = cmd;
  staged.release_time_sec = compute_release_time_sec(cmd.stamp_sec);

  if (queue_.size() >= max_queue_size_) {
    queue_.pop_front();
  }

  queue_.push_back(staged);
  broker_state_ = "queued";
}

void TimeBroker::notify_step_complete(double)
{
  step_completed_since_last_release_ = true;
  if (queue_.empty()) {
    broker_state_ = "idle";
  }
}

std::optional<QueuedCommand> TimeBroker::try_pop_ready(double now_sec)
{
  while (!queue_.empty()) {
    const auto & front = queue_.front();

    if (is_stale(front, now_sec)) {
      queue_.pop_front();
      broker_state_ = "dropping_stale";
      continue;
    }

    if (front.release_time_sec > now_sec) {
      broker_state_ = "waiting_release_deadline";
      return std::nullopt;
    }

    if (step_complete_gate_enabled_ && !step_completed_since_last_release_) {
      broker_state_ = "waiting_step_complete";
      return std::nullopt;
    }

    auto ready = front;
    queue_.pop_front();
    step_completed_since_last_release_ = false;
    broker_state_ = ready.emergency_stop ? "released_estop" : "released";
    return ready;
  }

  broker_state_ = "idle";
  return std::nullopt;
}

std::size_t TimeBroker::queue_depth() const
{
  return queue_.size();
}

double TimeBroker::estimated_latency_ms() const
{
  return estimate_.one_way_latency_ms;
}

double TimeBroker::estimated_jitter_ms() const
{
  return estimate_.jitter_ms;
}

double TimeBroker::current_rtf() const
{
  return estimate_.rtf > 0.0 ? estimate_.rtf : default_rtf_;
}

double TimeBroker::release_lookahead_ms() const
{
  return release_lookahead_ms_;
}

double TimeBroker::physics_step_ms() const
{
  return physics_step_ms_;
}

double TimeBroker::network_step_ms() const
{
  return network_step_ms_;
}

bool TimeBroker::step_complete_gate_enabled() const
{
  return step_complete_gate_enabled_;
}

bool TimeBroker::deterministic_replay_enabled() const
{
  return deterministic_replay_enabled_;
}

std::string TimeBroker::broker_state() const
{
  return broker_state_;
}

double TimeBroker::compute_release_time_sec(double now_sec) const
{
  const double rtf = std::max(0.1, current_rtf());
  const double latency_sec = estimate_.one_way_latency_ms / 1000.0;
  const double jitter_sec = estimate_.jitter_ms / 1000.0;
  const double loss_margin_sec = estimate_.packet_error_rate * (hold_last_command_ms_ / 1000.0);
  const double lookahead_sec = release_lookahead_ms_ / 1000.0;
  const double cadence_guard_sec = std::max(physics_step_ms_, network_step_ms_) / 1000.0;
  return now_sec + (latency_sec + 0.5 * jitter_sec + loss_margin_sec + lookahead_sec + cadence_guard_sec) / rtf;
}

bool TimeBroker::is_stale(const QueuedCommand & cmd, double now_sec) const
{
  return (now_sec - cmd.stamp_sec) * 1000.0 > max_command_age_ms_;
}

}  // namespace network_aware_broker

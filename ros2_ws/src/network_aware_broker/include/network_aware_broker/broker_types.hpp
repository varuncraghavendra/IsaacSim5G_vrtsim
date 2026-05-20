#pragma once

#include <cstdint>
#include <deque>
#include <string>

namespace network_aware_broker
{

struct QueuedCommand
{
  double stamp_sec{0.0};
  uint32_t seq{0};
  double linear_x{0.0};
  double angular_z{0.0};
  double desired_rtf{1.0};
  bool emergency_stop{false};
  std::string source{"unknown"};
  double release_time_sec{0.0};
};

struct NetworkEstimate
{
  double one_way_latency_ms{0.0};
  double round_trip_latency_ms{0.0};
  double jitter_ms{0.0};
  double packet_error_rate{0.0};
  double snr_db{0.0};
  double rtf{1.0};
  bool los{true};
  double path_loss_db{0.0};
};

struct RobotState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double speed{0.0};
};

struct TimedPose
{
  double stamp_sec{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double speed{0.0};
};

}  // namespace network_aware_broker

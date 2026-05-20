#include "network_aware_broker/telemetry_fuser.hpp"

#include <cmath>

namespace network_aware_broker
{

network_aware_msgs::msg::RobotTelemetry TelemetryFuser::build(
  const RobotState & robot,
  const NetworkEstimate & estimate,
  const std::string & broker_state) const
{
  network_aware_msgs::msg::RobotTelemetry msg;
  msg.x = static_cast<float>(robot.x);
  msg.y = static_cast<float>(robot.y);
  msg.yaw = static_cast<float>(robot.yaw);
  msg.speed = static_cast<float>(robot.speed);
  msg.distance_from_origin = static_cast<float>(std::sqrt(robot.x * robot.x + robot.y * robot.y));
  msg.rtf = static_cast<float>(estimate.rtf);
  msg.estimated_latency_ms = static_cast<float>(estimate.one_way_latency_ms);
  msg.estimated_jitter_ms = static_cast<float>(estimate.jitter_ms);
  msg.packet_error_rate = static_cast<float>(estimate.packet_error_rate);
  msg.snr_db = static_cast<float>(estimate.snr_db);
  msg.los = estimate.los;
  msg.broker_state = broker_state;
  return msg;
}

}  // namespace network_aware_broker

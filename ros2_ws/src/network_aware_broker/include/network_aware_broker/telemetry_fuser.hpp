#pragma once

#include "network_aware_broker/broker_types.hpp"
#include "network_aware_msgs/msg/robot_telemetry.hpp"

namespace network_aware_broker
{

class TelemetryFuser
{
public:
  network_aware_msgs::msg::RobotTelemetry build(
    const RobotState & robot,
    const NetworkEstimate & estimate,
    const std::string & broker_state) const;
};

}  // namespace network_aware_broker

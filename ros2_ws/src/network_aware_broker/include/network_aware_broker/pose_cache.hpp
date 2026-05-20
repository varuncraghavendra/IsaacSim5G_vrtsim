#pragma once

#include <deque>
#include <optional>

#include "network_aware_broker/broker_types.hpp"

namespace network_aware_broker
{

class PoseCache
{
public:
  void set_max_size(std::size_t max_size);
  void push(const TimedPose & pose);
  std::optional<TimedPose> latest() const;
  std::optional<TimedPose> interpolate_or_latest(double stamp_sec) const;

private:
  std::size_t max_size_{256};
  std::deque<TimedPose> poses_;
};

}  // namespace network_aware_broker

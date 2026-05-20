#include "network_aware_broker/pose_cache.hpp"

#include <algorithm>

namespace network_aware_broker
{

void PoseCache::set_max_size(std::size_t max_size)
{
  max_size_ = std::max<std::size_t>(2, max_size);
}

void PoseCache::push(const TimedPose & pose)
{
  poses_.push_back(pose);
  while (poses_.size() > max_size_) {
    poses_.pop_front();
  }
}

std::optional<TimedPose> PoseCache::latest() const
{
  if (poses_.empty()) {
    return std::nullopt;
  }
  return poses_.back();
}

std::optional<TimedPose> PoseCache::interpolate_or_latest(double stamp_sec) const
{
  if (poses_.empty()) {
    return std::nullopt;
  }
  if (poses_.size() == 1 || stamp_sec >= poses_.back().stamp_sec) {
    return poses_.back();
  }

  for (std::size_t idx = 1; idx < poses_.size(); ++idx) {
    const auto & prev = poses_[idx - 1];
    const auto & next = poses_[idx];
    if (stamp_sec >= prev.stamp_sec && stamp_sec <= next.stamp_sec) {
      const double span = std::max(1e-6, next.stamp_sec - prev.stamp_sec);
      const double ratio = (stamp_sec - prev.stamp_sec) / span;
      TimedPose out;
      out.stamp_sec = stamp_sec;
      out.x = prev.x + ratio * (next.x - prev.x);
      out.y = prev.y + ratio * (next.y - prev.y);
      out.yaw = prev.yaw + ratio * (next.yaw - prev.yaw);
      out.speed = prev.speed + ratio * (next.speed - prev.speed);
      return out;
    }
  }

  return poses_.front();
}

}  // namespace network_aware_broker

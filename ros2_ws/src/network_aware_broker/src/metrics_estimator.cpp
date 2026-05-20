#include "network_aware_broker/metrics_estimator.hpp"

#include <algorithm>

namespace network_aware_broker
{

MetricsEstimator::MetricsEstimator(double alpha)
: alpha_(alpha)
{
}

void MetricsEstimator::set_alpha(double alpha)
{
  alpha_ = std::clamp(alpha, 0.0, 1.0);
}

NetworkEstimate MetricsEstimator::update(const NetworkEstimate & sample) const
{
  if (!initialized_) {
    smoothed_ = sample;
    initialized_ = true;
    return smoothed_;
  }

  const auto blend = [this](double old_value, double new_value) {
      return (1.0 - alpha_) * old_value + alpha_ * new_value;
    };

  smoothed_.one_way_latency_ms = blend(smoothed_.one_way_latency_ms, sample.one_way_latency_ms);
  smoothed_.round_trip_latency_ms = blend(smoothed_.round_trip_latency_ms, sample.round_trip_latency_ms);
  smoothed_.jitter_ms = blend(smoothed_.jitter_ms, sample.jitter_ms);
  smoothed_.packet_error_rate = blend(smoothed_.packet_error_rate, sample.packet_error_rate);
  smoothed_.snr_db = blend(smoothed_.snr_db, sample.snr_db);
  smoothed_.rtf = blend(smoothed_.rtf, sample.rtf);
  smoothed_.los = sample.los;
  smoothed_.path_loss_db = blend(smoothed_.path_loss_db, sample.path_loss_db);
  return smoothed_;
}

}  // namespace network_aware_broker

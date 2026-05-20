#pragma once

#include "network_aware_broker/broker_types.hpp"

namespace network_aware_broker
{

class MetricsEstimator
{
public:
  explicit MetricsEstimator(double alpha = 0.2);

  void set_alpha(double alpha);
  NetworkEstimate update(const NetworkEstimate & sample) const;

private:
  double alpha_{0.2};
  mutable bool initialized_{false};
  mutable NetworkEstimate smoothed_;
};

}  // namespace network_aware_broker

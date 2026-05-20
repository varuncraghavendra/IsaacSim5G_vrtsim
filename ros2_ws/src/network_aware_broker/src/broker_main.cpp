#include "network_aware_broker/broker_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<network_aware_broker::BrokerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

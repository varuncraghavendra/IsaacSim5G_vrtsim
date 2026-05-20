#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <stdexcept>
#include <string>

#include "network_aware_broker/json_extract.hpp"
#include "network_aware_msgs/msg/teleop_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
builtin_interfaces::msg::Time stamp_from_now(rclcpp::Node & node)
{
  const auto t = node.now();
  builtin_interfaces::msg::Time msg;
  const auto ns = t.nanoseconds();
  msg.sec = static_cast<int32_t>(ns / 1000000000LL);
  msg.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return msg;
}
}  // namespace

class UeIngressNode : public rclcpp::Node
{
public:
  UeIngressNode()
  : Node("ue_ingress_node")
  {
    const auto port = declare_parameter<int>("udp_port", 16000);
    const auto topic = declare_parameter<std::string>("output_topic", "/network/teleop_command");

    pub_ = create_publisher<network_aware_msgs::msg::TeleopCommand>(topic, 20);
    setup_socket(port);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&UeIngressNode::poll_socket, this));
  }

  ~UeIngressNode() override
  {
    if (sock_ >= 0) {
      close(sock_);
    }
  }

private:
  void setup_socket(int port)
  {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      throw std::runtime_error("Failed to create UDP socket");
    }

    int reuse = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      throw std::runtime_error("Failed to bind UDP socket");
    }
  }

  void poll_socket()
  {
    char buffer[4096];
    sockaddr_in src{};
    socklen_t src_len = sizeof(src);

    const int received = recvfrom(
      sock_,
      buffer,
      sizeof(buffer) - 1,
      MSG_DONTWAIT,
      reinterpret_cast<sockaddr *>(&src),
      &src_len);

    if (received <= 0) {
      return;
    }

    buffer[received] = '\0';
    const std::string payload(buffer);

    network_aware_msgs::msg::TeleopCommand msg;
    msg.stamp = stamp_from_now(*this);
    msg.seq = static_cast<uint32_t>(network_aware_broker::json_number(payload, "seq", 0.0));
    msg.linear_x = static_cast<float>(network_aware_broker::json_number(payload, "linear_x", 0.0));
    msg.angular_z = static_cast<float>(network_aware_broker::json_number(payload, "angular_z", 0.0));
    msg.desired_rtf = static_cast<float>(network_aware_broker::json_number(payload, "desired_rtf", 0.95));
    msg.emergency_stop = network_aware_broker::json_bool(payload, "emergency_stop", false);
    msg.source = network_aware_broker::json_string(payload, "source", "udp_json");

    pub_->publish(msg);
  }

  int sock_{-1};
  rclcpp::Publisher<network_aware_msgs::msg::TeleopCommand>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UeIngressNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <zmq.hpp>
#include <cstring>
#include <sstream>

class JoyToZMQ : public rclcpp::Node {
public:
  JoyToZMQ() : Node("joy_to_zmq") {
    target_ip_ = this->declare_parameter<std::string>("target_ip", "127.0.0.1");
    target_port_ = this->declare_parameter<int>("target_port", 5555);

    socket_ = std::make_unique<zmq::socket_t>(context_, zmq::socket_type::push);
    std::stringstream addr;
    addr << "tcp://" << target_ip_ << ":" << target_port_;
    socket_->connect(addr.str());

    RCLCPP_INFO(this->get_logger(),
                "Sending Joy messages from topic '%s' to %s:%d",
                "joy", target_ip_.c_str(), target_port_);

    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyToZMQ::joy_callback, this, std::placeholders::_1));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    uint32_t axes_count = static_cast<uint32_t>(msg->axes.size());
    uint32_t buttons_count = static_cast<uint32_t>(msg->buttons.size());

    size_t total_size = sizeof(uint32_t) * 2 +
                        axes_count * sizeof(float) +
                        buttons_count * sizeof(int32_t);

    zmq::message_t message(total_size);
    size_t offset = 0;
    std::memcpy(static_cast<char*>(message.data()) + offset, &axes_count,
                sizeof(uint32_t));
    offset += sizeof(uint32_t);
    std::memcpy(static_cast<char*>(message.data()) + offset, &buttons_count,
                sizeof(uint32_t));
    offset += sizeof(uint32_t);
    if (axes_count > 0) {
      std::memcpy(static_cast<char*>(message.data()) + offset,
                  msg->axes.data(), axes_count * sizeof(float));
      offset += axes_count * sizeof(float);
    }
    if (buttons_count > 0) {
      std::memcpy(static_cast<char*>(message.data()) + offset,
                  msg->buttons.data(), buttons_count * sizeof(int32_t));
    }
    try {
      socket_->send(message, zmq::send_flags::dontwait);
    } catch (const zmq::error_t &e) {
      RCLCPP_WARN(this->get_logger(), "ZMQ send failed: %s", e.what());
    }
  }

  zmq::context_t context_{1};
  std::unique_ptr<zmq::socket_t> socket_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  std::string target_ip_;
  int target_port_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToZMQ>());
  rclcpp::shutdown();
  return 0;
}

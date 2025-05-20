#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <zmq.hpp>
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

    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyToZMQ::joy_callback, this, std::placeholders::_1));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    std::ostringstream out;
    out << "axes:";
    for (size_t i = 0; i < msg->axes.size(); ++i) {
      if (i) out << ',';
      out << msg->axes[i];
    }
    out << "|buttons:";
    for (size_t i = 0; i < msg->buttons.size(); ++i) {
      if (i) out << ',';
      out << msg->buttons[i];
    }
    auto data = out.str();
    zmq::message_t message(data.begin(), data.end());
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

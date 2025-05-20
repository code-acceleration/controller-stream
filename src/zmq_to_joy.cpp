#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <zmq.hpp>
#include <sstream>
#include <thread>
#include <atomic>

class ZMQToJoy : public rclcpp::Node {
public:
  ZMQToJoy() : Node("zmq_to_joy") {
    bind_ip_ = this->declare_parameter<std::string>("bind_ip", "0.0.0.0");
    bind_port_ = this->declare_parameter<int>("bind_port", 5555);

    socket_ = std::make_unique<zmq::socket_t>(context_, zmq::socket_type::pull);
    std::stringstream addr;
    addr << "tcp://" << bind_ip_ << ":" << bind_port_;
    socket_->bind(addr.str());

    pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy_out", 10);

    recv_thread_ = std::thread([this]() { this->recv_loop(); });
  }

  ~ZMQToJoy() override {
    running_ = false;
    try {
      if (socket_)
        socket_->close();
    } catch (const zmq::error_t &e) {
      RCLCPP_WARN(this->get_logger(), "ZMQ close failed: %s", e.what());
    }
    if (recv_thread_.joinable())
      recv_thread_.join();
  }

private:
  void recv_loop() {
    while (running_) {
      zmq::message_t msg;
      try {
        socket_->recv(msg, zmq::recv_flags::none);
      } catch (const zmq::error_t &e) {
        if (!running_)
          break;
        RCLCPP_WARN(this->get_logger(), "ZMQ recv failed: %s", e.what());
        continue;
      }

      if (!running_)
        break;

      std::string data(static_cast<char*>(msg.data()), msg.size());
      sensor_msgs::msg::Joy joy_msg;

      auto buttons_pos = data.find("|buttons:");
      if (buttons_pos == std::string::npos)
        continue;

      std::string axes_part = data.substr(6, buttons_pos - 6); // after "axes:"
      std::string buttons_part = data.substr(buttons_pos + 9); // after "|buttons:"

      parse_floats(axes_part, joy_msg.axes);
      parse_ints(buttons_part, joy_msg.buttons);

      pub_->publish(joy_msg);
    }
  }

  void parse_floats(const std::string &str, std::vector<float> &out) {
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, ',')) {
      if (!item.empty()) {
        out.push_back(std::stof(item));
      }
    }
  }

  void parse_ints(const std::string &str, std::vector<int32_t> &out) {
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, ',')) {
      if (!item.empty()) {
        out.push_back(std::stoi(item));
      }
    }
  }

  zmq::context_t context_{1};
  std::unique_ptr<zmq::socket_t> socket_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  std::thread recv_thread_;
  std::atomic_bool running_{true};
  std::string bind_ip_;
  int bind_port_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZMQToJoy>());
  rclcpp::shutdown();
  return 0;
}

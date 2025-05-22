#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <zmq.hpp>
#include <cstring>
#include <sstream>
#include <thread>
#include <atomic>

class ZMQToJoy : public rclcpp::Node {
public:
  ZMQToJoy() : Node("zmq_to_joy") {
    bind_ip_ = this->declare_parameter<std::string>("bind_ip", "0.0.0.0");
    bind_port_ = this->declare_parameter<int>("bind_port", 5555);

    socket_ = std::make_unique<zmq::socket_t>(context_, zmq::socket_type::pull);
    socket_->set(zmq::sockopt::rcvhwm, 1);
    socket_->set(zmq::sockopt::conflate, 1);
    socket_->set(zmq::sockopt::tcp_keepalive, 1);
    std::stringstream addr;
    addr << "tcp://" << bind_ip_ << ":" << bind_port_;
    socket_->bind(addr.str());

    pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy_out", 10);

    RCLCPP_INFO(this->get_logger(),
                "Receiving Joy on %s:%d and publishing to topic '%s'",
                bind_ip_.c_str(), bind_port_, "joy_out");

    recv_thread_ = std::thread([this]() { this->recv_loop(); });

    rclcpp::on_shutdown([this]() { this->stop(); });
  }

  ~ZMQToJoy() override { stop(); }

private:
  void stop() {
    if (running_) {
      running_ = false;
      try {
        if (socket_)
          socket_->close();
      } catch (const zmq::error_t &e) {
        RCLCPP_WARN(this->get_logger(), "ZMQ close failed: %s", e.what());
      }
    }
    if (recv_thread_.joinable())
      recv_thread_.join();
  }
  void recv_loop() {
    while (running_) {
      message_.rebuild();
      try {
        socket_->recv(message_, zmq::recv_flags::none);
      } catch (const zmq::error_t &e) {
        if (!running_)
          break;
        RCLCPP_WARN(this->get_logger(), "ZMQ recv failed: %s", e.what());
        continue;
      }

      if (!running_)
        break;

      const char *data = static_cast<const char *>(message_.data());
      size_t size = message_.size();

      if (size < sizeof(uint32_t) * 2)
        continue;

      uint32_t axes_count = 0;
      uint32_t buttons_count = 0;
      std::memcpy(&axes_count, data, sizeof(uint32_t));
      std::memcpy(&buttons_count, data + sizeof(uint32_t), sizeof(uint32_t));

      size_t expected = sizeof(uint32_t) * 2 +
                        axes_count * sizeof(float) +
                        buttons_count * sizeof(int32_t);

      if (size != expected)
        continue;

      sensor_msgs::msg::Joy joy_msg;
      joy_msg.axes.resize(axes_count);
      joy_msg.buttons.resize(buttons_count);

      size_t offset = sizeof(uint32_t) * 2;
      if (axes_count > 0) {
        std::memcpy(joy_msg.axes.data(), data + offset,
                    axes_count * sizeof(float));
        offset += axes_count * sizeof(float);
      }
      if (buttons_count > 0) {
        std::memcpy(joy_msg.buttons.data(), data + offset,
                    buttons_count * sizeof(int32_t));
      }

      pub_->publish(joy_msg);
    }
  }

  zmq::context_t context_{1};
  std::unique_ptr<zmq::socket_t> socket_;
  zmq::message_t message_;
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

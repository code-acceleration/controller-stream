#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <zmq.hpp>
#include <cstring>
#include <sstream>
#include <thread>
#include <atomic>

class Ros2ZmqBridge : public rclcpp::Node {
public:
  Ros2ZmqBridge() : Node("ros2_zmq_bridge") {
    mode_ = this->declare_parameter<std::string>("mode", "ros2_to_zmq");
    topic_name_ = this->declare_parameter<std::string>("topic", "chatter");
    type_name_ = this->declare_parameter<std::string>("type", "std_msgs/msg/String");
    ip_ = this->declare_parameter<std::string>("ip", "127.0.0.1");
    port_ = this->declare_parameter<int>("port", 5555);

    if (mode_ == "ros2_to_zmq") {
      socket_ = std::make_unique<zmq::socket_t>(context_, zmq::socket_type::push);
      socket_->set(zmq::sockopt::sndhwm, 1);
      socket_->set(zmq::sockopt::conflate, 1);
      socket_->set(zmq::sockopt::tcp_keepalive, 1);
      std::stringstream addr;
      addr << "tcp://" << ip_ << ":" << port_;
      socket_->connect(addr.str());
      sub_ = this->create_generic_subscription(
          topic_name_, type_name_, rclcpp::QoS(10),
          std::bind(&Ros2ZmqBridge::sub_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(),
                  "Forwarding ROS2 topic '%s' [%s] to %s:%d",
                  topic_name_.c_str(), type_name_.c_str(), ip_.c_str(), port_);
    } else if (mode_ == "zmq_to_ros2") {
      socket_ = std::make_unique<zmq::socket_t>(context_, zmq::socket_type::pull);
      socket_->set(zmq::sockopt::rcvhwm, 1);
      socket_->set(zmq::sockopt::conflate, 1);
      socket_->set(zmq::sockopt::tcp_keepalive, 1);
      std::stringstream addr;
      addr << "tcp://" << ip_ << ":" << port_;
      socket_->bind(addr.str());
      pub_ = this->create_generic_publisher(topic_name_, type_name_, rclcpp::QoS(10));
      recv_thread_ = std::thread([this]() { this->recv_loop(); });
      RCLCPP_INFO(this->get_logger(),
                  "Receiving ZMQ on %s:%d and publishing to topic '%s' [%s]",
                  ip_.c_str(), port_, topic_name_.c_str(), type_name_.c_str());
    } else {
      RCLCPP_FATAL(this->get_logger(), "Unknown mode '%s'", mode_.c_str());
      rclcpp::shutdown();
    }

    rclcpp::on_shutdown([this]() { this->stop(); });
  }

  ~Ros2ZmqBridge() override { stop(); }

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

  void sub_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    try {
      socket_->send(
          zmq::const_buffer(msg->get_rcl_serialized_message().buffer, msg->size()),
          zmq::send_flags::dontwait);
    } catch (const zmq::error_t &e) {
      RCLCPP_WARN(this->get_logger(), "ZMQ send failed: %s", e.what());
    }
  }

  void recv_loop() {
    while (running_) {
      zmq::message_t m;
      try {
        socket_->recv(m, zmq::recv_flags::none);
      } catch (const zmq::error_t &e) {
        if (!running_)
          break;
        RCLCPP_WARN(this->get_logger(), "ZMQ recv failed: %s", e.what());
        continue;
      }
      if (!running_)
        break;
      rclcpp::SerializedMessage ros_msg(m.size());
      std::memcpy(ros_msg.get_rcl_serialized_message().buffer, m.data(), m.size());
      ros_msg.get_rcl_serialized_message().buffer_length = m.size();
      pub_->publish(ros_msg);
    }
  }

  zmq::context_t context_{1};
  std::unique_ptr<zmq::socket_t> socket_;
  std::thread recv_thread_;
  std::atomic_bool running_{true};
  rclcpp::SubscriptionBase::SharedPtr sub_;
  rclcpp::GenericPublisher::SharedPtr pub_;
  std::string mode_;
  std::string topic_name_;
  std::string type_name_;
  std::string ip_;
  int port_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2ZmqBridge>());
  rclcpp::shutdown();
  return 0;
}


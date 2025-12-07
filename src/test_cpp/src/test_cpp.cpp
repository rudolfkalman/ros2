#include <chrono>
#include <functional>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include <eigen3/Eigen/Dense>

using namespace std::chrono_literals;
using namespace Eigen;

class IntPublisher : public rclcpp::Node {
public:
  IntPublisher() : Node("int_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("message", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&IntPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    std_msgs::msg::Int32 message;
    message.data = static_cast<int32_t>(count_++);

    RCLCPP_INFO(this->get_logger(), "publishing: %d", message.data);

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  int32_t count_;
};

int main(int argc, char * argv[]) {
  Matrix2d m;
  m(0, 0) = 3;
  m(1, 0) = 2.5;
  m(0, 1) = -1;
  m(1, 1) = m(1, 0) + m(0, 1);
  std::cout << m << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IntPublisher>());
  rclcpp::shutdown();
  return 0;
}


// Copyright (c) 2026 WHEELTEC ROS Textbook Contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class CoursePublisher : public rclcpp::Node
{
public:
  CoursePublisher()
  : Node("course_cpp_talker")
  {
    publisher_ = create_publisher<std_msgs::msg::String>("/course/chatter", 10);
    timer_ = create_wall_timer(500ms, std::bind(&CoursePublisher::publish_message, this));
  }

private:
  void publish_message()
  {
    std_msgs::msg::String message;
    message.data = "C++ says hello #" + std::to_string(sequence_++);
    RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t sequence_{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoursePublisher>());
  rclcpp::shutdown();
  return 0;
}

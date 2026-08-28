// Copyright (c) 2026 WHEELTEC ROS Textbook Contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class CourseSubscriber : public rclcpp::Node
{
public:
  CourseSubscriber()
  : Node("course_cpp_listener")
  {
    subscription_ = create_subscription<std_msgs::msg::String>(
      "/course/chatter", 10,
      std::bind(&CourseSubscriber::receive_message, this, _1));
  }

private:
  void receive_message(const std_msgs::msg::String & message) const
  {
    RCLCPP_INFO(get_logger(), "Received: '%s'", message.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CourseSubscriber>());
  rclcpp::shutdown();
  return 0;
}

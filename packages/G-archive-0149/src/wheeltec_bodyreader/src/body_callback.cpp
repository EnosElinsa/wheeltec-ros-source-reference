#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32.hpp>

#include "ai_msgs/msg/target.hpp"
#include "wheeltec_bodymsg/msg/bodyposture.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class body_callback : public rclcpp::Node
{
   rclcpp::Subscription<wheeltec_bodymsg::msg::Bodyposture>::SharedPtr sub_bodyposture;
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmdvel;
  public:
    body_callback()
    : Node("body_callback")
    {
      sub_bodyposture = this->create_subscription<wheeltec_bodymsg::msg::Bodyposture>(
      "/bodyposture_event", 10, std::bind(&body_callback::topic_callback, this, _1));
      pub_cmdvel =this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

  private:
    void topic_callback(const wheeltec_bodymsg::msg::Bodyposture::ConstSharedPtr msg)
    {
    geometry_msgs::msg::Twist twist;
    
    if (msg->left_foot_up==1 && msg->right_foot_up==0 ){
    twist.linear.x=0.15;
    }
    else if (msg->right_foot_up==1 && msg->left_foot_up==0 ){
        twist.linear.x=-0.15;
    }
    else{
        twist.linear.x=0;
    }
   pub_cmdvel->publish(twist);
    }  

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<body_callback>());
  rclcpp::shutdown();
  return 0;
}

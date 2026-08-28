#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ObstacleAvoidance : public rclcpp::Node
{
public:
  ObstacleAvoidance() : Node("obstacle_avoidance")
  {
    // 创建发布者
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // 创建订阅者
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_ori", 10,
      std::bind(&ObstacleAvoidance::cmd_vel_callback, this, std::placeholders::_1));
 
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&ObstacleAvoidance::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance node initialized");
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    current_cmd_ = *msg;
    // check_and_publish();
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
	
		// 检查激光雷达数据是否有效
		if (scan->ranges.empty()) {
		  RCLCPP_WARN(this->get_logger(), "Received empty laser scan");
		  return;
		}
	
		obstacle_detected_ = false;
    
		for (size_t i = 0; i < scan->ranges.size(); i++) {
		  double angle = scan->angle_min + i * scan->angle_increment;
		  // 只检查±60度范围（±1.047弧度）
		  if (angle >= -1.047 && angle <= 1.047) {
			if (std::isfinite(scan->ranges[i]) && scan->ranges[i] < 0.3) {
			RCLCPP_INFO(this->get_logger(), "Obstacle detected at angle: %.2f, range: %.2f", angle, scan->ranges[i]);
			  obstacle_detected_ = true;
			  break;
			}
		  }
		}
	

     check_and_publish();
  }

  void check_and_publish()
  {
    auto cmd_msg = geometry_msgs::msg::Twist();
    
    if (obstacle_detected_) {
      // 如果检测到障碍物，发送零速度命令
      cmd_msg.linear.x = 0.0;
      cmd_msg.linear.y = 0.0;
      cmd_msg.linear.z = 0.0;
      cmd_msg.angular.x = 0.0;
      cmd_msg.angular.y = 0.0;
      cmd_msg.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Obstacle detected! Stopping.");
    } else {
      // 如果没有障碍物，使用原始速度命令
      cmd_msg = current_cmd_;
    }

    cmd_vel_pub_->publish(cmd_msg);
  }

  // 成员变量
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  
  geometry_msgs::msg::Twist current_cmd_;
  bool obstacle_detected_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleAvoidance>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
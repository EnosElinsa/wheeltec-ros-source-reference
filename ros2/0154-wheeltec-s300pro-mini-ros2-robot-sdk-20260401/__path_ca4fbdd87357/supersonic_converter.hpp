#pragma once

#include <string>
#include <map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <robot_interfaces/msg/supersonic.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>



struct UltraConfig
{
  std::string robot_type; // s300_pro or s300_mini
  std::string input_topic; // Distance
  std::string base_frame;  // base_footprint
  bool publish_pointcloud{true};
  double min_range{0.02};
  double max_range{3.0};
  double field_of_view{0.5}; // radians
};

class SupersonicConverter : public rclcpp::Node
{
public:
  explicit SupersonicConverter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void supersonic_callback(const robot_interfaces::msg::Supersonic::SharedPtr msg);
  void publish_one_range(const std::string & frame_id, const std::string & topic_suffix, float distance, const rclcpp::Time & stamp);
  void publish_pointcloud( robot_interfaces::msg::Supersonic & msg);

  UltraConfig cfg_;
  rclcpp::Subscription<robot_interfaces::msg::Supersonic>::SharedPtr sub_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> range_pubs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

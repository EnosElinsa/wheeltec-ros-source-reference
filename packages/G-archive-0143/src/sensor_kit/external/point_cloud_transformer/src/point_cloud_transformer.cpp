#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

class PointCloudTransformer : public rclcpp::Node
{
public:
  PointCloudTransformer(const rclcpp::NodeOptions &options)
  : Node("point_cloud_transformer", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/top/point_cloud_raw", rclcpp::QoS(5),
      std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/concatenated/pointcloud", 10);
    
    while (!tf_buffer_.canTransform("base_link", "laser", rclcpp::Time())) {
      //RCLCPP_WARN(get_logger(), "Waiting for TF transform between base_link and radar_frame...");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    try {
      transform = tf_buffer_.lookupTransform("base_link", "laser", rclcpp::Time());

    } catch (tf2::TransformException &ex) {
      //RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    auto oldstamp = msg->header.stamp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*pcl_cloud, *pcl_transformed_cloud, tf2::transformToEigen(transform.transform).cast<float>());
    pcl::toROSMsg(*pcl_transformed_cloud, transformed_cloud);

    transformed_cloud.header.stamp = oldstamp;
    transformed_cloud.header.frame_id = "base_link";

    point_cloud_pub_->publish(transformed_cloud);

  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::TransformStamped transform;
};

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudTransformer)

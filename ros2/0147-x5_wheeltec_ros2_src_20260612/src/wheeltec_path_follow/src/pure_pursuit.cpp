/* A ROS implementation of the Pure pursuit path tracking algorithm (Coulter 1992).
   Terminology (mostly :) follows:
   Coulter, Implementation of the pure pursuit algoritm, 1992 and 
   Sorniotti et al. Path tracking for Automated Driving, 2017.
*/
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include <turn_on_wheeltec_robot/msg/position.hpp>
#include <kdl/frames.hpp>

#include <cmath>           // for std::hypot, cos, sin, atan2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using std::string;

class PurePursuit : public rclcpp::Node
{
  public:
    PurePursuit();
    // Generate the command for the vehicle according to the current position and the waypoints
    void cmd_generator(nav_msgs::msg::Odometry odom);
    // Listen to the waypoints topic
    void waypoints_listener(nav_msgs::msg::Path path);
    void current_position_Callback(const turn_on_wheeltec_robot::msg::Position& msg);
    // Transform the pose to the base_link
    KDL::Frame trans2base(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Transform& tf);
    // Eucledian distance computation
    template<typename T1, typename T2>
    double distance(T1 pt1, T2 pt2)
    {
      return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
    }
    // Ros_spin.
    void run();
  private:
    // 辅助函数：停止车辆
    void stop_vehicle();
  
    // 辅助函数：发布到达消息
    void publish_arrival();
  
    // 辅助函数：统一发布命令（速度、TF、Marker）
    void publish_commands();
  
    // 辅助函数：查找预瞄点索引
    size_t findLookaheadPoint(const geometry_msgs::msg::Vector3 & current_pos);

  
    // 辅助函数：当路径结束时，沿终点方向外推预瞄点
    void computeLookaheadBeyondGoal(
      const geometry_msgs::msg::TransformStamped & tf,
      const geometry_msgs::msg::Pose & goal_pose);
  
    // 辅助函数：从 KDL::Frame 设置 lookahead_ transform
    void setLookaheadFromOffset(const KDL::Frame & offset);
 
  private:
    // Parameters
    double wheelbase;
    double lookahead_distance_, position_tolerance_;
    double v_max_, v_, w_max_;
    int idx_memory;
    unsigned idx_;
    bool goal_reached_, path_loaded_;

    nav_msgs::msg::Path path_;
    geometry_msgs::msg::Twist cmd_vel_;
    visualization_msgs::msg::Marker lookahead_marker_;
    
    // ROS
    //ros::Publisher pub_vel_, pub_acker_, pub_marker_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;  
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_arrival;  
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<turn_on_wheeltec_robot::msg::Position>::SharedPtr current_position_sub; 
    // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped lookahead_;
    string map_frame_id_, robot_frame_id_, lookahead_frame_id_;
    float distance1;    //障碍物距离
    float dis_angleX;    //障碍物方向,前面为0度角，右边为正，左边为负 
    float avoid_distance;
 
};

PurePursuit::PurePursuit() : rclcpp::Node("pure_pursuit"),v_max_(0.1), v_(v_max_), idx_(0), goal_reached_(false)
{
  // Get parameters from the parameter server
  this->declare_parameter<double>("lookahead_distance",0.6);
  this->get_parameter("lookahead_distance", lookahead_distance_);
  this->declare_parameter<double>("w_max",1.0);
  this->get_parameter("w_max", w_max_);
  this->declare_parameter<double>("v_max",0.1);
  this->get_parameter("v_max", v_max_);
  this->declare_parameter<double>("position_tolerance",0.1);
  this->get_parameter("position_tolerance", position_tolerance_);
  this->declare_parameter<std::string>("lookahead_frame_id","lookahead");
  this->get_parameter("lookahead_frame_id", lookahead_frame_id_);
  this->declare_parameter<double>("avoid_distance",0.3);
  this->get_parameter("avoid_distance", avoid_distance);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);


  map_frame_id_ = "map";
  robot_frame_id_ = "base_link";
  lookahead_.header.frame_id = robot_frame_id_;
  lookahead_.child_frame_id = lookahead_frame_id_;

  idx_memory = 0;
  path_loaded_ = false;
  distance1=100.0;
  dis_angleX=0.0; 
  RCLCPP_INFO(this->get_logger(),"init!");
  pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead", 10);
  pub_arrival = this->create_publisher<std_msgs::msg::Bool>("arrival", 10);
  sub_path_ = this->create_subscription<nav_msgs::msg::Path>("/waypoints", 5,std::bind(&PurePursuit::waypoints_listener, this,std::placeholders::_1));
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom_combined", 5,std::bind(&PurePursuit::cmd_generator, this,std::placeholders::_1));
  current_position_sub = this->create_subscription<turn_on_wheeltec_robot::msg::Position>("object_tracker/current_position", 5,std::bind(&PurePursuit::current_position_Callback, this,std::placeholders::_1));

}

void PurePursuit::cmd_generator(nav_msgs::msg::Odometry odom)
{
  if (!path_loaded_ || path_.poses.empty()) {
    stop_vehicle();
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(this->get_logger(), "TF lookup failed: " << ex.what());
    stop_vehicle();
    return;
  }

  // Current pose in map frame
  const auto & current_pos = tf.transform.translation;

  // Find lookahead point
  size_t target_idx = findLookaheadPoint(current_pos);

  // Check if reached goal
  if (target_idx >= path_.poses.size()) {
    const auto & goal_pose = path_.poses.back().pose;
    double dx = goal_pose.position.x - current_pos.x;
    double dy = goal_pose.position.y - current_pos.y;
    double dist_to_goal = std::hypot(dx, dy);

    if (dist_to_goal <= position_tolerance_) {
      goal_reached_ = true;
      stop_vehicle();
      publish_arrival();
      path_loaded_ = false;
      path_ = nav_msgs::msg::Path(); // clear path
      return;
    } else {
      // Extend beyond goal along final heading
      computeLookaheadBeyondGoal(tf, goal_pose);
    }
  } else {
    // Normal tracking: transform lookahead point to base_link
    KDL::Frame offset = trans2base(path_.poses[target_idx].pose, tf.transform);
    setLookaheadFromOffset(offset);
    idx_memory = target_idx;
  }

  // Avoidance override
  if (distance1 < avoid_distance) {
    stop_vehicle();
    publish_commands();
    return;
  }

  // Velocity command generation
  if (!goal_reached_) {
    v_ = copysign(v_max_, v_); // enforce max speed magnitude with direction

    double lateral_error = lookahead_.transform.translation.y;
    // Pure Pursuit curvature: κ = 2 * y / L^2  →  ω = v * κ = 2 * v * y / L^2
    double angular_velocity = (2.0 * v_ * lateral_error) / (lookahead_distance_ * lookahead_distance_);
    cmd_vel_.angular.z = std::clamp(angular_velocity, -w_max_, w_max_);
    cmd_vel_.linear.x = v_; // already clamped by copysign + v_max_
  } else {
    stop_vehicle();
  }

  publish_commands();
}

void PurePursuit::stop_vehicle()
{
  cmd_vel_.linear.x = 0.0;
  cmd_vel_.angular.z = 0.0;
}

void PurePursuit::publish_arrival()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  pub_arrival->publish(msg);
}

void PurePursuit::publish_commands()
{
  // Publish TF
  lookahead_.header.frame_id = "map";
  lookahead_.header.stamp = this->now();
  tf_broadcaster_->sendTransform(lookahead_);

  // Publish velocity
  pub_vel_->publish(cmd_vel_);

  // Publish marker
  lookahead_marker_.header.frame_id = "map";
  lookahead_marker_.header.stamp = this->now();
  lookahead_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  lookahead_marker_.action = visualization_msgs::msg::Marker::ADD;
  lookahead_marker_.scale.x = lookahead_marker_.scale.y = lookahead_marker_.scale.z = 0.1;
// Set orientation to identity (no rotation)
lookahead_marker_.pose.orientation.x = 0.0;
lookahead_marker_.pose.orientation.y = 0.0;
lookahead_marker_.pose.orientation.z = 0.0;
lookahead_marker_.pose.orientation.w = 1.0;

  lookahead_marker_.color.a = 1.0;

  if (!goal_reached_) {
    const auto & wp = path_.poses[idx_memory].pose.position;
    lookahead_marker_.id = static_cast<int>(idx_memory);
    lookahead_marker_.pose.position = wp;
    lookahead_marker_.color.r = 0.0; lookahead_marker_.color.g = 1.0; lookahead_marker_.color.b = 0.0;
  } else {
    lookahead_marker_.id = static_cast<int>(idx_memory++);
    lookahead_marker_.pose.position.x = lookahead_.transform.translation.x;
    lookahead_marker_.pose.position.y = lookahead_.transform.translation.y;
    lookahead_marker_.pose.position.z = lookahead_.transform.translation.z;
    lookahead_marker_.color.r = 1.0; lookahead_marker_.color.g = 0.0; lookahead_marker_.color.b = 0.0;
  }

  if (goal_reached_ && (idx_memory % 5 != 0)) return;
  pub_marker_->publish(lookahead_marker_);
}

size_t PurePursuit::findLookaheadPoint(const geometry_msgs::msg::Vector3 & current_pos)
{
  for (size_t i = idx_memory; i < path_.poses.size(); ++i) {
    double dx = path_.poses[i].pose.position.x - current_pos.x;
    double dy = path_.poses[i].pose.position.y - current_pos.y;
    if (std::hypot(dx, dy) > lookahead_distance_) {
      return i;
    }
  }
  return path_.poses.size();
}


void PurePursuit::computeLookaheadBeyondGoal(
    const geometry_msgs::msg::TransformStamped & tf,
    const geometry_msgs::msg::Pose & goal_pose)
{
  // Get final heading from last segment or goal orientation
  double goal_yaw;
  if (path_.poses.size() >= 2) {
    const auto & p1 = path_.poses[path_.poses.size() - 2].pose.position;
    const auto & p2 = goal_pose.position;
    goal_yaw = atan2(p2.y - p1.y, p2.x - p1.x);
  } else {
    tf2::Quaternion q(
        goal_pose.orientation.x,
        goal_pose.orientation.y,
        goal_pose.orientation.z,
        goal_pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, goal_yaw);
  }

  // Lookahead point: current + L * [cos(yaw), sin(yaw)]
  double x_ld = tf.transform.translation.x + lookahead_distance_ * cos(goal_yaw);
  double y_ld = tf.transform.translation.y + lookahead_distance_ * sin(goal_yaw);

  lookahead_.transform.translation.x = x_ld;
  lookahead_.transform.translation.y = y_ld;
  lookahead_.transform.translation.z = tf.transform.translation.z;

  tf2::Quaternion q_out;
  q_out.setRPY(0, 0, goal_yaw);
  tf2::convert(q_out, lookahead_.transform.rotation);
}

void PurePursuit::setLookaheadFromOffset(const KDL::Frame & offset)
{
  lookahead_.transform.translation.x = offset.p.x();
  lookahead_.transform.translation.y = offset.p.y();
  lookahead_.transform.translation.z = offset.p.z();
  offset.M.GetQuaternion(
      lookahead_.transform.rotation.x,
      lookahead_.transform.rotation.y,
      lookahead_.transform.rotation.z,
      lookahead_.transform.rotation.w);
}

void PurePursuit::waypoints_listener(nav_msgs::msg::Path new_path)
{ 
  if (new_path.header.frame_id == map_frame_id_)
  {
    path_ = new_path;
    idx_ = 0;
    if (new_path.poses.size() > 0)
    {
      RCLCPP_INFO(this->get_logger(),"Received Waypoints!");
      path_loaded_ = true;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),"Received empty waypoint!");
    }
  }
  else
  {
    RCLCPP_WARN_STREAM(this->get_logger(),"The waypoints must be published in the " << map_frame_id_ << " frame! Ignoring path in " << new_path.header.frame_id << " frame!");
  }
}

void PurePursuit::current_position_Callback(const turn_on_wheeltec_robot::msg::Position& msg)  
{
  distance1 = msg.distance;
  dis_angleX = msg.angle_x;   
}


KDL::Frame PurePursuit::trans2base(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Transform& tf)
{
  // Pose in map
  KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                        KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
  // base_link in map
  KDL::Frame F_map_tf(KDL::Rotation::Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w),
                      KDL::Vector(tf.translation.x, tf.translation.y, tf.translation.z));
                      
  return F_map_tf.Inverse()*F_map_pose;
}

void PurePursuit::run()
{
  while(rclcpp::ok()){
      rclcpp::spin_some(this->get_node_base_interface());
  }
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.00;
  msg.angular.z = 0.00;
  pub_vel_->publish(msg);
  
}

int main(int argc, char**argv)
{
  rclcpp::init(argc, argv);

  // PurePursuit controller;
  // controller.run();
  auto node = std::make_shared<PurePursuit>();
  rclcpp::spin(node);

  return 0;
}

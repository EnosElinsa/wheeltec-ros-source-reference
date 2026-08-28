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
  if (path_loaded_)
  {
    // Get the current pose
    geometry_msgs::msg::TransformStamped tf;
    try
    {
      tf = tf_buffer_->lookupTransform("map", "base_link",tf2::TimePoint());
      // Detetmine the waypoint to track on the basis of 1) current_pose 2) waypoints_info 3) lookahead_distance
      for (idx_=idx_memory; idx_ < path_.poses.size(); idx_++)
      {
        if (distance(path_.poses[idx_].pose.position, tf.transform.translation) > lookahead_distance_)
        {
          KDL::Frame pose_offset = trans2base(path_.poses[idx_].pose, tf.transform);
          lookahead_.transform.translation.x = pose_offset.p.x();
          lookahead_.transform.translation.y = pose_offset.p.y();
          lookahead_.transform.translation.z = pose_offset.p.z();
          pose_offset.M.GetQuaternion(lookahead_.transform.rotation.x, lookahead_.transform.rotation.y,
                                      lookahead_.transform.rotation.z, lookahead_.transform.rotation.w);
          idx_memory = idx_;
          break;
        }
      }
      // If approach the goal (last waypoint)
      if (!path_.poses.empty() && idx_ >= path_.poses.size())
      {
        KDL::Frame goal_offset = trans2base(path_.poses.back().pose, tf.transform);

        // Reach the goal
        if (fabs(goal_offset.p.x()) <= position_tolerance_)
        {
          goal_reached_ = true;
          path_ = nav_msgs::msg::Path(); // Reset the path
        }
        // Not meet the position tolerance: extend the lookahead distance beyond the goal
        else
        {
          // Find the  intersection between the circle of radius(lookahead_distance) centered at the current pose
          // and the line defined by the last waypoint
          double roll, pitch, yaw;
          goal_offset.M.GetRPY(roll, pitch, yaw);
          double k_end = tan(yaw); // Slope of line defined by the last waypoint
          double l_end = goal_offset.p.y() - k_end * goal_offset.p.x();
          double a = 1 + k_end * k_end;
          double b = 2 * l_end;
          double c = l_end * l_end - lookahead_distance_ * lookahead_distance_;
          double D = sqrt(b*b - 4*a*c);
          double x_ld = (-b + copysign(D,v_)) / (2*a);
          double y_ld = k_end * x_ld + l_end;
          
          lookahead_.transform.translation.x = x_ld;
          lookahead_.transform.translation.y = y_ld;
          lookahead_.transform.translation.z = goal_offset.p.z();
          goal_offset.M.GetQuaternion(lookahead_.transform.rotation.x, lookahead_.transform.rotation.y,
                                      lookahead_.transform.rotation.z, lookahead_.transform.rotation.w);
        }
      }
      // Waypoint follower
      if (!goal_reached_)
      {
        v_ = copysign(v_max_, v_);

        
        double lateral_offset = lookahead_.transform.translation.y;
        cmd_vel_.angular.z = std::min(2*v_/lookahead_distance_*lookahead_distance_*lateral_offset, w_max_);
        
        // Linear velocity
        cmd_vel_.linear.x = std::clamp(v_,-v_max_,v_max_);
      }
      // Reach the goal: stop the vehicle
      else
      {
        cmd_vel_.linear.x = 0.00;
        cmd_vel_.angular.z = 0.00;
        path_loaded_ = false;
        std_msgs::msg::Bool arrival;
        arrival.data = true;
        pub_arrival->publish(arrival);
      }
      // Publish the lookahead target transform.
      lookahead_.header.frame_id = "map";
      lookahead_.header.stamp = rclcpp::Node::now();
      tf_broadcaster_->sendTransform(lookahead_);
      // Publish the velocity command
      //avoid
      if(distance1<avoid_distance)
      {
        cmd_vel_.linear.x = 0.00;
        cmd_vel_.angular.z = 0.00;
      }
      pub_vel_->publish(cmd_vel_);
      // Publish the ackerman_steering command
      // Publish the lookahead_marker for visualization
      lookahead_marker_.header.frame_id = "map";
      lookahead_marker_.header.stamp = rclcpp::Node::now();
      lookahead_marker_.type = visualization_msgs::msg::Marker::SPHERE;
      lookahead_marker_.action = visualization_msgs::msg::Marker::ADD;
      lookahead_marker_.scale.x = 0.1;
      lookahead_marker_.scale.y = 0.1;
      lookahead_marker_.scale.z = 0.1;
      lookahead_marker_.pose.orientation.x = 0.0;
      lookahead_marker_.pose.orientation.y = 0.0;
      lookahead_marker_.pose.orientation.z = 0.0;
      lookahead_marker_.pose.orientation.w = 1.0;
      lookahead_marker_.color.a = 1.0;
      if (!goal_reached_)
      {
        lookahead_marker_.id = idx_;
        lookahead_marker_.pose.position.x = path_.poses[idx_].pose.position.x;
        lookahead_marker_.pose.position.y = path_.poses[idx_].pose.position.y;
        lookahead_marker_.pose.position.z = path_.poses[idx_].pose.position.z;
        lookahead_marker_.color.r = 0.0;
        lookahead_marker_.color.g = 1.0;
        lookahead_marker_.color.b = 0.0;
        pub_marker_->publish(lookahead_marker_);
      }
      else
      {
        lookahead_marker_.id = idx_memory;
        idx_memory += 1;
        lookahead_marker_.pose.position.x = tf.transform.translation.x;
        lookahead_marker_.pose.position.y = tf.transform.translation.y;
        lookahead_marker_.pose.position.z = tf.transform.translation.z;
        lookahead_marker_.color.r = 1.0;
        lookahead_marker_.color.g = 0.0;
        lookahead_marker_.color.b = 0.0;
        if (idx_memory%5 == 0)
        {
          pub_marker_->publish(lookahead_marker_); 
        }
      }
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN_STREAM(this->get_logger(),ex.what());
    }
  }
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

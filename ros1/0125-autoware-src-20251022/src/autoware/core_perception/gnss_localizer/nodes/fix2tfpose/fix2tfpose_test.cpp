/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <gnss/geo_pos_conv.hpp>
#include <sensor_msgs/Imu.h>

static ros::Publisher pose_publisher;

static ros::Publisher stat_publisher;
static std_msgs::Bool gnss_stat_msg;

static geometry_msgs::PoseStamped _prev_pose;
static geometry_msgs::Quaternion _quat;
//static double yaw;
double imu_yaw = 0;
static double yaw;
static int tmp=0;
// true if position history is long enough to compute orientation
static bool _orientation_ready = false;
static int _plane;
//geo.set_plane(_plane);
static double BASE_LATITUDE =0;    /* Base station latitude */;
static double BASE_LONGITUDE =0;   /* Base station longitude */;
static double BASE_HIGHT =0;      /* Base station longitude */;

static void IMUCallback(const sensor_msgs::ImuConstPtr &msg)
{
  // get the yaw value from the imu message
  tf::Quaternion q(
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // store the yaw value in the global variable
  imu_yaw = yaw;
}
static void GNSSCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  geo_pos_conv geo;
  geo.set_plane(_plane);
  geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);

  static tf::TransformBroadcaster pose_broadcaster;
  tf::Transform pose_transform;
  tf::Quaternion pose_q;
  
  if(tmp==0)
  {
  geometry_msgs::PoseStamped base_pose;
  BASE_LATITUDE= geo.x();
  BASE_LONGITUDE= geo.y();
  BASE_HIGHT= geo.z();
  tmp=1;
  }

  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = "map";
  pose.pose.position.x = geo.x()-BASE_LATITUDE;
  pose.pose.position.y = geo.y()-BASE_LONGITUDE;
  pose.pose.position.z = geo.z()-BASE_HIGHT;

  if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
  {
    gnss_stat_msg.data = false;
  }
  else
  {
    gnss_stat_msg.data = true;
  }

  double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
                         pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));
  std::cout << "distance: " << distance << std::endl;

  if (distance > 0.2)
  {
    // Update yaw based on IMU data
    // Assuming you have an IMU message called imu_msg with a field for yaw (e.g., imu_msg.orientation.yaw)
    yaw = imu_yaw;
    _quat = tf::createQuaternionMsgFromYaw(yaw);
    _prev_pose = pose;
    _orientation_ready = true;
  }

  if (_orientation_ready)
  {
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(imu_yaw);
  
    //pose.pose.orientation = _quat;
    pose_publisher.publish(pose);
    stat_publisher.publish(gnss_stat_msg);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    
    transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    q.setRPY(0, 0, yaw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "navsat_link"));
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fix2tfpose_test");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("plane", _plane);
  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
  stat_publisher = nh.advertise<std_msgs::Bool>("/gnss_stat", 1000);
  ros::Subscriber gnss_pose_subscriber = nh.subscribe("/gps/fix", 100, GNSSCallback);
  ros::Subscriber imu_subscriber = nh.subscribe("/imu_raw", 1, IMUCallback);
  ros::spin();
  return 0;
}

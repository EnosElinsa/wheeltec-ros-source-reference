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
static double plane_lat;
static double plane_lon;
static int _plane;
//geo.set_plane(_plane);
#define M_PI 3.1415926535897932384626433832795

static double wrapToPm(double a_num, const double a_max)
{
  if (a_num >= a_max)
  {
    a_num -= 2.0 * a_max;
  }
  return a_num;
}

static double wrapToPmPi(const double a_angle_rad)
{
  return wrapToPm(a_angle_rad, M_PI);
}

static void imuUpsideDown(const sensor_msgs::Imu::Ptr input)
{
  double input_roll, input_pitch, input_yaw;

  tf::Quaternion input_orientation;
  tf::quaternionMsgToTF(input->orientation, input_orientation);
  tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);
  input_yaw *= -1;
  input_yaw = input_yaw * M_PI/180.0;
  input->orientation = tf::createQuaternionMsgFromRollPitchYaw(input_roll, input_pitch, input_yaw);
}
static void IMUCallback(const sensor_msgs::Imu::Ptr input)
{
  imuUpsideDown(input);
  double imu_roll, imu_pitch, yaw;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(input->orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, yaw);
  imu_roll = wrapToPmPi(imu_roll);
  imu_pitch = wrapToPmPi(imu_pitch);
  yaw = wrapToPmPi(yaw);
  printf("yaw :%d\n",yaw);
  imu_yaw = yaw;
}
static void GNSSCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  geo_pos_conv geo;
  if(tmp==0)
  {
  plane_lat=(msg->latitude)*M_PI/180.0;
  plane_lon=(msg->longitude)*M_PI/180.0;
  tmp=1;
  }
  geo.set_plane(_plane);
  //geo.set_plane(plane_lat,plane_lon);
  geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);

  static tf::TransformBroadcaster pose_broadcaster;
  tf::Transform pose_transform;
  tf::Quaternion pose_q;

  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = "map";
  pose.pose.position.x = geo.y();
  pose.pose.position.y = geo.x();
  pose.pose.position.z = geo.z();

  if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
  {
    gnss_stat_msg.data = false;
   printf("x is %f\n,y is %f\n,z is %f\n",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
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
    //yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
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
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "gps"));
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fix2tfpose");
  ros::NodeHandle nh;  
  ros::NodeHandle private_nh("~");
  private_nh.getParam("plane", _plane);
  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);
  stat_publisher = nh.advertise<std_msgs::Bool>("/gnss_stat", 1000);
  ros::Subscriber gnss_pose_subscriber = nh.subscribe("/gps/fix", 10, GNSSCallback);
  ros::Subscriber imu_subscriber = nh.subscribe("/imu_raw", 100, IMUCallback);
  ros::spin();
  return 0;
}

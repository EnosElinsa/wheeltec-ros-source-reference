/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.h>
#endif

#include <string>

#include <sstream>
#include "../include/qt_ros_test/qnode.hpp"
#include <QObject>
#include <QDebug>


/*****************************************************************************
** Implementation
*****************************************************************************/
using std::placeholders::_1;

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {
    
}

QNode::~QNode() {
    if(rclcpp::ok()) {
      rclcpp::shutdown(); // explicitly needed since we use ros::start();
    }
	wait();
}

bool QNode::init() {
	rclcpp::init(init_argc,init_argv);
  nh = std::make_shared<rclcpp::Node>("qt_ros_test");
	if ( ! rclcpp::ok() ) {
    qDebug()<<"QNode::init failed!";
		return false;
	}
	// Add your ros communications here.
  init_pubsub();
  init_param();
	start();
	return true;
}

void QNode::run() {
    //auto imagecomp_sub = nh->create_subscription<sensor_msgs::msg::CompressedImage>(rgbtopic,5,std::bind(&QNode::image2_callback, this, std::placeholders::_1));
    //auto image_sub = nh->create_subscription<sensor_msgs::msg::Image>(depthtopic,5,std::bind(&QNode::image1_callback, this, std::placeholders::_1));
    rclcpp::Rate loop_rate(10);
	while ( rclcpp::ok() ) {
		rclcpp::spin_some(nh);
    loop_rate.sleep();

	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::init_pubsub()
{
    cmd_pub = nh->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1000);
    battery_sub = nh->create_subscription<std_msgs::msg::Float32>("/PowerVoltage", 5,std::bind(&QNode::batteryCallback, this, std::placeholders::_1));
    Cmd_Vel_Sub = nh->create_subscription<nav_msgs::msg::Odometry>("/odom", 5,std::bind(&QNode::speedCallback, this, std::placeholders::_1));
    sub_camera("/camera/color/image_raw/compressed");
}

void QNode::init_param()
{
    control_speed = 0; //#前进后退实际控制速度
    control_turn  = 0; //#转向实际控制速度
    control_HorizonMove = 0; //#横向移动实际控制速度

    // nh->declare_parameter<std::string>("rgbtopic","/camera/color/image_raw/compressed");
    // nh->get_parameter("rgbtopic", rgbtopic);
    // nh->declare_parameter<std::string>("depthtopic","/camera/depth/image_raw");
    // nh->get_parameter("depthtopic", depthtopic);
    // subcameratopic = QString::fromStdString(rgbtopic);
}

/*
 * 函数：按钮控制小车运动
 * car_model:0:麦轮小车  1：全向轮
 * keybutton：按钮返回值    u i o
 *                        j k l
 *                        m , .
 * speed:线速度
 * turn:角速度
*/
void QNode::pub_cmd(bool car_model, int keybutton, double speed, double turn)
{
    float x,th;
    switch (keybutton)
    {
    case 2:x=1;th=0;break;
    case 3:x=1;th=-1;break;
    case 4:x=0;th=1;break;
    case 6:x=0;th=-1;break;
    case 1:x=1;th=1;break;
    case 8:x=-1;th=0;break;
    case 9:
        if(car_model==0){x=-1;th=-1;}
        else if(car_model==1) {x=-1;th=1;}
        break;
    case 7:
        if(car_model==0){x=-1;th=1;}
        else if(car_model==1) {x=-1;th=-1;}
        break;
    case 5:x=0;th=0;break;
    default:x=0;th=0;break;
    }
    double target_speed = speed * x;
    double target_turn  = turn * th;
    double target_HorizonMove = speed*th;

    geometry_msgs::msg::Twist twist;
    if(car_model==0)
    {
        twist.linear.x  = target_speed;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = target_turn;
    }
    else
    {
        twist.linear.x  = target_speed;
        twist.linear.y = target_HorizonMove;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
    }
    qDebug()<<car_model<<keybutton<<speed<<turn<<target_speed<<target_turn<<target_HorizonMove;
    cmd_pub->publish(twist);
}

void QNode::sub_camera(QString cameratopic)
{
    subcameratopic = cameratopic;
    if(cameratopic.contains("compress"))
    {
      static auto imagecomp_sub = nh->create_subscription<sensor_msgs::msg::CompressedImage>(cameratopic.toStdString(),5,std::bind(&QNode::image2_callback, this, std::placeholders::_1));
      imagecomp_sub= nullptr;
      imagecomp_sub = nh->create_subscription<sensor_msgs::msg::CompressedImage>(cameratopic.toStdString(),5,std::bind(&QNode::image2_callback, this, std::placeholders::_1));
    }
    else
    {
      static auto image_sub = nh->create_subscription<sensor_msgs::msg::Image>(cameratopic.toStdString(),5,std::bind(&QNode::image1_callback, this, std::placeholders::_1));
      image_sub= nullptr;
      image_sub = nh->create_subscription<sensor_msgs::msg::Image>(cameratopic.toStdString(),5,std::bind(&QNode::image1_callback, this, std::placeholders::_1));    
    }
}
void QNode::image1_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    if(subcameratopic.contains("compress"))return;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      if(subcameratopic.contains("depth"))
      {
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
      }
      else  cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception & e)
    {
      qDebug()<<"image subscription error!!!"<<subcameratopic;  
      return; 
    }        
    QImage im=Mat2QImage(cv_ptr->image);
    emit image_val(im);
}

void QNode::image2_callback(const sensor_msgs::msg::CompressedImage &msg)
{
    if(!subcameratopic.contains("compress"))return;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      if(subcameratopic.contains("depth"))
      {
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
      }
      else  cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception & e)
    {
      qDebug()<<"image compress subscription error!!!"<<subcameratopic;  
      return; 
    }        
    QImage im=Mat2QImage(cv_ptr->image);
    emit image_val(im);
}

void QNode::batteryCallback(const std_msgs::msg::Float32::SharedPtr message)
{
    emit batteryState(message->data);
}

void QNode::speedCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    emit speedState(odom->twist.twist.linear.x,odom->twist.twist.linear.y,odom->twist.twist.linear.z,odom->twist.twist.angular.x,odom->twist.twist.angular.y,odom->twist.twist.angular.z);
}
QImage QNode::Mat2QImage(cv::Mat const& src)
{
  QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
  const float scale = 255.0;
  if (src.depth() == CV_8U) {
    if (src.channels() == 1) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          int level = src.at<quint8>(i, j);
          dest.setPixel(j, i, qRgb(level, level, level));
        }
      }
    } else if (src.channels() == 3) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
          dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
        }
      }
    }
  } else if(src.depth() == CV_16U){
      if (src.channels() == 1) {
        for (int i = 0; i < src.rows; ++i) {
          for (int j = 0; j < src.cols; ++j) {
            int level = (int)src.at<ushort>(i, j);
            dest.setPixel(j, i, qRgb(level, level, level));
          }
        }
      } else if (src.channels() == 3) {
        for (int i = 0; i < src.rows; ++i) {
          for (int j = 0; j < src.cols; ++j) {
            cv::Vec3b bgr = src.at<cv::Vec3b>(i, j)*65535;
            dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
          }
        }
      }
    }else if (src.depth() == CV_32F) {
    if (src.channels() == 1) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          int level = scale * src.at<float>(i, j);
          dest.setPixel(j, i, qRgb(level, level, level));
        }
      }
    } else if (src.channels() == 3) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
          dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
        }
      }
    }
  }

  return dest;
}




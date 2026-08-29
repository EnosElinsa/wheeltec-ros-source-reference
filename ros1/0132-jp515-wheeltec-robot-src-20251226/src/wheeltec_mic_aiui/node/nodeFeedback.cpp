/****************************************************************/
/* Copyright (c) 2025 WHEELTEC Technology, Inc   				*/
/* function:Functional node feedback							*/
/* 功能：功能节点反馈												*/
/****************************************************************/
#include <ros/ros.h>
#include <signal.h>
#include <stdlib.h>

#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include "feedback.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher cmdvel_pub;     		//速度信息发布者
ros::Publisher feedback_words_pub;	//语音反馈发布者
geometry_msgs::Twist cmd_vel_msg;   //速度控制信息数据

int laser_follow_flag = 0;    		//雷达跟随标志位
int visual_follow_flag = 0;	  		//色块跟随标志位
int rrt_flag = 0;	  				//自主建图标志位

/**************************************************************************
函数功能：雷达跟随开启成功标志位sub回调函数
入口参数：laser_follow_flag.msg  laser.py
返回  值：无
**************************************************************************/
void laser_follow_flagCallback(std_msgs::Int8 msg)
{
	laser_follow_flag = msg.data;
	
	if(laser_follow_flag == 1){
		feedback_text.data = "雷达跟随打开成功";
		feedback_words_pub.publish(feedback_text);
		std::cout<<"雷达跟随打开成功"<<std::endl;
	}
	printf("%d\n",laser_follow_flag);
}

/**************************************************************************
函数功能：自主建图开启成功标志位sub回调函数
入口参数：rrt_flag.msg  wait_for_fin.cpp
返回  值：无
**************************************************************************/
void rrt_flagCallback(std_msgs::Int8 msg)
{
	rrt_flag = msg.data;
	if(rrt_flag == 1){
		feedback_text.data = "已打开自主建图";
		feedback_words_pub.publish(feedback_text);
		std::cout<<"自主建图打开成功"<<std::endl;
	}	
	printf("%d\n",rrt_flag);
}

/**************************************************************************
函数功能：色块跟随开启成功标志位sub回调函数
入口参数：visual_follow_flag.msg  laser_follow.py
返回  值：无
**************************************************************************/
void visual_follow_flagCallback(std_msgs::Int8 msg)
{
	visual_follow_flag = msg.data;
	if(visual_follow_flag == 1){
		feedback_text.data = "已打开色块跟随";
		feedback_words_pub.publish(feedback_text);
		std::cout<<"色块跟随打开成功"<<std::endl;
	}
	printf("%d\n",visual_follow_flag);
}

void voice_choose_callback(const std_msgs::String& msg)
{
	/***指令***/
	std::string str1 = msg.data.c_str();    //取传入数据
	std::string str2 = "关闭雷达跟随";
	std::string str3 = "关闭色块跟随";
	std::string str4 = "关闭自主建图";
	std::string str5 = "关闭导航";
 
	if(str1 == str2){
		system("rosnode kill /follower");
		cmd_vel_msg.linear.x = 0;
		cmd_vel_msg.angular.z = 0;
		cmdvel_pub.publish(cmd_vel_msg);
		feedback_text.data = "已关闭雷达跟随";
		feedback_words_pub.publish(feedback_text);
		std::cout<<"已关闭雷达跟随"<<std::endl;
	}
	else if(str1 == str3){
		system("rosnode kill /follower");
		cmd_vel_msg.linear.x = 0;
		cmd_vel_msg.angular.z = 0;
		cmdvel_pub.publish(cmd_vel_msg);
		system("rosnode kill /usb_cam");
		system("rosnode kill /camera/camera");
		system("rosnode kill /visual_tracker");    
		// system("rosnode kill /camera/driver");
		// system("rosnode kill /camera/camera_nodelet_manager");
		// system("rosnode kill /camera/depth_metric");
		// system("rosnode kill /camera/depth_metric_rect");
		// system("rosnode kill /camera/depth_points");
		// system("rosnode kill /camera/depth_rectify_depth");
		// system("rosnode kill /camera/depth_registered_hw_metric_rect");
		// system("rosnode kill /camera/depth_registered_metric");
		// system("rosnode kill /camera/depth_registered_rectify_depth");
		// system("rosnode kill /camera/points_xyzrgb_hw_registered");
		// system("rosnode kill /camera/rgb_rectify_color");
		// system("rosnode kill /camera_base_link");
		// system("rosnode kill /camera_base_link1");
		// system("rosnode kill /camera_base_link2");
		// system("rosnode kill /camera_base_link3");
		Launch = gnome_terminal + simple_follower + " laserTracker.launch";
		system(Launch.c_str());
		feedback_text.data = "已关闭色块跟随";
		feedback_words_pub.publish(feedback_text);
		std::cout<<"已关闭色块跟随"<<std::endl;
	}
	else if(str1 == str4){
		system("rosnode kill /assigner");
		system("rosnode kill /save_map");
		system("rosnode kill /slam_gmapping");
		system("rosnode kill /wait_for_fin");
		system("rosnode kill /filter");
		system("rosnode kill /local_detector");
		system("rosnode kill /global_detector");
		system("rosnode kill /move_base");
		Launch = gnome_terminal + wheeltec_mic_aiui_ + "voi_navigation.launch";
		system(Launch.c_str());
		feedback_text.data = "已关闭自主建图";
		feedback_words_pub.publish(feedback_text);

		std::cout<<"已关闭自主建图"<<std::endl;
	}
	else if(str1 == str5){
		feedback_text.data = "好的";
		feedback_words_pub.publish(feedback_text);
		sleep(0.5);
		system("rosnode kill /send_mark_mic");
		system("rosnode kill /move_base");
		system("rosnode kill /map_server_for_test");
		system("rosnode kill /amcl");
		cmd_vel_msg.linear.x = 0;
		cmd_vel_msg.angular.z = 0;
		cmdvel_pub.publish(cmd_vel_msg);
		Launch = gnome_terminal + wheeltec_mic_aiui_ + "voi_navigation.launch";
		system(Launch.c_str());
		feedback_text.data = "已关闭导航";
		feedback_words_pub.publish(feedback_text);
		std::cout<<"已关闭导航"<<std::endl;
	}

} 

int main(int argc, char** argv)
{

	ros::init(argc, argv, "feedback_node");
	ros::NodeHandle nha; 

	nha.param("source_path", source_path, std::string("/home/wheeltec/wheeltec_robot/src"));

	/***创建节点关闭判断语句订阅者***/
	ros::Subscriber voice_choose_sub = nha.subscribe("voice_words",1,voice_choose_callback);
	ros::Subscriber laser_follow_flag_sub = nha.subscribe("laser_follow_flag", 1, laser_follow_flagCallback);//雷达跟随开启标志位订阅
	ros::Subscriber visual_follow_flag_sub = nha.subscribe("visual_follow_flag", 1, visual_follow_flagCallback);//视觉跟随开启标志位订阅
	ros::Subscriber rrt_flag_sub = nha.subscribe("rrt_flag", 1, rrt_flagCallback);//自主探索建图开启标志位订阅
	cmdvel_pub = nha.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	feedback_words_pub = nha.advertise<std_msgs::String>("feedback_words", 1);
	ros::spin();
	return 0;	
}


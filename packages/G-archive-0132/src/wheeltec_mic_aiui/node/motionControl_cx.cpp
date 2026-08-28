/************************************************************************************************/
/* Copyright (c) 2025 WHEELTEC Technology, Inc   												*/
/* function:Motion controller, Process multiple information sources and output cmd_vel(cx)		*/
/* 功能：运动控制器，对多个信息源进行处理并输出cmd_vel													*/
/************************************************************************************************/
#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <wheeltec_mic_aiui/position.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <feedback.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
//#include <move_base_msgs/MoveBaseAction.h>
 
ros::Publisher cmd_vel_Pub;    		//创建底盘速度控制话题发布者
ros::Publisher feedback_words_pub;	//语音反馈发布者
geometry_msgs::Twist cmd_vel_msg;   //速度控制信息数据
ros::Publisher akm_navigation_pub;

float distance1;    	//障碍物距离
float dis_angleX;    	//障碍物方向
float dis_angleX_change;
int angle = 0;    		//当前唤醒角度
int follow_flag = 0;    //寻找声源标志位
int cmd_vel_flag = 0;   //底盘控制器标志位
int goal_control=0;
int overmap_flag =0;

float radar_range ;
int radar_count ;
double direction;

std::string reach = "" ;

void gaol_Callback(std_msgs::Int8 msg)
{
	goal_control = msg.data;
}

void overmap_Callback(std_msgs::Int8 msg)
{
	overmap_flag = msg.data;
}
/**************************************************************************
函数功能：sub回调函数
入口参数：  laserTracker.py
返回  值：无
**************************************************************************/
void current_position_Callback(const wheeltec_mic_aiui::position& msg)	
{
	distance1 = msg.distance;
	dis_angleX = msg.angleX;

}

/**************************************************************************
函数功能：当前唤醒角度sub回调函数
入口参数：current_angle_msg  refresh_mic.cpp
返回  值：无
**************************************************************************/
void angle_Callback(std_msgs::Int32 msg)
{
	angle = msg.data;
}

/**************************************************************************
函数功能：底盘运动sub回调函数（原始数据）
入口参数：cmd_msg  command_recognition.cpp
返回  值：无
**************************************************************************/
void ori_vel_Callback(const geometry_msgs::Twist& msg)
{
	cmd_vel_msg.linear.x = msg.linear.x;
	cmd_vel_msg.angular.z = msg.angular.z; 
	
}

/**************************************************************************
函数功能：寻找声源标志位sub回调函数
入口参数：follow_flag_msg  command_recognition.cpp
返回  值：无
**************************************************************************/
void follow_flag_Callback(std_msgs::Int8 msg)
{
	follow_flag = msg.data;

}

/**************************************************************************
函数功能：底盘控制器标志位sub回调函数
入口参数：cmd_vel_flag_msg  command_recognition.cpp
返回  值：无
**************************************************************************/
void cmd_vel_flag_Callback(std_msgs::Int8 msg)
{
	cmd_vel_flag = msg.data;

}

/**************************************************************************
函数功能：中断程序
入口参数：
返回  值：无
**************************************************************************/
void shutdown(int sig)
{
	cmd_vel_Pub.publish(geometry_msgs::Twist());
	ROS_INFO("motion_control.cpp ended!");
	ros::shutdown();
}
 
void pose_Callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	double orientation_z;    
	double orientation_w;
	double direction_r;
	orientation_z = msg.pose.pose.orientation.z;    //»ñÈ¡×îÐÂµÄÀï³ÌŒÆ·œÎ»Öµ
	orientation_w = msg.pose.pose.orientation.w;

	//printf("z = %f-----\n",orientation_z);
	//printf("w = %f-----\n",orientation_w);
	direction_r = atan2(2*(orientation_w*orientation_z),1-2*(pow(orientation_z,2.0)));

	direction = direction_r*180/3.1415926;
	if(direction<0)
		{direction+=360;}
}

void goal_reach_Callback(const move_base_msgs::MoveBaseActionResult& msg)
{
	reach = msg.status.text;
	if(reach == "Goal reached." && goal_control==1)//I、J、K点到达播报
	{
		feedback_text.data = "已到达目标点";
		feedback_words_pub.publish(feedback_text);
		goal_control=0;
	}
	else if(reach == "Goal reached." && overmap_flag==1)//建图完成播报
	{
		feedback_text.data = "已完成建图";
		feedback_words_pub.publish(feedback_text);
		overmap_flag=0;
	}

}
/**************************************************************************
函数功能：寻找声源控制转向部分
入口参数：angle
返回  值：无
**************************************************************************/
void follow_turn(int angle)
{
	int ticks;    //计数变量
	float angular_turn_msg = 0.5;  //转向速度弧度制
	float angular_duration;    //转向时间
	float rate1 = 50;    //控制频率
	ros::Rate loopRate1(rate1);

	printf("angle =%d\n",angle);
	cmd_vel_msg.linear.x = 0;    //保持X轴方向速度为0
	
	/***控制不同方向转向***/
	if(angle<=180)
	{
		cmd_vel_msg.angular.z = angular_turn_msg;
	}
	else
	{
		angle=360-angle;
		cmd_vel_msg.angular.z = -angular_turn_msg;
	}
	
	angular_duration = angle / angular_turn_msg /180 * 3.14159;    //计算转弯时间
	printf("time =%f\n",angular_duration);
	ticks = int(angular_duration * rate1);    //计算控制次数
	printf("ticks =%d\n",ticks);

	/***控制转向***/
	for(int i = 0; i < ticks + 15 ; i++)
	{
		cmd_vel_Pub.publish(cmd_vel_msg); // 将速度指令发送给机器人
		loopRate1.sleep();    //控制频率50Hz
		printf("i =%d\n",i);
	}
	cmd_vel_msg.angular.z = 0;    //速度置零
	cmd_vel_Pub.publish(geometry_msgs::Twist());
}

void akm_follow_turn(int angle)
{
	float angle_r;
	float goal_angle_r;
	float x_to_robot;
	float y_to_robot;
	tf::TransformListener listener;

	angle_r=angle*3.1415926/180;
	//geometry_msgs::PoseStamped robot_point;
	geometry_msgs::PointStamped robot_point;
	geometry_msgs::PointStamped map_point;
	robot_point.header.frame_id = "base_footprint";
	robot_point.header.stamp = ros::Time();
    robot_point.point.x = cos(angle_r);
    robot_point.point.y = sin(angle_r);
    robot_point.point.z = 0.0;	 

	try{
		listener.waitForTransform("map","base_footprint",ros::Time(0),ros::Duration(3.0));
        listener.transformPoint("map", robot_point, map_point);
       }

    /*
    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
    laser_point.point.x, laser_point.point.y, laser_point.point.z,
    base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    }
    */
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"laser\" to \"base_link\": %s", ex.what());
    }

	//direction
	goal_angle_r= (direction+angle) /180*3.1415926;
    //goal_angle_r= (angle-direction) /180*3.1415926;
	printf("goal_angle_r=%f\n",goal_angle_r);
    geometry_msgs::PoseStamped map_pose;
	map_pose.header.seq = 0;
	map_pose.header.frame_id = "map";
	map_pose.pose.orientation.x=0;
	map_pose.pose.orientation.y=0;
	map_pose.pose.orientation.z=sin(goal_angle_r/2);
	map_pose.pose.orientation.w=cos(goal_angle_r/2);

	map_pose.pose.position.x = map_point.point.x;
	map_pose.pose.position.y = map_point.point.y;
	map_pose.pose.position.z = 0;
	akm_navigation_pub.publish(map_pose);
	ros::Time start_time = ros::Time::now();

	ros::Duration timeout(15.0); // 设置15秒超时
	bool goal_reached = false;
	while(ros::ok() && !goal_reached && (ros::Time::now() - start_time < timeout))
	{
		ros::spinOnce(); // 处理回调
		
		// 检查是否到达目标
		if(reach == "Goal reached.")
		{
			goal_reached = true;
			std::cout<<"目标已到达"<<std::endl;
		}
		else
		{
			ros::Duration(0.1).sleep();
		}
	}
	// 检查超时情况
	if(!goal_reached)
	{
		ROS_WARN("跟随声源导航超时");
	}
	reach = "";
}

/**************************************************************************
函数功能：判断障碍物距离是否小于0.75米
入口参数：无
返回  值：1或0
**************************************************************************/
int distance_judgment(void)
{
	if(distance1<=radar_range) 
		return 1;
	else
		return 0;
	
}
 
/**************************************************************************
函数功能：判断障碍物方向是否在小车运动趋势方向上
入口参数：无
返回  值：1或0
**************************************************************************/
int dis_angleX_judgment(void)
{

	if(dis_angleX>=-1.57)
	 	dis_angleX_change=dis_angleX-1.57;
	else
	 	dis_angleX_change=dis_angleX+1.57;
	 	
	 	
	if(cmd_vel_msg.linear.x > 0 && (dis_angleX > 1.57 || dis_angleX < -1.57))
		return 1;

	else if(cmd_vel_msg.linear.x < 0 && dis_angleX > -1.57 && dis_angleX < 1.57)
		return 1;
		
	else if(cmd_vel_msg.angular.z > 0 && dis_angleX < 0)
		return 1;
		
	else if(cmd_vel_msg.angular.z <0 && dis_angleX > 0)
		return 1;
		
	else 
		return 0;
		
}

/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(int argc, char** argv)
{
	int turn_fin_flag=0;    //完成转向标志位
	int temp_count = 0;    //计数变量
	std::string str1 = "遇到障碍物";    //障碍物字符串
	std::string if_akm ;
	int i;

	ros::init(argc, argv, "motion_ctrl");    //初始化ROS节点

	ros::NodeHandle node;    //创建句柄

	/***创建底盘速度控制话题发布者***/
	cmd_vel_Pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	/***创建当I、J、K点到达标志位话题订阅者***/
	ros::Subscriber goal_sub = node.subscribe("goal_control_flag", 1, gaol_Callback);//
	/***创建建图完成标志位话题订阅者***/
	ros::Subscriber overmap_sub = node.subscribe("overmap_flag", 1, overmap_Callback);
	/***创建当前唤醒角度话题订阅者***/
	ros::Subscriber angle_sub = node.subscribe("/mic/awake/angle", 1, angle_Callback);
	/***创建底盘运动话题订阅者***/
	ros::Subscriber vel_sub = node.subscribe("ori_vel", 1, ori_vel_Callback);
	/***创建寻找声源标志位话题订阅者***/
	ros::Subscriber follow_flag_sub = node.subscribe("follow_flag", 1, follow_flag_Callback);
	/***创建底盘运动控制器标志位话题订阅者***/
	ros::Subscriber cmd_vel_flag_sub = node.subscribe("cmd_vel_flag", 1, cmd_vel_flag_Callback);
  	/***创建障碍物方位话题订阅者***/
	ros::Subscriber current_position_sub = node.subscribe("/object_tracker/current_position", 1, current_position_Callback);
	ros::Subscriber goal_reach_sub = node.subscribe("/move_base/result", 1, goal_reach_Callback);
	ros::Subscriber pose_sub = node.subscribe("/robot_pose_ekf/odom_combined",1,pose_Callback);	

	/***创建离线命令词识别结果话题发布者***/
	ros::Publisher control1 = node.advertise<std_msgs::String>("voice_words", 10);
	ros::Publisher cmd_vel_flag_pub = node.advertise<std_msgs::Int8>("cmd_vel_flag",1);
	akm_navigation_pub = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
	feedback_words_pub = node.advertise<std_msgs::String>("feedback_words", 1);

	//execute a shutdown function when exiting
	signal(SIGINT, shutdown);
	ROS_INFO("motion_control.cpp start...");

	double rate2 = 10;    //频率10Hz
	ros::Rate loopRate2(rate2);

	node.param<float>("/radar_range", radar_range, 0.75);
	node.param("/radar_count", radar_count, 3);
	node.param("/if_akm_yes_or_no", if_akm, std::string("no"));

	while(ros::ok())
	{
		if(follow_flag)    //寻找声源转向部分
		{
			if (if_akm=="yes")
			{
				akm_follow_turn(angle);
			}
			else
			{
				follow_turn(angle);
			}
				follow_flag = 0;
				turn_fin_flag = 1;
				cmd_vel_msg.angular.z = 0;
				cmd_vel_msg.linear.x = 0.2;	
		}

		if(cmd_vel_flag || turn_fin_flag)    //底盘运动控制部分
		{
			cmd_vel_Pub.publish(cmd_vel_msg);    //将速度指令发送给机器人

			if(cmd_vel_msg.linear.x ==0 && cmd_vel_msg.angular.z == 0)
			{
				std_msgs::Int8 cmd_vel_flag_msg;
        			cmd_vel_flag_msg.data = 0;
       				cmd_vel_flag_pub.publish(cmd_vel_flag_msg);
			}

			if(distance_judgment() && dis_angleX_judgment())    //判断障碍物的距离和方向
			{
				temp_count++;
				if(temp_count > radar_count)    //连续计数5️次停止运动防止碰撞，避免雷达有噪点
				{
					if(turn_fin_flag == 0)    //非寻找声源控制遇到障碍物则向用户界面发送提醒
					{
						std_msgs::String count_msg;
						count_msg.data = str1;
						control1.publish(count_msg);
						cmd_vel_msg.angular.z = 0;
						cmd_vel_msg.linear.x = 0;    //速度置零
						cmd_vel_Pub.publish(geometry_msgs::Twist());
						cmd_vel_flag = 0;    //各标志位置零
						turn_fin_flag = 0;
						temp_count = 0;
					}
					else
					{
						cmd_vel_msg.angular.z = 0;
						cmd_vel_msg.linear.x = 0;    //速度置零
						cmd_vel_Pub.publish(geometry_msgs::Twist());
						cmd_vel_flag = 0;    //各标志位置零
						turn_fin_flag = 0;
						temp_count = 0;
						feedback_text.data = "已找到障碍物";
						feedback_words_pub.publish(feedback_text);
					}
				}
			}
			else temp_count = 0;    //排除雷达噪点
		}
		ros::spinOnce();
		loopRate2.sleep();
	} 
	//return 0;
}


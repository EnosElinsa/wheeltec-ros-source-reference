#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include<iostream>
#include <std_msgs/Float64.h>
using namespace std;
class wheeltec_joy
{
public:
    wheeltec_joy();
    std_msgs::Float64 vlinear_x; //默认值
    std_msgs::Float64 vlinear_z;
private:
    void callback(const sensor_msgs::Joy::ConstPtr& Joy); 
    //实例化节点
    ros::NodeHandle n; 
    ros::Subscriber sub ;
    ros::Publisher pub ;
    //控制乌龟的速度，是通过这两个变量调整
    double vlinear,vangular;
    //手柄键值
    int axis_ang,axis_lin; 
    int dir;

};

wheeltec_joy::wheeltec_joy() 
{
   //读取参数服务器中的变量值
   ros::NodeHandle private_nh("~"); //创建节点句柄
   private_nh.param<int>("axis_linear",axis_lin,1); //默认axes[1]接收速度
   private_nh.param<int>("axis_angular",axis_ang,0);//默认axes[0]接收角度
   private_nh.param<double>("vlinear",vlinear,2);//默认线速度1 m/s
   private_nh.param<double>("vangular",vangular,1);//默认角速度1 单位rad/s

   pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);//将速度发给乌龟
   sub = n.subscribe<sensor_msgs::Joy>("joy",10,&wheeltec_joy::callback,this); //订阅手柄发来的数据
} 

void wheeltec_joy::callback(const sensor_msgs::Joy::ConstPtr& Joy) //键值回调函数
 {
   double vangle_key,linera_key;
   double acce_x,acce_z;
   geometry_msgs::Twist v;

   vangle_key =Joy->axes[0];  //获取axes[0]的值
   linera_key =Joy->axes[1];  //获取axes[1]的值

   acce_x=Joy->axes[4]+1.0;   //读取右摇杆的值对海龟的线速度进行加减速处理
   acce_z=Joy->axes[3]+1.0;   //读取右摇杆的值对海龟的角速度进行加减速处理
   //判断前进后退
   if(linera_key>0)  
   {
   dir=1;
   vlinear_x.data=vlinear;          
   }
   else if(linera_key<0)
   {
   dir=-1;
   vlinear_x.data=-vlinear;          
   } 
   else  vlinear_x.data=0;
   //判断左转右转，大于0为左转，小于0为右转
   if(vangle_key>0)  vlinear_z.data=vangular;
   else if(vangle_key<0) vlinear_z.data=-vangular; 
   else vlinear_z.data=0;
   //处理数据
   v.linear.x = vlinear_x.data*acce_x;
   v.angular.z = dir*vlinear_z.data*acce_z;
   //打印输出
   ROS_INFO("linear:%.3lf angular:%.3lf",vlinear_x.data,v.angular.z);
   pub.publish(v);

}

int main(int argc,char** argv)
{
  ros::init(argc, argv, "joy_control");
  wheeltec_joy teleop_turtle;
  ros::spin();
  return 0;

}

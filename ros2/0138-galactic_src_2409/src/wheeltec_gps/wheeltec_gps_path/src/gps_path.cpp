
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <math.h>
#define EARTH_RADIUS 6378.137
using std::placeholders::_1;
bool pose_init;

class GpsPath : public rclcpp::Node
{
  public:
    GpsPath()
    : Node("GpsPath")
    {
      state_pub_ = create_publisher<nav_msgs::msg::Path>("gps_path", 10);
      subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fix", 1, std::bind(&GpsPath::gps_callback, this, _1));
    }
    struct lla_pose
    {
      double latitude;
      double longitude;
      double altitude;
    };
    double rad(double d) 
    {
    	return d * 3.1415926 / 180.0;
    }
  
  private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg) 
    {
      if(!pose_init)
       {
        init_pose.latitude = gps_msg->latitude;
        init_pose.longitude = gps_msg->longitude;
        init_pose.altitude = gps_msg->altitude;
        pose_init = true;
        }
      else
        {
        //计算相对位置
            double radLat1 ,radLat2, radLong1,radLong2,delta_lat,delta_long,x,y;
            
            radLat1 = rad(init_pose.latitude);
            radLong1 = rad(init_pose.longitude);
            
            radLat2 = rad(gps_msg->latitude);
            radLong2 = rad(gps_msg->longitude);
                //计算x
            delta_lat = radLat2 - radLat1;
            delta_long = 0;
            
            
            if(delta_lat>0)
              x = -2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));
            else
              x = -2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));
                x = x*EARTH_RADIUS*1000;
        
                //计算y
            delta_lat = 0;
                delta_long = radLong2  - radLong1;
            if(delta_long>0)
              y = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
            else
              y = -2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
               y = y*EARTH_RADIUS*1000;
    
            //计算z
            double z = gps_msg->altitude - init_pose.altitude;
    
            //发布轨迹
            ros_path_.header.frame_id = "path";
            ros_path_.header.stamp = rclcpp::Node::now();  
            geometry_msgs::msg::PoseStamped pose;
            pose.header = ros_path_.header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            ros_path_.poses.push_back(pose);
            
            state_pub_->publish(ros_path_);
    
            RCLCPP_INFO(this->get_logger(),"( x:%0.6f ,y:%0.6f ,z:%0.6f)",x ,y ,z );
    
            //
        }        
        
        
    }
    
    nav_msgs::msg::Path ros_path_;
    lla_pose init_pose;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  pose_init = false;
  rclcpp::spin(std::make_shared<GpsPath>());
  rclcpp::shutdown();
  return 0;
}




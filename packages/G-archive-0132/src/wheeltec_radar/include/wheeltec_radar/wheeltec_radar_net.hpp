#ifndef __WHEELTEC_RADAR_HPP_
#define __WHEELTEC_RADAR_HPP_

#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h> 
#include <stdlib.h>    
#include <unistd.h> 

#include "ros/ros.h"                    // ROS 1 核心头文件
#include <sys/stat.h>
#include <fcntl.h>          
#include <stdbool.h>

#include <sensor_msgs/PointCloud2.h>

#include <sys/types.h>

#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <wheeltec_radar/RadarDetection.h>
#include <wheeltec_radar/RadarDetectionArray.h>
#include <wheeltec_radar/RadarTrack.h>
#include <wheeltec_radar/RadarTrackArray.h>


#include <vector>
#include <random>

using namespace std;

using sensor_msgs::PointCloud2;

class turn_on_radar
{
public:
    turn_on_radar();  // 构造函数
    ~turn_on_radar(); // 析构函数

    void Get_Data();
 //   pcl::PointXYZI getRadarPointXYZI(const wheeltec_radar::RadarReturn & radar, float intensity);
    sensor_msgs::PointCloud2 toPointcloud2(const wheeltec_radar::RadarDetectionArray & radar_scan);
   int ReadFromIO(uint8_t *rx_buf, uint32_t rx_buf_len); 
   bool WriteToIo(const uint8_t *tx_buf, uint32_t tx_buf_len, uint32_t *tx_len);

   
    uint8_t Receive_Data[17];        // 接收缓冲区
    ros::NodeHandle* node_handle;    // 指针形式保留（可改为直接成员）

  std::string devip_str_;
  int cur_rpm_;
  int return_mode_;
  bool npkt_update_flag_;
  bool add_multicast;
  std::string group_ip;
  int UDP_PORT_NUMBER_DIFOP;
  int sid;
  int socket_id_difop;
  int sockfd_;
  std::string devip_str_difop;
    in_addr devip_;
  in_addr devip_difop;

    ros::Publisher pos_Pub;
    ros::Publisher track_Pub;
    ros::Publisher pub_pointcloud_;

    wheeltec_radar::RadarDetectionArray scan_msg;
    wheeltec_radar::RadarTrackArray tracks_msg;
};

#endif // __WHEELTEC_RADAR_HPP_

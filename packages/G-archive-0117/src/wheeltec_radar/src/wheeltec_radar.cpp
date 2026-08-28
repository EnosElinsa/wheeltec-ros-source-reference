#include "wheeltec_radar/wheeltec_radar.hpp"
#include "geometry_msgs/Point32.h"

// 全局节点句柄（用于发布）
ros::NodeHandle* nh_ptr = nullptr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turn_on_radar");
    turn_on_radar radar;
    radar.Get_Data();  // 进入数据采集循环
    return 0;
}

sensor_msgs::PointCloud2 turn_on_radar::toPointcloud2(const wheeltec_radar::RadarDetectionArray & radar_scan)
{
  pcl::PointCloud<pcl::PointXYZI> pcl;
  for (const auto & radar : radar_scan.detections) {
    pcl::PointXYZI point;
    point.x = radar.position.x;
    point.y = radar.position.y;
    point.z = 0;
    point.intensity = 0;
    pcl.push_back(point);
  }
  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(pcl, pointcloud_msg);
  pointcloud_msg.header = radar_scan.header;
  return pointcloud_msg;
}
geometry_msgs::Point32 createPoint32(const double x, const double y)
{
  geometry_msgs::Point32 p;
  p.x = x;
  p.y = y;
  p.z = 0.0;

  return p;
}

void turn_on_radar::Get_Data()
{
  while(ros::ok())
  {
    uint8_t Receive_Data_Pr[1];
    static int count = 0;

    try {
      radar_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr));
    } catch (const std::exception& e) {
      //ROS_ERROR("Serial read failed: %s", e.what());
      continue;
    }

    Receive_Data[count] = Receive_Data_Pr[0];

    if(Receive_Data_Pr[0] == 0xAA || count > 0)
      count++;
    else
      count = 0;

    if(count == 14)
    {
      count = 0;
      if(Receive_Data[12] == 0x55 && Receive_Data[13] == 0x55)
      {
        if(Receive_Data[3] == 0x06 && Receive_Data[2] == 0x0b)
        {
          float datay = (Receive_Data[5] * 32 + (Receive_Data[6] >> 3)) * 0.1 - 500;
          float datax = (((Receive_Data[6] & 0x07) * 256 + Receive_Data[7]) * 0.1 - 102.3);    
          float datavelocity_y = (Receive_Data[8]<<2+(Receive_Data[9]>>6))*0.25-128;
          float datavelocity_x = ((Receive_Data[9]&0x3F)*8+(Receive_Data[10]>>5))*0.25-64;
          //ROS 右手坐标系
          float xx = datay;
          float yy = -datax;
          float velocity_x = datavelocity_y;
          float velocity_y = -datavelocity_x;
          float dis = sqrt(xx * xx + yy * yy);
          float angle = atan2(yy, xx);

          wheeltec_radar::RadarDetection radardetection;
          radardetection.detection_id = Receive_Data[4];
          radardetection.position.x = xx;
          radardetection.position.y = yy;
          radardetection.velocity.x = velocity_x;
          radardetection.velocity.y = velocity_y;
          radardetection.amplitude = 0;
          scan_msg.detections.push_back(radardetection);
          
          wheeltec_radar::RadarTrack out_object;
          out_object.track_id = Receive_Data[4];
          out_object.track_shape.points.push_back(createPoint32(yy-0.1, xx));
          out_object.track_shape.points.push_back(createPoint32(yy, xx+0.1));
          out_object.track_shape.points.push_back(createPoint32(yy+0.1, xx));
          out_object.track_shape.points.push_back(createPoint32(yy, xx-0.1));
          //out_object.track_shape.points[0].y = yy;
          //out_object.track_shape.points[0].x = xx;
          out_object.linear_velocity.y = velocity_x;
          out_object.linear_velocity.x = velocity_y;
          tracks_msg.tracks.push_back(out_object);

        }
        else if(Receive_Data[3] == 0x06 && Receive_Data[2] == 0x0a)
        {
          scan_msg.header.stamp = ros::Time::now();
          scan_msg.header.frame_id = "radar";
          pos_Pub.publish(scan_msg);

          pub_pointcloud_.publish(toPointcloud2(scan_msg));

          tracks_msg.header.stamp = ros::Time::now();
          tracks_msg.header.frame_id = "radar";
          track_Pub.publish(tracks_msg);

          scan_msg.detections.clear();
          tracks_msg.tracks.clear();
        }
      }
    }

//    ros::spinOnce();  // 必须调用，否则订阅器和发布器不工作
  }
}

// 构造函数
turn_on_radar::turn_on_radar()
{
  ros::NodeHandle nh("~");  // 创建 NodeHandle

  // 获取参数
  nh.param<int>("serial_baud_rate", serial_baud_rate, 115200);
  nh.param<std::string>("usart_port_name", usart_port_name, "/dev/ttyUSB0");

  // 创建发布者
  pos_Pub = nh.advertise<wheeltec_radar::RadarDetectionArray>("/radarscan", 10);
  track_Pub = nh.advertise<wheeltec_radar::RadarTrackArray>("/radartrack", 10);
  pub_pointcloud_ = nh.advertise<sensor_msgs::PointCloud2>("/radarpointcloud", 10);

  try
  {
    radar_Serial.setPort(usart_port_name);
    radar_Serial.setBaudrate(serial_baud_rate);
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
    radar_Serial.setTimeout(_time);
    radar_Serial.open();
    radar_Serial.flushInput();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR("wheeltec_radar can not open serial port %s, Please check the serial port cable!",usart_port_name.c_str());
  }

  if (radar_Serial.isOpen())
  {
    ROS_INFO("wheeltec_radar serial port opened");
  }
}

// 析构函数
turn_on_radar::~turn_on_radar()
{
  radar_Serial.close();
  ROS_INFO("Shutting down");
}

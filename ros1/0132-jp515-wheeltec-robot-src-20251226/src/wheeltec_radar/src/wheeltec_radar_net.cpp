#include "wheeltec_radar/wheeltec_radar_net.hpp"
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
  int count = 0;
  while(ros::ok())
  {
    try {
      count = ReadFromIO(Receive_Data,2048);
    } catch (const std::exception& e) {
      //ROS_ERROR("Serial read failed: %s", e.what());
      continue;
    }

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
int turn_on_radar::ReadFromIO(uint8_t *rx_buf, uint32_t rx_buf_len) {
        double time1 = ros::Time::now().toSec();
        int q = 0;
        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN;
        static const int POLL_TIMEOUT = 2000; // one second (in msec)

        sockaddr_in sender_address;
        socklen_t sender_address_len = sizeof(sender_address);
        // while (true)
        while (ros::ok())
        {
            // poll() until input available
            do
            {
                int retval = poll(fds, 1, POLL_TIMEOUT);
                if (retval < 0) // poll() error?
                {
                    if (errno != EINTR)
                        ROS_ERROR("poll() error: %s", strerror(errno));
                    return 0;
                }
                if (retval == 0) // poll() timeout?
                {
                    ROS_WARN("lslidar poll() timeout");
                    return 0;
                }
                if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) // device error?
                {
                    ROS_ERROR("poll() reports lslidar error");
                    return 0;
                }
            } while ((fds[0].revents & POLLIN) == 0);

            // Receive packets that should now be available from the
            // socket using a blocking read.
            ssize_t nbytes = recvfrom(sockfd_, rx_buf, rx_buf_len, 0,
                                      (sockaddr *)&sender_address, &sender_address_len);
            //        ROS_DEBUG_STREAM("incomplete lslidar packet read: "
            //                         << nbytes << " bytes");
            q = (int)nbytes;
            if (nbytes < 0)
            {
                if (errno != EWOULDBLOCK)
                {
                    perror("recvfail");
                    ROS_INFO("recvfail");
                    return 1;
                }
            }
            else if ((size_t)nbytes <= rx_buf_len || (size_t)nbytes >= 50)
            {

                // read successful,
                // if packet is not from the lidar scanner we selected by IP,
                // continue otherwise we are done
                if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
                    continue;
                else
                    break; // done
            }
            
        }
        // if (flag == 0)
        // {
        //     abort();
        // }
        // this->getFPGA_GPSTimeStamp(packet);

        // Average the times at which we begin and end reading.  Use that to
        // estimate when the scan occurred.
        double time2 = ros::Time::now().toSec();
        // packet->stamp = ros::Time((time2 + time1) / 2.0);
        // packet->stamp = this->timeStamp;
        //for(int i=0;i<rx_buf_len;i++)printHexData(rx_buf[i]);
        return q;
}

bool turn_on_radar::WriteToIo(const uint8_t *tx_buf, uint32_t tx_buf_len,
                                  uint32_t *tx_len) {
        sockaddr_in server_sai;
        server_sai.sin_family = AF_INET; // IPV4 协议族
        server_sai.sin_port = htons(UDP_PORT_NUMBER_DIFOP);
        server_sai.sin_addr.s_addr = inet_addr(devip_str_.c_str());
        int rtn = 0;
        rtn = sendto(sockfd_, tx_buf, tx_buf_len, 0, (struct sockaddr *)&server_sai, sizeof(struct sockaddr));
        if (rtn < 0)
        {
            printf("start scan error !\n");
        }
        else
        {
            usleep(3000000);
            return 1;
        }
        return 0;
}


// 构造函数
turn_on_radar::turn_on_radar()
{
  ros::NodeHandle nh("~");  // 创建 NodeHandle

  // 获取参数
  nh.param("device_ip", devip_str_, std::string("192.168.1.200"));
  nh.param("device_ip_difop", devip_str_difop, std::string("192.168.1.102"));
  nh.param<bool>("add_multicast", add_multicast, false);
  nh.param<std::string>("group_ip", group_ip, "224.1.1.2");
  nh.param<int>("device_port", UDP_PORT_NUMBER_DIFOP, 2369);
  nh.param<int>("difop_port", sid, 2368);

  if (!devip_str_.empty())
      ROS_INFO_STREAM("Only accepting packets from IP address: " << devip_str_);

  // 创建发布者
  pos_Pub = nh.advertise<wheeltec_radar::RadarDetectionArray>("/radarscan", 10);
  track_Pub = nh.advertise<wheeltec_radar::RadarTrackArray>("/radartrack", 10);
  pub_pointcloud_ = nh.advertise<sensor_msgs::PointCloud2>("/radarpointcloud", 10);

  try
  {
    sockfd_ = -1;
    if (!devip_str_.empty())
    {
        inet_aton(devip_str_.c_str(), &devip_);
        inet_aton(devip_str_difop.c_str(), &devip_difop);
    }
    ROS_INFO_STREAM("Opening UDP socket ");
    sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd_ == -1)
    {
        printf("socket"); // TODO: ROS_ERROR errno
        return;
    }
    int opt = 1;
    if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt)))
    {
        printf("setsockopt error!\n");
        return;
    }

    sockaddr_in my_addr;                         // my address information
    memset(&my_addr, 0, sizeof(my_addr));        // initialize to zeros
    my_addr.sin_family = AF_INET;                // host byte order
    my_addr.sin_port = htons(sid);              // port in network byte order
    my_addr.sin_addr.s_addr = htonl(INADDR_ANY); // automatically fill in my IP

    if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
    {
        printf("bind"); // TODO: ROS_ERROR errno
        return;
    }
    if (add_multicast)
    {
        struct ip_mreq group;
        // char *group_ip_ = (char *) group_ip.c_str();
        group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
        // group.imr_interface.s_addr =  INADDR_ANY;
        group.imr_interface.s_addr = htonl(INADDR_ANY);
        // group.imr_interface.s_addr = inet_addr("192.168.1.102");

        if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group)) < 0)
        {
            printf("Adding multicast group error ");
            close(sockfd_);
            exit(1);
        }
        else
            printf("Adding multicast group...OK.\n");
    }

    if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
        printf("non-block");
        return;
    }
  }
  catch (std::exception &e)
  {
    ROS_ERROR("wheeltec_radar can not open!");
  }

  // if (radar_Serial.isOpen())
  // {
  //   ROS_INFO("wheeltec_radar serial port opened");
  // }
}

// 析构函数
turn_on_radar::~turn_on_radar()
{
  (void)close(sockfd_); 
  ROS_INFO("Shutting down");
}

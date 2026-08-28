/*******************************************************
@company: Copyright (C) 2021, Leishen Intelligent System
@product: LSN10
@filename: lsn10.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           21-8-21     leo          new
*******************************************************/
#include "lsn10/lsn10.h"
#include <stdio.h>
#include <signal.h> 

namespace ls{
LSN10 * LSN10::instance()
{
  static LSN10 obj;
  return &obj;
}

LSN10::LSN10()
{
  int code = 0;
  initParam();
  pub_ = n_.advertise<sensor_msgs::LaserScan>(scan_topic_, 3);
  serial_ = LSIOSR::instance(serial_port_, baud_rate_);
  code = serial_->init();
  if(code != 0)
  {
	printf("open_port %s ERROR !\n", serial_port_.c_str());
	ros::shutdown();
	exit(0);
  }
  printf("open_port %s  OK !\n", serial_port_.c_str());
  recv_thread_ = new boost::thread(boost::bind(&LSN10::recvThread, this));
  pubscan_thread_ = new boost::thread(boost::bind(&LSN10::pubScanThread, this));
  difop_switch = n_.subscribe<std_msgs::Int8>("rev_order", 1 ,&LSN10::rev_order,this);
  packet_res = false; 
}

LSN10::~LSN10()
{

  is_shutdown_ = true;

  pubscan_thread_->interrupt();
  pubscan_thread_->join();
  pubscan_thread_ = NULL;
  delete pubscan_thread_;

  recv_thread_->interrupt();
  recv_thread_->join();
  recv_thread_ = NULL;
  delete recv_thread_;

  serial_->close();
  serial_ = NULL;
  delete serial_;
}

void LSN10::rev_order(const std_msgs::Int8 msg){
  int i = msg.data;
  if(i<6 || i>12)	return;
  char data[188] = {0x00};
  data[0] = 0xA5;
  data[1] = 0x5A;
  data[2] = 0x01;
  data[172] = char(i);
  data[184] = 0x0a;
  data[185] = 0x01;
  data[186] = 0xFA;
  data[187] = 0xFB;
 
  int rtn = serial_->send((const char*)data, 188);
  if (rtn < 0)
	printf("start scan error !\n");
  return ;
}

void LSN10::initParam()
{
  std::string scan_topic = "/scan";
  std::string frame_id = "laser_link";
  std::string port = "/dev/ttyUSB0";
  ros::NodeHandle nh("~");
  nh.param("scan_topic", scan_topic_, scan_topic);
  nh.param("frame_id", frame_id_, frame_id);
  nh.param("serial_port", serial_port_, port);
  nh.param("baud_rate", baud_rate_, 460800);
  nh.param("min_distance", min_distance, 0.0);
  nh.param("max_distance", max_distance, 30.0);
  nh.param("truncated_mode", truncated_mode_, 0);
  nh.param<std::vector<int>>("disable_min", disable_angle_min_range, disable_angle_range_default);
  nh.param<std::vector<int>>("disable_max", disable_angle_max_range, disable_angle_range_default);
  count_sum = 0;

  is_shutdown_ = false;
  data_len_ = 200;
  points_size_ = 800 ;
  scan_points_.resize(points_size_);

}

int LSN10::getScan(std::vector<ScanPoint> &points, ros::Time &scan_time, float &scan_duration)
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  points.assign(scan_points_bak_.begin(), scan_points_bak_.end());
  scan_time = pre_time_;
  scan_duration = (time_ - pre_time_).toSec();
  if(scan_duration == 0)
  {
	  time_ = ros::Time::now();
	  scan_duration = (time_ - pre_time_).toSec();
  }
}

int LSN10::getVersion(std::string &version)
{
  version = "lsn10_v1_0";
  return 0;
}

void LSN10::recvThread()
{
  unsigned char * packet_bytes = new unsigned char [data_len_];
  int idx = 0;
  int link_time = 0;
  double degree;
  double end_degree;
  double degree_interval;
  double last_degree = 0.0;

  boost::posix_time::ptime t1,t2;
  t1 = boost::posix_time::microsec_clock::universal_time();
  
  while(!is_shutdown_&&ros::ok()){
	int count;
	if(!packet_res)
		count = serial_->read(packet_bytes, 58);
	else
	{
		count = serial_->read(packet_bytes+1, 57);
	}
	if(count <= 0) 
		link_time++;
	else
		link_time = 0;
	if(count <= 0) 
		continue;
	if(count <58 || !packet_res)
	{
		//printf("count_old = %d\n",count);
		while(1)
		{
		int count_a = serial_->read(packet_bytes+count, 58-count);
		count += count_a;
		//printf("a = %d\n",count_a);
		if(count >= 58)
			break;
		}
		//printf("count = %d\n",count);
	}
	else if(count <57 || packet_res)
	{
		//printf("count_old = %d\n",count);
		while(1)
		{
		int count_a = serial_->read(packet_bytes+count+1, 57-count);
		count += count_a;
		//printf("a = %d\n",count_a);
		if(count >= 57)
			break;
		}
		//printf("count = %d\n",count);
	}
	int packet_H = packet_bytes[0];
	int packet_E = packet_bytes[57];
	if (packet_E == 0xA5 && packet_H == 0x5A)
	{
		memcpy(packet_bytes+1, packet_bytes, 57);
		packet_bytes[0] = 0xA5;
 		packet_res = true;
	}

	if(link_time > 10000)
	{
		serial_->close();
		int ret = serial_->init();
		if(ret < 0)
		{
			ROS_WARN("serial open fail");
			usleep(300000);
		}
		link_time = 0;
	}
	


	for (int i = 0; i < count; i++)
	{
		
		
		int k = packet_bytes[i];
		k < 0 ? k += 256 : k;
		int y = packet_bytes[i + 1];
		y < 0 ? y += 256 : y;
		if (k == 0xA5 && y == 0x5A)					 //应答距离
		{
			packet_res = false;
			//printf("i =%d\n",i);
			if(i != 0)
			{
				memcpy(packet_bytes, packet_bytes + i, 58 - i);
				serial_->read(packet_bytes + 58 - i, i);
			}
			if(packet_bytes[57] != CalCRC8(packet_bytes, 57))		break;
			int s = packet_bytes[5];			
			s < 0 ? s += 256 : s;
			int z = packet_bytes[6];
			z < 0 ? z += 256 : z;
			
				degree = (s * 256 + z) / 100.f;
			
			int s_e = packet_bytes[55];
			s_e < 0 ? s_e += 256 : s_e;
			int z_e = packet_bytes[56];
			z_e < 0 ? z_e += 256 : z_e;
			
			end_degree = (s_e * 256 + z_e) / 100.f;
			//printf("degree = %f\n",degree);
			//printf("end_degree = %f\n",end_degree);
			if(degree > end_degree)
				degree_interval = end_degree + 360 - degree;
			else
				degree_interval = end_degree - degree;	
			int invalidValue = 0;
		
			for (size_t num = 0; num < 48; num+=3)
			{
				int s = packet_bytes[num + 7];
				s < 0 ? s += 256 : s;
				int z = packet_bytes[num + 8];
				z < 0 ? z += 256 : z;
				int y_i = packet_bytes[num + 9];
				y_i < 0 ? y_i += 256 : y_i;
				if ((s * 256 + z) != 0xFFFF)
				{	
					if(idx<=800)
					{
					scan_points_[idx].range = double(s * 256 + (z)) / 1000.f;
					scan_points_[idx].intensity = int(y_i);
					idx++;
					count_sum++;
					}
				}
				else
				{
					invalidValue++;
				}
			}

			invalidValue = 16 - invalidValue;
			

			for (size_t i = 0; i < invalidValue; i++)
			{
				if(idx<=800)
				{
				if ((degree + (degree_interval / invalidValue * i)) > 360)
					scan_points_[idx-invalidValue+i].degree = degree + (degree_interval / invalidValue * i) - 360; 
				else
					scan_points_[idx-invalidValue+i].degree = degree + (degree_interval / invalidValue * i);
				}
			}

			if (degree < last_degree) 	
			{			
				idx = 0;
				for(int k=0;k<scan_points_.size();k++)
				{	
					
					if((360-scan_points_[k].degree) > 360 || (360-scan_points_[k].degree) < 0)
							scan_points_[k].range = 0;

					
					if(scan_points_[k].range < min_distance || scan_points_[k].range > max_distance)
						scan_points_[k].range = 0;
				}
				boost::unique_lock<boost::mutex> lock(mutex_);
				scan_points_bak_.resize(scan_points_.size());
				scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
				for(int k=0; k<scan_points_.size(); k++)
				{
					scan_points_[k].range = 0;
					scan_points_[k].degree = 0;
				}

				pre_time_ = time_;
				lock.unlock();
				pubscan_cond_.notify_one();				
				time_ = ros::Time::now();
			}

			last_degree = degree;
		}
	}		
  }	
  if (packet_bytes)
  {
    packet_bytes = NULL;
    delete packet_bytes;
  }

}

uint8_t LSN10::CalCRC8(unsigned char * p, int len)
{
  uint8_t crc = 0;
  int sum = 0;

  for (int i = 0; i < len; i++)
  {
    sum += uint8_t(p[i]);
  }
  crc = sum & 0xff;
  return crc;
}


void LSN10::pubScanThread()
{
  bool wait_for_wake = true;
  boost::unique_lock<boost::mutex> lock(pubscan_mutex_);

  while (ros::ok() && !is_shutdown_)
  {
    while (wait_for_wake)
    {
      pubscan_cond_.wait(lock);
      wait_for_wake = false;
    }
    std::vector<ScanPoint> points;
    ros::Time start_time;
    float scan_time;
    this->getScan(points, start_time, scan_time);
    int count = count_sum;//points.size();
	//printf("count = %d\n",count);
    if (count <= 0)
      continue;
count = points_size_;
	int scan_num = count;

    sensor_msgs::LaserScan msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = start_time;
	count_sum = 0;

    msg.angle_min = - M_PI;
    msg.angle_max = M_PI;
	msg.angle_increment = 2 * M_PI / count;
    msg.range_min = min_distance;
    msg.range_max = max_distance;
    msg.ranges.resize(scan_num);
    msg.intensities.resize(scan_num);
    msg.scan_time = scan_time;
    msg.time_increment = scan_time / (double)(count - 1);
	
	for(int k=0; k<scan_num; k++)
	{
		msg.ranges[k] = std::numeric_limits<float>::infinity();
        msg.intensities[k] = 0;
	}
	int start_num = floor(0 * count/360);
	int end_num = floor(360 * count/360);

	for (int i = 0; i < count; i++) {
		int point_idx = (360-points[i].degree) * count/360;
		//printf("points[%d].degree = %lf\n",i,points[i].degree);
		//printf("i=%d point_idx=%d\n",i,point_idx);
		if(point_idx<(end_num-count))
			point_idx += count;
		point_idx =  point_idx - start_num;
		if(point_idx < 0 || point_idx > scan_num) 
			continue;
		if (points[i].range == 0.0) {
			msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
			msg.intensities[point_idx] = 0;
		}
		else {
			double dist = points[i].range;
			msg.ranges[point_idx] = (float) dist;
			msg.intensities[point_idx] = points[i].intensity;
		}
      	if(truncated_mode_==1)
			{
				for (int j = 0; j < disable_angle_max_range.size(); ++j) {
		        if ((point_idx >= (disable_angle_min_range[j] * count / 360) ) &&
		              (point_idx <= (disable_angle_max_range[j] * count / 360 ))) {
		            msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
		            msg.intensities[point_idx] = 0;
		          }
		        }
			}
    }

    pub_.publish(msg);
    wait_for_wake = true;
  }
}
}

void handleSig(int signo)
{
  printf("handleSig\n");
  ros::shutdown();
  exit(0);
}

int main(int argv, char **argc)
{
  signal(SIGINT, handleSig);
  signal(SIGTERM, handleSig);
  ros::init(argv, argc, "lsn10");
 
  ls::LSN10* lsn10 = ls::LSN10::instance();
  usleep(100000);
  ros::spin();
  return 0;
}

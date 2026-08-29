/*******************************************************
@company: Copyright (C) 2021, Leishen Intelligent System
@product: LSN10
@filename: lsn10.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           21-8-21     leo          new
*******************************************************/
#include "lsn10/lsn10.h"
#include "lsn10/tofbf.h"
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <algorithm>

#define	MAX_BUF 2304000

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
  code = serial_->init(1);
  if(code != 0)
  {
	ros::shutdown();
	exit(0);
  }
  recv_thread_ = new boost::thread(boost::bind(&LSN10::recvThread, this));
}

LSN10::~LSN10()
{
  printf("start LSN10::~LSN10()\n");

  is_shutdown_ = true;

  recv_thread_->interrupt();
  recv_thread_->join();

  recv_thread_ = NULL;
  delete recv_thread_;

  serial_->close();
  serial_ = NULL;
  delete serial_;
  printf("end LSN10::~LSN10()\n");
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
  nh.param("baud_rate", baud_rate_, 230400);
  
  nh.param("range_min", range_min, 0.5);
  nh.param("range_max", range_max, 150.0);
  nh.param("angle_min", angle_min,0.0);
  nh.param("angle_max", angle_max, 360.0);
  
  is_shutdown_ = false;
  mTimestamp = 0;
}


void LSN10::recvThread()
{
  int count;
  int link_time = 0;
  char * all_bytes = new char[MAX_BUF];

  if (all_bytes == NULL)
  {
    all_bytes = NULL;
  }
  
  while(!is_shutdown_&&ros::ok()){

	count = serial_->read(&all_bytes[0], MAX_BUF);

	if(count <= 0) 
		link_time++;
	else
		link_time = 0;
	
	if(link_time > 10000)
	{
		serial_->close();
		int ret = serial_->init(2);
		if(ret < 0)
		{
			ROS_WARN("serial open fail");
			usleep(300000);
		}
		link_time = 0;
	}
	
	if(count <= 0) 
		continue;

	for (int i = 0; i < count; i++)
	{
		mDataTmp.push_back(*(all_bytes + i));
	}		
	
	if (mDataTmp.size() < sizeof(LiDARFrameTypeDef))
		continue;
	
	if (mDataTmp.size() > sizeof(LiDARFrameTypeDef) * 100)
	{
		mErrorTimes++;
		mDataTmp.clear();
		continue;
	}
	
	uint16_t start = 0;

	while (start < mDataTmp.size() - 2)
	{
		start = 0;
		while (start < mDataTmp.size() - 2)
		{
			if ((mDataTmp[start] == PKG_HEADER) && (mDataTmp[start + 1] == PKG_VER_LEN))
			{
				break;
			}
			start++;
		}

		if (start != 0)
		{
			mErrorTimes++;
			for (int i = 0; i < start; i++)
			{
				mDataTmp.erase(mDataTmp.begin());
			}
		}

		if (mDataTmp.size() < sizeof(LiDARFrameTypeDef))
			break;
	
		LiDARFrameTypeDef* pkg = (LiDARFrameTypeDef *)mDataTmp.data();
		pkg->speed = mDataTmp[3]*256+mDataTmp[4];
		pkg->start_angle = mDataTmp[5]*256+mDataTmp[6];
		pkg->end_angle = mDataTmp[55]*256+mDataTmp[56];
		
		//printf("speed %d\n", pkg->start_angle);
		
		for(int k=0;k<POINT_PER_PACK;k++)
		{
			pkg->point[k].distance = mDataTmp[7+k*3]*256+mDataTmp[8+k*3];
			pkg->point[k].confidence = mDataTmp[9+k*3];
		}
		
		if(mDataTmp[57] != CalCRC8(mDataTmp, mDataTmp.size()-1)) break;

		double diff = (pkg->end_angle / 100 - pkg->start_angle / 100 + 360) % 360;
		if (diff > (double)pkg->speed*POINT_PER_PACK / 4500 * 3 / 2)
		{
			mErrorTimes++;
		}
		else
		{
			mSpeed = pkg->speed;
			uint32_t diff = ((uint32_t)pkg->end_angle + 36000 - (uint32_t)pkg->start_angle) % 36000;
			float step = diff / (POINT_PER_PACK - 1) / 100.0;
			float start = (double)pkg->start_angle / 100.0;
			float end = (double)(pkg->end_angle % 36000) / 100.0;

			PointData data;
			for (int i = 0; i < POINT_PER_PACK; i++)
			{
				data.distance = pkg->point[i].distance;
				data.angle = start + i * step;
				if (data.angle >= 360.0)
				{
					data.angle -= 360.0;
				}
				data.confidence = pkg->point[i].confidence;
				mOnePkg[i] = data;
				mFrameTmp.push_back(PointData(data.angle, data.distance, data.confidence));
			}
			mOnePkg.back().angle = end;
			float last_angle = 0;
			std::vector<PointData> tmp;
			int count = 0;
			
			for (auto n : mFrameTmp)
			{
				/*wait for enough data, need enough data to show a circle*/
				if (n.angle - last_angle < (-180.f)) /* enough data has been obtained */
				{
					Tofbf tofbfLd06(mSpeed); 
					tmp = tofbfLd06.NearFilter(tmp);
					std::sort(tmp.begin(), tmp.end(), [](PointData a, PointData b) {return a.angle < b.angle; });
					if(tmp.size()>0)
					{
						ToLaserscan(tmp);
						for(auto i=0;i<count;i++)
						{
							mFrameTmp.erase(mFrameTmp.begin());
						}
						break;
					}
				}
				else
				{
					tmp.push_back(n);  /* getting data */
				}
				count++;
				last_angle = n.angle;
			}
		}

		for (uint32_t i = 0; i < sizeof(LiDARFrameTypeDef); i++)
		{
			mDataTmp.erase(mDataTmp.begin());
		}

		if (mDataTmp.size() < sizeof(LiDARFrameTypeDef))
		{
			break;
		}
	}
  }

  if (all_bytes)
  {
    all_bytes = NULL;
    delete all_bytes;
  }
}

uint8_t LSN10::CalCRC8(std::vector<uint8_t> p, uint8_t len)
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

void LSN10::ToLaserscan(std::vector<PointData> src)
{
  float angle_increment;

  /*Angle resolution, the smaller the resolution, the smaller the error after conversion*/
 // angle_increment = ANGLE_TO_RADIAN(mSpeed/4500);
  angle_increment = (2 * M_PI) / 450;

  /*Calculate the number of scanning points*/
  unsigned int beam_size = ceil((2 * M_PI) / angle_increment);
	
  output.header.stamp = ros::Time::now();
  output.header.frame_id = frame_id_;
  output.angle_min = 0.0;
  output.angle_max = 2 * M_PI;
  output.range_min = range_min;
  output.range_max = range_max;
  output.angle_increment = angle_increment;
  output.time_increment = 0.0;
  output.scan_time = 0.0;
  
  /*First fill all the data with Nan*/
  output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
  output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

  for (auto point : src)
  {
	point.angle = point.angle - 90.0;
	if(point.angle < 0.0)
		point.angle = point.angle + 360.0;
	
	float range = point.distance/1000.0 ;
    float angle = ANGLE_TO_RADIAN(point.angle);
	
    int index = (int)((angle - output.angle_min) / output.angle_increment);

	if (index >= angle_min/360.0*beam_size && index < angle_max/360.0*beam_size)
    {
      /*If the current content is Nan, it is assigned directly*/
      if (std::isnan(output.ranges[beam_size - 1 - index]))
      {
        output.ranges[beam_size  - 1 - index] = range;
      }   
      else
      {
        if (range < output.ranges[beam_size - 1 - index])
        {
          output.ranges[beam_size - 1  - index] = range;
        }
      }
      output.intensities[beam_size  - 1 - index] = point.confidence;
    }
  }
  pub_.publish(output);
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

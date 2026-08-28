/*******************************************************
@company: Copyright (C) 2021, Leishen Intelligent System
@product: LSN10
@filename: lsn10.h
@brief:
@version:       date:       author:     comments:
@v1.0           21-8-21     fu          new
*******************************************************/
#ifndef LSN10_H
#define LSN10_H
#include <stdint.h>
#include <vector>
#include <array>
#include <iostream>
#include <boost/thread.hpp>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "lsn10/lsiosr.h"

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59/180000)


enum
{
	PKG_HEADER = 0xA5,
	PKG_VER_LEN = 0x5A,
	POINT_PER_PACK = 16,
};

typedef struct  __attribute__((packed))
{
	uint16_t distance;
	uint8_t confidence;
}LidarPointStructDef;

typedef struct  __attribute__((packed))
{
	uint8_t           header;
	uint8_t           header2;
	uint8_t           data_len;
	uint16_t          speed;
	uint16_t          start_angle;
	LidarPointStructDef point[POINT_PER_PACK];
	uint16_t          end_angle;
	uint8_t           crc;
}LiDARFrameTypeDef;

struct PointData
{
	float angle;
	uint16_t distance;
	uint16_t confidence;
	double x;
    double y;
	PointData(float angle, uint16_t distance, uint16_t confidence , double x = 0, double y = 0)
	{
		this->angle = angle;
		this->distance = distance;
		this->confidence = confidence;
		this->x = x;
        this->y = y;
	}
	PointData(){}
	friend std::ostream& operator<<(std::ostream &os , const PointData &data)
    {
        os << data.angle << " "<< data.distance << " " << (int)data.confidence << " "<<data.x << " "<<data.y;
        return  os;
    }
};

namespace ls {

class LSN10
{
public:
  LSN10();
  ~LSN10();

    static LSN10* instance();
    void run();
	sensor_msgs::LaserScan output;
	
    void initParam();
    void recvThread();
	uint16_t GetTimestamp(void) { return mTimestamp; }   /*time stamp of the packet */
	uint8_t CalCRC8(std::vector<uint8_t> p, uint8_t len);
    LSIOSR * serial_;
    boost::thread *recv_thread_;
	
	double angle_min;
	double angle_max;
	double range_min;
	double range_max;
	uint16_t mTimestamp;
	double mSpeed;
	std::vector<uint8_t> mDataTmp;
	long mErrorTimes;

	std::vector<PointData> mFrameTmp;
	std::array<PointData, POINT_PER_PACK>mOnePkg;
	void ToLaserscan(std::vector<PointData> src);
 
	bool is_shutdown_;    // shutdown recvthread

    std::string serial_port_;
    int baud_rate_;
    std::string scan_topic_;
    std::string frame_id_;

    ros::NodeHandle n_;
    ros::Publisher pub_;
};

}
#endif

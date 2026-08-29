#ifndef __SELF_CHECK_H_
#define __SELF_CHECK_H_

#include "ros/ros.h"
#include <iostream>
#include <std_msgs/UInt32.h>

using namespace std;

//cout相关，用于打印带颜色的信息
#define RESET   string("\033[0m")
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"

extern uint32_t Self_CheckingFlag;
enum 
{
	Drvie_overVOL				= (1 << 0),  //电机驱动过压
    Drvie_underVOL              = (1 << 1),  //电机驱动欠压
    L_Motor_overCUR             = (1 << 2),  //左电机过流
    L_Motor_overLoad            = (1 << 3),  //左电机过载
    L_Motor_CUR_ERROR           = (1 << 4),  //左电机电流异常
    L_Motor_Encoder_ERROR       = (1 << 5),  //左电机编码器数据异常
    L_Motor_SPEED_ERROR         = (1 << 6),  //左电机速度异常
    L_Motor_VOL_ERROR           = (1 << 7),  //左电机内部参考电压出错
	Drvie_EEPROM_ERROR          = (1 << 8),	 //驱动器内部EEPROM读写错误
    L_Motor_HAL_ERROR           = (1 << 9),  //左电机霍尔线未插

	R_Motor_overCUR             = (1 << 10),  //右电机过流
    R_Motor_overLoad            = (1 << 11),  //右电机过载
    R_Motor_CUR_ERROR           = (1 << 12),  //右电机电流异常
    R_Motor_Encoder_ERROR       = (1 << 13),  //右电机编码器数据异常
    R_Motor_SPEED_ERROR         = (1 << 14),  //右电机速度异常
    R_Motor_VOL_ERROR           = (1 << 15),  //右电机内部参考电压出错
    R_Motor_HAL_ERROR           = (1 << 16),  //右电机霍尔线未插
	
    Drvie_Timeout               = (1 << 17), //电机驱动器离线
    AutoRecharge_Timeout        = (1 << 18), //自动回充装备离线
    RGBboard_Timeout            = (1 << 19), //超声波控制板离线
    Lower_Power                 = (1 << 20), //电池电压不足
	Stop_Switch_DOWN            = (1 << 21), //急停开关被按下
	lost_left_redsignal         = (1 << 22), //未接收到充电桩左边的红外(39ms红外)
	lost_right_redsignal        = (1 << 23), //未接收到充电桩右边的红外(52ms红外)
};

#define Get_Checking_FLAG(mask)       (Self_CheckingFlag & (mask))         //读取报错标志位


#endif


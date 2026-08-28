#include "AutoRecharge.h"

u8 which_mode;
u8 show_in_windows;

// Enable navigation to find charging pile
//是否开启导航寻找充电桩
u8 nav_walk=0;
// Whether automatic charging is allowed
//是否允许进行自动回充
u8 Allow_Recharge=0;
// Whether the robot is charging, and whether the recharge equipment receives infrared signals
//机器人是否在充电，回充装备是否接收到红外信号
u8 Charging, RED_STATE;
float Charging_Current=0;
float Recharge_Red_Move_X, Recharge_Red_Move_Y, Recharge_Red_Move_Z; 
float Recharge_UP_Move_X, Recharge_UP_Move_Y, Recharge_UP_Move_Z; 

//对接速度
float Red_Docker_X=-0.1f, Red_Docker_Y=0, Red_Docker_Z=0.2f; 

//红外刷新时间 x*10 ms 例子：写80 就是 80*10=800ms刷新1次
u8 refalsh_time = 50;

u8 L_A,L_B,R_B,R_A;

u8 red_now_state;
u8 touch_state;

void CAN_Send_AutoRecharge(void)
{
	u8 CAN_SENT[8];
	
	//预留位
	CAN_SENT[0]=0;
	
	//Set the speed of the infrared interconnection, unit m/s
	//设置红外对接的速度大小，单位mm/s
	CAN_SENT[1]=((short)(Red_Docker_X*1000))>>8;
	CAN_SENT[2]=((short)(Red_Docker_X*1000));
	CAN_SENT[3]=((short)(Red_Docker_Y*1000))>>8;
	CAN_SENT[4]=((short)(Red_Docker_Y*1000));
	CAN_SENT[5]=((short)(Red_Docker_Z*1000))>>8;
	CAN_SENT[6]=((short)(Red_Docker_Z*1000));
	CAN_SENT[7] = refalsh_time;	
	CAN1_Send_Num(0x105,CAN_SENT);

}


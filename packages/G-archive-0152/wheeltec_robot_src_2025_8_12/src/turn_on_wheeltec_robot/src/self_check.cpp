#include "self_check.h"

uint32_t Self_CheckingFlag;//用于接收自检数据

uint32_t Last_State;//保存上一次的自检数据

uint8_t refresh_show = 0;//定时刷新标志位

float realtime = 0;//真实时间

bool display_run = true;

//自检数据回调函数
void check_dataCallback(const std_msgs::UInt32 &msg)
{
	static uint8_t show_once = 1;
	uint8_t tip = 1;//自检错误数量提示

	Self_CheckingFlag = msg.data;//接收自检数据

	if(show_once)
	{
		show_once = 0;
		if(display_run==false)
		{
			if(Self_CheckingFlag!=0) refresh_show=1;	
		}
		else
		{
			refresh_show=1;	
		}
	}

	//当数据发生变化或定时时间到时，在终端输出自检结果
	if(Self_CheckingFlag != Last_State || refresh_show )
	{
		refresh_show = 0;//复位等待下次更新

		printf(YELLOW);

		if(Self_CheckingFlag==0) printf("\r\n底盘当前无报错与警告");

		if(Get_Checking_FLAG(Drvie_overVOL)) printf("\r\n%d",tip++),printf(".电机驱动过压");

		if(Get_Checking_FLAG(Drvie_underVOL)) printf("\r\n%d",tip++),printf(".电机驱动欠压");

		if(Get_Checking_FLAG(Drvie_EEPROM_ERROR)) printf("\r\n%d",tip++),printf(".电机驱动器内部EEPROM读写错误");

		if(Get_Checking_FLAG(L_Motor_overCUR)) printf("\r\n%d",tip++),printf(".左电机过流");

		if(Get_Checking_FLAG(L_Motor_overLoad)) printf("\r\n%d",tip++),printf(".左电机过载");

		if(Get_Checking_FLAG(L_Motor_CUR_ERROR)) printf("\r\n%d",tip++),printf(".左电机电流异常");

		if(Get_Checking_FLAG(L_Motor_Encoder_ERROR)) printf("\r\n%d",tip++),printf(".左电机编码器数据异常");

		if(Get_Checking_FLAG(L_Motor_SPEED_ERROR)) printf("\r\n%d",tip++),printf(".左电机速度异常");

		if(Get_Checking_FLAG(L_Motor_VOL_ERROR)) printf("\r\n%d",tip++),printf(".左电机内部参考电压出错");

		if(Get_Checking_FLAG(L_Motor_HAL_ERROR)) printf("\r\n%d",tip++),printf(".左电机霍尔线未插"); 

		if(Get_Checking_FLAG(R_Motor_overCUR)) printf("\r\n%d",tip++),printf(".右电机过流");

		if(Get_Checking_FLAG(R_Motor_overLoad)) printf("\r\n%d",tip++),printf(".右电机过载");

		if(Get_Checking_FLAG(R_Motor_CUR_ERROR)) printf("\r\n%d",tip++),printf(".右电机电流异常");

		if(Get_Checking_FLAG(R_Motor_Encoder_ERROR)) printf("\r\n%d",tip++),printf(".右电机编码器数据异常");

		if(Get_Checking_FLAG(R_Motor_SPEED_ERROR)) printf("\r\n%d",tip++),printf(".右电机速度异常");

		if(Get_Checking_FLAG(R_Motor_VOL_ERROR)) printf("\r\n%d",tip++),printf(".右电机内部参考电压出错");

		if(Get_Checking_FLAG(R_Motor_HAL_ERROR)) printf("\r\n%d",tip++),printf(".右电机霍尔线未插"); 

		if(Get_Checking_FLAG(Drvie_Timeout)) printf("\r\n%d",tip++),printf(".电机驱动器离线"); 

		if(Get_Checking_FLAG(AutoRecharge_Timeout)) printf("\r\n%d",tip++),printf(".自动回充装备离线");

		if(Get_Checking_FLAG(RGBboard_Timeout)) printf("\r\n%d",tip++),printf(".超声波控制板离线");

		if(Get_Checking_FLAG(Lower_Power)) printf("\r\n%d",tip++),printf(".电池电压不足");

		if(Get_Checking_FLAG(Stop_Switch_DOWN)) printf("\r\n%d",tip++) ,printf(".急停开关被按下");

		if(Get_Checking_FLAG(lost_left_redsignal)) printf("\r\n%d",tip++),printf(".未检测到充电桩左边的红外信号");

		if(Get_Checking_FLAG(lost_right_redsignal)) printf("\r\n%d",tip++),printf(".未检测到充电桩右边的红外信号");

		printf("\r\n\r\n");

		printf("\033[0m");
	}

	Last_State = Self_CheckingFlag;//保存本次结果，用于下一次作对比


}

int main(int argc, char** argv) 
{

	//计时变量，用于定时在终端刷新
	ros::Time Begin_Time, End_Time;

	//初始化节点
	ros::init(argc, argv, "self_check");

	//创建节点句柄
	ros::NodeHandle n;
	ros::NodeHandle nh("~"); 

	//当通过launch文件运行时，display_run = false ，不会持续显示
	nh.param<bool> ("display_run",  display_run,  "true");

	//订阅者创建
	ros::Subscriber check_sub = n.subscribe("/self_check_data",10,check_dataCallback);

	while(ros::ok())
	{
		//真实时间复位时，开启第一次计时
		if(realtime==0) Begin_Time = ros::Time::now();
		
		//不间断获取时间
		End_Time = ros::Time::now();

		//计算时间间隔
		realtime = (End_Time - Begin_Time).toSec();

		//5s 刷新1次自检输出
		if(realtime>5) 
		{
			realtime = 0;

			if(display_run==false) refresh_show = 0;//通过launch文件启动，不持续显示

			else refresh_show = 1; //单独启动，5秒刷新1次
		}

		//等待
		ros::spinOnce();
	}
	

	return 0;  
} 

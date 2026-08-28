#include "sys.h"

IMU_DATA_t imu_data = { 0 };          //9轴数据值
ATTITUDE_DATA_t imu_attitude = { 0 }; //3轴姿态角

int main(void)
{
	//设置系统中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//初始化SysTick,配置其频率为1000Hz
	SysTick_Init(1000);
	
	LED_Init();   //LED初始化
	OLED_Init();  //OLED初始化
	
	TIM7_Init(83,4999);//开启定时器7中断,频率200HZ
	
	//ICM20948陀螺仪初始化(内部已包含软件iic初始化)
	pIMUInterface_t icm20948 = &UserICM20948;
	icm20948->Init();
	
	while(1)
	{
		//显示3轴的角速度(弧度)以及加速度,数据在定时器中断7中获取
		OLED_ShowString(0,0,"X");
		OLED_ShowString(0,10,"Y");
		OLED_ShowString(0,20,"Z");
		OLED_ShowFloat(15,0,imu_data.gyro.x,2,2);
		OLED_ShowFloat(70,0,imu_data.accel.x,2,2);
		OLED_ShowFloat(15,10,imu_data.gyro.y,2,2);
		OLED_ShowFloat(70,10,imu_data.accel.y,2,2);
		OLED_ShowFloat(15,20,imu_data.gyro.z,2,2);
		OLED_ShowFloat(70,20,imu_data.accel.z,2,2);
		
		//显示欧拉角,单位度,数据在定时器中断7中计算得到
		OLED_ShowString(0,30,"Pitch");
		OLED_ShowString(0,40,"Roll ");
		OLED_ShowString(0,50,"Yaw  ");
		OLED_ShowFloat(45,30,imu_attitude.pitch,3,2);
		OLED_ShowFloat(45,40,imu_attitude.roll,3,2);
		OLED_ShowFloat(45,50,imu_attitude.yaw,3,2);
		
		OLED_Refresh_Gram();
		
		delay_ms(50);
	}
	
}

//定时器7更新中断服务函数,根据配置的运行频率触发
void TIM7_IRQHandler(void)
{
	static uint32_t LedTickCnt = 0;
	
	if(TIM_GetITStatus(TIM7, TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		
		pIMUInterface_t icm20948 = &UserICM20948;         //指定操作设备
		icm20948->Update_9axisVal(&imu_data);             //获取9轴数据值
		icm20948->UpdateAttitude(imu_data,&imu_attitude); //根据9轴数据,计算出姿态角.当任务频率发生改变时,请更新 attitudeUpdate 函数内 halftime 变量
		
		//LED闪烁,1秒闪烁1次.辅助检查频率是否正确
		LedTickCnt++;
		if( LedTickCnt > 200 * 1 ) 
		{
			LedTickCnt=0;
			LED = !LED;
		}
	}
}



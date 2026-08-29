#include "sys.h"

IMU_DATA_t imu_data = { 0 };          //9轴数据值
ATTITUDE_DATA_t imu_attitude = { 0 }; //3轴姿态角

volatile uint8_t ADC_StartFlag = 0;
volatile uint8_t BuzzerTipsTime = 0;
volatile uint8_t showkey = 0;

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
	
	Adc_Init();   //ADC初始化
	KEY_Init();   //用户按键、使能开关初始化
	Buzzer_Init();//蜂鸣器初始化
	

	while(1)
	{
		//蜂鸣器demo
		if( BuzzerTipsTime!=0 )
		{
			for(uint8_t i=0;i<BuzzerTipsTime*2;i++)
			{
				Buzzer=!Buzzer;
				delay_ms(100);
			}
			BuzzerTipsTime=0;
		}
		
		//显示用户按键以及急停开关
		OLED_ShowString(0,0,"KEY:");
		OLED_ShowNumber(40,0,showkey,1,12);
		OLED_ShowString(0,20,"EN: ");
		OLED_ShowNumber(40,20,EN,1,12);
		OLED_Refresh_Gram();

	}
	
}

//定时器7更新中断服务函数,根据配置的运行频率触发
void TIM7_IRQHandler(void)
{
	static uint32_t LedTickCnt = 0;
	
	static uint32_t adc_taskCnt = 0;
	
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
		
		//ADC任务
		adc_taskCnt++;
		if( adc_taskCnt == 10 )
		{
			adc_taskCnt = 0;
			ADC_StartFlag=1;
		}
		
		//按键扫描
		uint8_t userKeyState = KEY_Scan(200,0);
		switch( userKeyState )
		{
			case single_click://单击
				BuzzerTipsTime = 1;
				showkey = single_click;
				break;
			case double_click://双击
				BuzzerTipsTime = 2;
				showkey = double_click;
				break;
			case long_click://长按
				BuzzerTipsTime = 3;
				showkey = long_click;
				break;
		}
	}
}



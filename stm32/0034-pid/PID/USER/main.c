#include "sys.h"

IMU_DATA_t imu_data = { 0 };          //9轴数据值
ATTITUDE_DATA_t imu_attitude = { 0 }; //3轴姿态角

volatile uint8_t ADC_StartFlag = 0;
volatile uint8_t BuzzerTipsTime = 0;

//4路电机控制PWM值
volatile int MotorA=0,MotorB=0,MotorC=0,MotorD=0;

//4路编码器值
volatile int EncoderA=0,EncoderB=0,EncoderC=0,EncoderD=0;

//电机转速
volatile float MotorArpm = 0,MotorBrpm = 0,MotorCrpm = 0,MotorDrpm = 0;

//电机目标转速
volatile float TargetArpm = 0,TargetBrpm = 0,TargetCrpm = 0,TargetDrpm = 0;

//电池电压
float robotVol;

//使用本例程时，请根据电机和编码器的实际参数配置以下的宏定义
#define MOTOR_REDUCTION_RATION  30  //电机减速比
#define ENCODER_ACCURACY        13  //编码器精度,GMR编码器是500,霍尔编码器是13
#define CONTROL_FREQ            100 //PID控制器的执行频率,也就是控制频率,跟中断配置相关,本例程配置的是200Hz

//PI控制器
float Velocity_KP = 5.0f,Velocity_KI = 1.6f;

int main(void)
{
	//设置系统中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//初始化SysTick,配置其频率为1000Hz
	SysTick_Init(1000);
	
	uart1_init(115200);//串口1初始化
	LED_Init();   //LED初始化
	OLED_Init();  //OLED初始化
	
	TIM7_Init(83,4999);//开启定时器7中断,频率200HZ
	
	//ICM20948陀螺仪初始化(内部已包含软件iic初始化)
	pIMUInterface_t icm20948 = &UserICM20948;
	icm20948->Init();
	
	Adc_Init();   //ADC初始化
	KEY_Init();   //用户按键、使能开关初始化
	Buzzer_Init();//蜂鸣器初始化
	
	//4路电机PWM初始化,PWM频率10KHz.一路电机控制需要2路PWM
	//0-16799对应电机0到最大转速 负数时电机转向相反
    TIM1_PWM_Init(16799,0);
    TIM9_PWM_Init(16799,0);
    TIM10_PWM_Init(16799,0);
    TIM11_PWM_Init(16799,0);
	
	//4路编码器初始化
	Encoder_Init_TIM2();
	Encoder_Init_TIM3();
	Encoder_Init_TIM4();
	Encoder_Init_TIM5();
	
	while(1)
	{
		//电池电压采集
		if( ADC_StartFlag==1 )
		{
			ADC_StartFlag=0;
			robotVol = (float)Get_Adc(Battery_Ch)/4095.0f * 3.3f * 11.0f;
		}
		
		//蜂鸣器提示
		if( BuzzerTipsTime!=0 )
		{
			BuzzerTipsTime--;
			Buzzer=1;
		}
		else Buzzer = 0;
		
		//4路电机目标转速显示
		OLED_ShowString(0,0,"A");
		OLED_ShowFloat(15,0,TargetArpm,3,2);
		OLED_ShowString(0,10,"B");
		OLED_ShowFloat(15,10,TargetBrpm,3,2);
		OLED_ShowString(0,20,"C");
		OLED_ShowFloat(15,20,TargetCrpm,3,2);
		OLED_ShowString(0,30,"D");
		OLED_ShowFloat(15,30,TargetDrpm,3,2);
		
		//4路电机实际转速显示
		OLED_ShowFloat(70,0,MotorArpm,3,2);
		OLED_ShowFloat(70,10,MotorBrpm,3,2);
		OLED_ShowFloat(70,20,MotorCrpm,3,2);
		OLED_ShowFloat(70,30,MotorDrpm,3,2);
		
		//显示C30D供电电压
		OLED_ShowFloat(60,50,robotVol,2,2);
		OLED_ShowString(108,50,"V");
		OLED_Refresh_Gram();
		
		//上位机显示A路电机的目标转速以及实际转速
		printf("{B%.3f:%.3f}$",TargetArpm,MotorArpm);
		
		delay_ms(40);
	}
	
}

//定时器7更新中断服务函数,根据配置的运行频率触发
void TIM7_IRQHandler(void)
{
	static uint32_t LedTickCnt = 0;
	
	static uint32_t adc_taskCnt = 0;
	
	static uint8_t EncoderTaskFlag = 0;
	
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
		
		//读取编码器原始值,降频至100Hz读取
		EncoderTaskFlag = !EncoderTaskFlag;
		if( EncoderTaskFlag==1 )
		{
			//读取编码器数值
			EncoderA = Read_Encoder(2);
			EncoderB = -Read_Encoder(3);
			EncoderC = Read_Encoder(4);
			EncoderD = -Read_Encoder(5);
			
			//电机旋转一圈编码器读数 = 电机减速比*编码器精度*4
			//电机转速RPM = 当前读取到的值/电机旋转一圈编码器读数 * 控制频率 * 60
			MotorArpm = (float)EncoderA/(MOTOR_REDUCTION_RATION*ENCODER_ACCURACY*4.0f) * CONTROL_FREQ * 60;
			MotorBrpm = (float)EncoderB/(MOTOR_REDUCTION_RATION*ENCODER_ACCURACY*4.0f) * CONTROL_FREQ * 60;
			MotorCrpm = (float)EncoderC/(MOTOR_REDUCTION_RATION*ENCODER_ACCURACY*4.0f) * CONTROL_FREQ * 60;
			MotorDrpm = (float)EncoderD/(MOTOR_REDUCTION_RATION*ENCODER_ACCURACY*4.0f) * CONTROL_FREQ * 60;
			
			//4路电机PI控制器
			MotorA = Incremental_PI_A(MotorArpm,TargetArpm);
			MotorB = Incremental_PI_B(MotorBrpm,TargetBrpm);
			MotorC = Incremental_PI_C(MotorCrpm,TargetCrpm);
			MotorD = Incremental_PI_D(MotorDrpm,TargetDrpm);
		}

	
		//按键扫描
		uint8_t userKeyState = KEY_Scan(200,0);
		switch( userKeyState )
		{
			case single_click://单击
				TargetArpm+=60; TargetBrpm+=60; TargetCrpm+=60; TargetDrpm+=60;
				if( robotVol < 10.0f ) BuzzerTipsTime = 10; //电池电压不足以驱动电机,蜂鸣器提示
				break;
			case double_click://双击
				TargetArpm-=60; TargetBrpm-=60; TargetCrpm-=60; TargetDrpm-=60;
				if( robotVol < 10.0f ) BuzzerTipsTime = 10; //电池电压不足以驱动电机,蜂鸣器提示
				break;
			case long_click://长按
				TargetArpm=0,TargetBrpm=0,TargetCrpm=0,TargetDrpm=0;
				break;
		}
		
		//PWM限幅
		MotorA = target_limit_int(MotorA,-16799,16799);
		MotorB = target_limit_int(MotorB,-16799,16799);
		MotorC = target_limit_int(MotorC,-16799,16799);
		MotorD = target_limit_int(MotorD,-16799,16799);
		
		//发送PWM到电机函数, 0-16799对应电机0到最大转速
		//0-16799对应电机0到最大转速 负数时电机转向相反
		Set_Pwm(MotorA,MotorB,MotorC,MotorD);
	}
}

int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}

//PI控制器
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}

int Incremental_PI_B (float Encoder,float Target)
{ 	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}

int Incremental_PI_C (float Encoder,float Target)
{ 	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}

int Incremental_PI_D (float Encoder,float Target)
{ 	
	 static float Bias=0,Pwm=0,Last_bias=0;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}

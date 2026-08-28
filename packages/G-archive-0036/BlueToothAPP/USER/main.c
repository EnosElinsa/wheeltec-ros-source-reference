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

//舵机目标角度
float target_angle = 90;

//定义使用的舵机是180度还是270度舵机
#define _270_SERVO  0
#define _180_SERVO  1

//电机控制相关参数配置
#define MOTOR_REDUCTION_RATION  30  //电机减速比
#define ENCODER_ACCURACY        13  //编码器精度,GMR编码器是500,霍尔编码器是13
#define CONTROL_FREQ            100 //PID控制器的执行频率,也就是控制频率,跟中断配置相关,本例程配置的是200Hz

//电机PI控制器参数
float Velocity_KP = 5.0f,Velocity_KI = 1.6f;

char app_page1[256]={ 0 };//发送到APP首页的数据缓冲区
char app_page2[256]={ 0 };//发送到APP波形页面的数据缓冲区
char app_page3[256]={ 0 };//发送到APP参数页面的数据缓冲区

//测试数值,可以通过APP显示并修改
uint16_t TestAPP_Param1 = 100;
uint16_t TestAPP_Param2 = 400;

int main(void)
{
	//设置系统中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//初始化SysTick,配置其频率为1000Hz
	SysTick_Init(1000);
	
	uart1_init(115200);//串口1初始化
	uart2_init(9600); //串口2初始化,用于蓝牙APP
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
	
	//舵机PWM初始化,初始化为50Hz,0-19099对应 0 到 100% 占空比
	//2.5%~12.5%(对应数值500~2500)占空比对应舵机的0-180或0-270度
	TIM12_SERVO_Init(19999,83);
	
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
		
		//显示AB电机以及舵机的目标值、实际值
		OLED_ShowString(0,0,"A");
		OLED_ShowString(0,10,"B");
		OLED_ShowString(0,20,"Servo");
		OLED_ShowFloat(15,0,TargetArpm,3,2);
		OLED_ShowFloat(15,10,TargetBrpm,3,2);
		OLED_ShowFloat(70,0,MotorArpm,3,2);
		OLED_ShowFloat(70,10,MotorBrpm,3,2);
		OLED_ShowFloat(70,20,target_angle,3,2);
		
		OLED_ShowNumber(0,40,TestAPP_Param1,5,12);
		OLED_ShowNumber(0,50,TestAPP_Param2,5,12);
		
		//显示C30D供电电压
		OLED_ShowFloat(60,50,robotVol,2,2);
		OLED_ShowString(108,50,"V");
		OLED_Refresh_Gram();
		
		//APP请求获取数据,这里可以把需要调试的参数发送过去,然后在APP解码函数里接收APP的数据修改.可用于调试PID参数等操作
		if( wheeltecApp.reportparam == 1 )
		{
			wheeltecApp.reportparam=0;
			sprintf((char*)app_page3,"{C%d:%d}$",TestAPP_Param1,TestAPP_Param2);
			AppSendData(app_page3,strlen(app_page3));//将数据发送到APP
			//发送数据到APP参数页面
		}
		else
		{
			//首页和波形页面的数据分时显示,降低频率
			static uint8_t showtime = 0;
			showtime++;
			if( showtime==1 )
			{
				//APP首页数据显示.数据顺序分别是左码盘、右码盘、电池电量百分比、角度
				int volper = voltage_to_percentage(robotVol);//计算电池电量百分比
				sprintf((char*)app_page1,"{A%d:%d:%d:%d}$",(int)MotorArpm,(int)MotorBrpm,volper,(int)imu_attitude.yaw);
				AppSendData(app_page1,strlen(app_page1));//将数据发送到APP
			}
			else if( showtime==2 )
			{
				//APP波形页面数据显示,显示欧拉角
				sprintf((char*)app_page2,"{B%d:%d:%d}$",(int)imu_attitude.pitch,(int)imu_attitude.roll,(int)imu_attitude.yaw);
				AppSendData(app_page2,strlen(app_page2));//将数据发送到APP
			}
			else showtime=0;
		}
		
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
		
		//获取蓝牙的方向键值，控制电机以及舵机
		//1~8 对应 摇杆往前，顺时针旋转
		switch(wheeltecApp.dirkey)
		{
			case 1: TargetArpm = 120,TargetBrpm = 120; break; //前
			
			case 2: TargetArpm = 120,TargetBrpm = 60, target_angle = 90 + 30; break; //右前
			
			case 3: target_angle = 90 + 30; break; //右
			
			case 4: TargetArpm = -120,TargetBrpm = -60,target_angle = 90 + 30; break; //右后
			
			case 5: TargetArpm = -120,TargetBrpm = -120; break; //后
			
			case 6: TargetArpm = -60,TargetBrpm = -120,target_angle = 90 - 30;break; //左后
			
			case 7: target_angle = 90 - 30; break; //左
			
			case 8: TargetArpm = 60,TargetBrpm = 120, target_angle = 90 - 30;break; //左前
			
			default: TargetArpm=0,TargetBrpm=0,target_angle=90; break;//其他值,停止
		}

		//APP请求掉电保存参数,对于单片机来说可以写flash或者外部EEPROM,这里用蜂鸣器模拟操作
		if( wheeltecApp.saveflash == 1 )
		{
			wheeltecApp.saveflash=0;
			BuzzerTipsTime=10;
		}
		
		//读取编码器原始值,并执行电机PI控制
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
		
		//舵机控制,限幅目标值
		#if _180_SERVO
		target_angle = target_limit_float(target_angle,0,180);
		#endif
		
		#if _270_SERVO
		target_angle = target_limit_float(target_angle,0,270);
		#endif
		
		//电机PWM限幅
		MotorA = target_limit_int(MotorA,-16799,16799);
		MotorB = target_limit_int(MotorB,-16799,16799);
		MotorC = target_limit_int(MotorC,-16799,16799);
		MotorD = target_limit_int(MotorD,-16799,16799);
		
		//电机控制：发送PWM到电机
		Set_Pwm(MotorA,MotorB,MotorC,MotorD);
		
		//舵机控制：将目标角度转换为PWM并赋值
		Servo_PWM = Angle_to_PWM(target_angle);
		
	}
}

//串口2接收中断,接收手机APP通过蓝牙模块发送过来的数据
void USART2_IRQHandler(void)
{	
	uint8_t Usart_Receive;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//判断是否接收到数据
	{	
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		Usart_Receive = USART_ReceiveData(USART2);
		
		//将接收到的串口数据传入解码函数,对手机APP的数据进行解析
		//解析后的数据存放在 结构体 wheeltecApp 处
		BlueToothAPPDecode(Usart_Receive);
	}
}



//串口2发送函数,用于发送数据到APP
void AppSendData(char* buffer,uint8_t Len)
{
	for(uint8_t i=0;i<Len;i++)
	{
		while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
		USART2->DR = (u8) buffer[i];  
	}
}


//舵机角度转化为PWM函数
int Angle_to_PWM(float Angle)
{
	//PWM 500~2500 对应占空比 2.5%~12.5%，对应舵机 0-180度
	
	#if _180_SERVO
	// 180度舵机角度转换
    if (Angle < 0.0f) Angle = 0.0f;
    if (Angle > 180.0f) Angle = 180.0f;
    
    // 线性映射：PWM = 500 + (2500-500)*Angle/180
    // 简化后：PWM = 500 + 2000*Angle/180
    int PWM = (int)(500 + (2000.0f * Angle / 180.0f));
    
    // 确保PWM值在有效范围内
    if (PWM < 500) PWM = 500;
    if (PWM > 2500) PWM = 2500;
    
    return PWM;
	#endif
	
	//如果是270度舵机，则使用下面内容
	#if _270_SERVO
	// 限制角度范围在0~270度
    if (Angle < 0.0f) Angle = 0.0f;
    if (Angle > 270.0f) Angle = 270.0f;
    
    // 线性映射：PWM = 500 + (2500-500)*Angle/270
    // 简化后：PWM = 500 + 2000*Angle/270
    int PWM = (int)(500 + (2000.0f * Angle / 270.0f));
    
    // 确保PWM值在有效范围内
    if (PWM < 500) PWM = 500;
    if (PWM > 2500) PWM = 2500;
    
    return PWM;
	#endif
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

float target_limit_float(float insert,float low,float high)
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

int voltage_to_percentage(float voltage) {
    const float V_MIN = 9.5f;  // 最小电压 (0%)
    const float V_MAX = 12.4f; // 最大电压 (100%)
    
    // 确保电压在有效范围内
    if (voltage < V_MIN) return 0;
    if (voltage > V_MAX) return 100;
    
    // 线性插值计算百分比并取整
    int percentage = (int)(((voltage - V_MIN) / (V_MAX - V_MIN)) * 100.0f + 0.5f); // 四舍五入
	
	if( percentage > 100 ) percentage = 100;
	if( percentage <= 0 ) percentage = 0;
	
    return percentage;
}

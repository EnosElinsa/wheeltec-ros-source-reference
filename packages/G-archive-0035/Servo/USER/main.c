#include "sys.h"

IMU_DATA_t imu_data = { 0 };          //9轴数据值
ATTITUDE_DATA_t imu_attitude = { 0 }; //3轴姿态角

volatile uint8_t ADC_StartFlag = 0;
volatile uint8_t BuzzerTipsTime = 0;

//电池电压
float robotVol;

//舵机目标角度
float target_angle = 90;

//定义使用的舵机是180度还是270度舵机
#define _270_SERVO  0
#define _180_SERVO  1

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
		
		//显示舵机的控制角度
		OLED_ShowString(0,0,"Angle");
		OLED_ShowFloat(50,0,target_angle,3,2);
		OLED_ShowString(0,20,"PWM  ");
		OLED_ShowNumber(50,20,Servo_PWM,4,12);
		
		//显示C30D供电电压
		OLED_ShowFloat(60,50,robotVol,2,2);
		OLED_ShowString(108,50,"V");
		OLED_Refresh_Gram();
		
		delay_ms(40);
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
				target_angle += 15;
				break;
			case double_click://双击
				target_angle -= 15;
				break;
			case long_click://长按
				target_angle = 90;
				break;
		}
		
		#if _180_SERVO
		//目标值限幅
		target_angle = target_limit_float(target_angle,0,180);
		#endif
		
		#if _270_SERVO
		//270度舵机
		target_angle = target_limit_float(target_angle,0,270);
		#endif
		
		//将目标角度转换为PWM并赋值
		Servo_PWM = Angle_to_PWM(target_angle);
		
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


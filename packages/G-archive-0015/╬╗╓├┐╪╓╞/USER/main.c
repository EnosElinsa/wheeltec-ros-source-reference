#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/

**************************************************************************/

int Encoder,Position=10000,Target_Position=10000; //编码器的脉冲计数
int Moto;//电机PWM变量 应是Motor的 向Moto致敬
float Position_KP=120,Position_KI=0.1,Position_KD=500; //PID系数

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断分组为2
	delay_init(72);                 //=====延时初始化
	uart_init(128000);              //=====初始化串口1
	MiniBalance_Motor_Init();       //=====初始化控制电机所需的IO
	MiniBalance_PWM_Init(7199,0);   //=====初始化PWM 10KHZ，用于驱动电机 
	KEY_Init();						//=====按键初始化
	Encoder_Init_TIM4();            //=====初始化编码器（TIM4的编码器接口模式） 
	Timer1_Init(99,7199);           //=====定时中断初始化 
	delay_ms(100);
	while(1)
		{
//			printf("Position: %d  ",Position);		//当前编码器位置打印到串口
//			printf("Moto: %d  ",Moto);				//当前的PID控制器输出打印到串口
//			printf("Target: %d\r\n",Target_Position);//当前的位置目标值打印到串口
//			delay_ms(200);
			
			DataScope();//Minibalance上位机显示波形
			delay_ms(50);//上位机需要的延时，严格50ms
		} 
}


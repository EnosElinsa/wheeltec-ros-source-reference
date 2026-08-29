#include "moto.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
void MiniBalance_Motor_Init(void)          //初始化控制电机所需的IO
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);  //使能GPIOB的时钟

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;         //推挽输出
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;   //PB12 PB13
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
}
void MiniBalance_PWM_Init(u16 arr,u16 psc) //初始化pwm输出引脚
{		 	

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;                             

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);   //使能定时器3时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);  //使能GPIOB的时钟

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;          //复用输出
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;                //PB1
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);

	TIM_TimeBaseInitStruct.TIM_Period = arr;              //设定计数器自动重装值 
	TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          //设定预分频器
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;         //设置时钟分割
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);       //初始化定时器

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     //设置输出极性
	TIM_OC4Init(TIM3,&TIM_OCInitStruct);                       //初始化输出比较参数

	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);   //CH4使能预装载寄存器

	TIM_ARRPreloadConfig(TIM3, ENABLE);                //使能TIM3在ARR上的预装载寄存器

	TIM_Cmd(TIM3,ENABLE);                              //使能定时器3
} 


#include "delay.h"
#include "led.h"
#include "key.h" 
#include "EXTI.h" 

//外部中断初始化函数
void EXTI5_Init(void)
{
 
 	  EXTI_InitTypeDef EXTI_InitStructure; //定义一个外部中断初始化的结构体
 	  NVIC_InitTypeDef NVIC_InitStructure; //定义一个中断优先级初始化的结构体

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//外部中断需要使能AFIO时钟

	  KEY_Init();//初始化按键IO

  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource5); //该函数将GPIO端口与中断线映射起来

  	EXTI_InitStructure.EXTI_Line=EXTI_Line5; //外部中断线5
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	//外部中断线0-15为产生中断
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE; //使能
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			  //属于外部中断5-9
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;				//响应优先级1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//中断使能
  	NVIC_Init(&NVIC_InitStructure); //根据NVIC_InitStrue的参数初始化VIC寄存器
  
}

void EXTI9_5_IRQHandler(void)
{
  delay_ms(10);    //防抖
	if(KEY==0)       //防抖
	{	  
		PAout(4)=!PAout(4); //LED灯取反
	}
	EXTI_ClearITPendingBit(EXTI_Line5);  //清除EXTI5线路中断标志
}

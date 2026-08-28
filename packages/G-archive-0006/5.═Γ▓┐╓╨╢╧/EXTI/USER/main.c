#include "led.h"
#include "key.h" 
#include "delay.h"
#include "EXTI.h" 


int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断优先级分组
	KEY_Init(); //初始化按键IO
	LED_Init(); //初始化LED灯IO
	delay_init(); //初始化延时函数
	EXTI5_Init(); //初始化外部中断
  while(1)
	{
    //本次例程无主函数，用户可自定义
	}
}


#include "led.h" 
#include "TIMER.h"

int main(void)
{
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断优先级分组，即优先级分级个数
	                                                //NVIC_PriorityGroup_2，代表抢占优先级位数位2，可以分[0, 1, 2, 3]四级优先级
	                                                //                          响应优先级位数位2，可以分[0, 1, 2, 3]四级优先级
	LED_Init(); //调用初始化LED函数
  TIM2_Int_Init(9999, 7199); //TIM2_Int_Init(u16 arr, u16 psc)，初始化定时器TIM2
	                           //定时时间=（arr+1)(psc+1)/Tclk，Tclk为内部通用定时器时钟，本例程默认设置为72MHZ
	
  while(1)
	{
		//本次例程无主函数，用户可自定义
	}
}

#include "TIMER.h"
#include "usart.h"   				

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
  TIM3_Cap_Init(0xffff,7200);	//定时器计数上限为0xffff，计数频率为72M/7200
	uart_init(9600);
	
  while(1)
	{		 
    //本次例程无主函数，用户可自定义
	}
}

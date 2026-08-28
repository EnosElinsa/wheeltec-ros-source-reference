#include "sys.h"


int main(void)
{
	//设置系统中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//初始化SysTick,配置其频率为1000Hz
	SysTick_Init(1000);
	
	LED_Init();   //LED初始化
	OLED_Init();  //OLED初始化
	
	float showtest1 = 0;
	uint8_t showtest2 = 0;
	
	while(1)
	{
		OLED_ShowString(0,0,"Hello C30D");        //字符串显示方法
		OLED_ShowNumber(0,20,showtest2++,3,12);   //非负整数显示方法
		OLED_ShowFloat(0,40,showtest1+=0.01f,3,2);//浮点数显示方法
		OLED_Refresh_Gram();
		
		LED = !LED;
		delay_ms(50);
	}
	
}


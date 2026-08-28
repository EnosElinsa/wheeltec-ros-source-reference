#include "stm32f10x_gpio.h" 
#include "led.h" 
#include "delay.h" 
#include "sys.h"

int main(void)
{
	LED_Init(); //调用初始化LED函数，引用"led.h"后可使用
	delay_init(); //调用初始化延迟函数，引用"delay.h"后可使用
  while(1)
	{
	  GPIO_SetBits(GPIOA, GPIO_Pin_4);    //GPIOA_PIN4输出高电平，LED灯熄灭，引用"stm32f10x_gpio.h"后可使用
		//PAout(4)=1; //引用sys.h头文件后，可以直接使用该宏定义操作IO(GPIOA_PIN4)输出高电平
		delay_ms(1000);                     //延迟1秒，引用"delay.h"后可使用 
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //设置A4引脚为低电平，点亮，引用"stm32f10x.h"后可使用
		//PAout(4)=0; //引用sys.h头文件后，可以直接使用该宏定义操作IO(GPIOA_PIN4)输出低电平
		delay_ms(1000);                     //延迟1秒，引用"delay.h"后可使用
	}
}


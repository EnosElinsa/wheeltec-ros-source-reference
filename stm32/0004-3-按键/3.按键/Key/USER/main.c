#include "led.h" 
#include "key.h" 
#include "delay.h"
//由于"key.h"中已引用"stm32f10x_gpio.h" ，则此处无须重复引用
//而"delay.h"的引用在"key.c"中，而不是在"key.h"中，则此处需要声明引用

int main(void)
{
	KEY_Init(); //初始化按键IO
	LED_Init(); //初始化LED灯IO
	delay_init(); //初始化延时函数
	int a=0;
  while(1)
	{
		if(KEY_Scan()) //判断按键是否按下
		{
			a=a+1;
			if(a>=2)a=0; //如果a=1,按键按下后a=0
		}
		if(a) GPIO_ResetBits(GPIOA, GPIO_Pin_4);  //设置A4引脚为低电平，点亮
		      //PAout(4)=0; //引用sys.h头文件后，可以直接使用该函数设置A4引脚为低电平
		else  GPIO_SetBits(GPIOA, GPIO_Pin_4);    //设置A4引脚为高电平，熄灭
		      //PAout(4)=1; //引用sys.h头文件后，可以直接使用该函数设置A4引脚为高电平
	}
}


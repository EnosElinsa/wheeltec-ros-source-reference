#include "key.h"
#include "delay.h"
//由于"key.h"中已引用"stm32f10x.h" ，则此处无须重复引用

void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO5_InitStructure; //定义一个引脚初始化的类
	GPIO5_InitStructure.GPIO_Pin=GPIO_Pin_5; //定义为引脚5
	GPIO5_InitStructure.GPIO_Mode=GPIO_Mode_IPU; //定义该引脚输入输出模式为上拉输入模式，默认为高电平，外接地后为低电平
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能GPIOA时钟，在STM32中使用IO口前都要使能对应时钟
	GPIO_Init(GPIOA, &GPIO5_InitStructure); //初始化引脚GPIOA5。GPIOA代表GPIOA，&GPIO5_InitStructure代表要初始化的引脚号和引脚状态
}

u8 KEY_Scan()
{
	static u8 flag_key=1;//按键按松开标志，static使flag_key在函数执行完后依然存在，值依然不变
	if(flag_key&&KEY==0)
	{
	  flag_key=0;
	  return 1;	// 按键按下
	}
  else if(KEY==1) flag_key=1;
	return 0;//无按键按下
}

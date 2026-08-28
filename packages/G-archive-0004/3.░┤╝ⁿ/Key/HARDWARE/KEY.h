#ifndef __KEY_H
#define __KEY_H	
#include "sys.h"
#include "stm32f10x_gpio.h" 

#define KEY GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)//宏定义KEY为读取按键状态，宏定义可以提高程序可读性
//#define KEY PAin(5) //引用sys.h头文件后，可以直接使用PAin(5)读取按键状态

void KEY_Init(void);    //KEY键初始化
u8 KEY_Scan(void);  	//按键扫描函数	

#endif

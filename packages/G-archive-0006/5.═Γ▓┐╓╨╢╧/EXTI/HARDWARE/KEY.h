#ifndef __KEY_H
#define __KEY_H	
#include "sys.h"

#include "stm32f10x.h" 

#define KEY GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)//读取按键状态  #define A B  代表定义全局变量A=B 
//#define KEY PAin(5) //引用sys.h头文件后，可以直接使用PAin(5)读取按键状态

void KEY_Init(void);    //IO初始化
u8 KEY_Scan(void);  	//按键扫描函数	

#endif

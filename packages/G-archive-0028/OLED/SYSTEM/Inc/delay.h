#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stm32f4xx.h"

//SysTick时钟
#define SysTickFreq (SystemCoreClock/8)

//对外函数
void SysTick_Init(uint32_t Fre);
uint32_t User_GetSysTickTime(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif


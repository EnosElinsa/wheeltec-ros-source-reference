#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "sys.h"

// No larger than 65535, because the timer of STM32F103 is 16 bit
//不可大于65535，因为STM32F103的定时器是16位的
#define ENCODER_TIM_PERIOD 0xFFFF  

void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
void Encoder_Init_TIM4(void);
void Encoder_Init_TIM5(void);

int Read_Encoder(u8 TIMX);

#endif /* __BSP_ENCODER_H */

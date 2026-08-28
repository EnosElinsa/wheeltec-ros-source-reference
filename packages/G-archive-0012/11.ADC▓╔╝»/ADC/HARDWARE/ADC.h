#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#include "stm32f10x_adc.h"

void adc_Init(void);
u16 Get_adc(u8 chn);
u16 Get_adc_Average(u8 chn, u8 times);

#endif

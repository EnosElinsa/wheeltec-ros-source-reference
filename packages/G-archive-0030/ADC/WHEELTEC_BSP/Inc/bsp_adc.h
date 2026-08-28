#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "sys.h"

void Adc_Init(void);
u16 Get_Adc(u8 ch);

#define Battery_Ch    8 //Battery voltage, ADC channel 8 //电池电压，ADC通道8
#define CarMode_Ch    9 //Potentiometer, ADC channel 9 //电位器，ADC通道9

#endif /* __BSP_ADC_H */

#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#include "system.h"

//Slide rail potentiometer, ADC channel 0
//滑轨电位器，ADC通道0
#define SLIDE_POSITION_ADC 0 //ADC0

//Top right potentiometer, ADC channel 1
//右上角电位器，ADC通道1
#define SERVO_BALANCE_ADC 1 

//Battery voltage, ADC channel 5 
//电池电压，ADC通道5
#define Battery_Ch 5 

//Top left potentiometer, ADC channel 13 
//左上角电位器，ADC通道13
#define CAR_MODE_ADC 13 

void Adc_Init(void);
u16 Get_Adc(u8 ch);
float Get_battery_volt(void) ;
u16 Get_adc_Average(u8 chn, u8 times);
extern float Voltage,Voltage_Count,Voltage_All; 	

#endif 



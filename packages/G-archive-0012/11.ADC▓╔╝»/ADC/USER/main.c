#include "adc.h"
#include "usart.h"   				
#include "delay.h"

int main(void)
{
	float AdcValue=0;
	delay_init();
	adc_Init();
	uart_init(9600);
  while(1)
	{		
		AdcValue=3.3*Get_adc_Average(ADC_Channel_0,10)/0x0fff; //ADC值范围为从0-2^12=4095（111111111111）
		                                                       //一般情况下如本例程对应电压为0-3.3V
		delay_ms(300);
    printf("adc_Average=%.3fV\r\n", AdcValue); //保留3为小数发送
	}
}

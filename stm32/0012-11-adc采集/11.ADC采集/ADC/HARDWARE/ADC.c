#include "adc.h"
#include "delay.h"

//ADC初始化函数
void adc_Init()
{
  GPIO_InitTypeDef GPIO_InitStructure; //定义一个引脚初始化的结构体  
  ADC_InitTypeDef ADC_InitStructure; //定义一个ADC初始化的结构体 
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能CPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能TIM4时钟
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0; //引脚0
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //引脚输入输出模式为模拟输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure); //根据上面设置好的GPIO_InitStructure参数，初始化引脚GPIOA_PIN0

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M
	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//是否为扫描（一组）模式：否：单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//是否为连续转换模式，否：单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据对齐模式：右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器  
	
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);	//使能复位校准   
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
}

//采集ADC值函数，输入参数为ADC通道
u16 Get_adc(u8 chn)
{
	ADC_RegularChannelConfig(ADC1, chn, 1, ADC_SampleTime_239Cycles5 );	//ADC1,chn：ADC通道，第3个参数设置该通道的转换顺序（多通道模式下）
	                                                                  //采样时间为239.5周期=239.5/ADCCLOK，ADCCLOK=72/6MHZ(6代表ADC初始化时的分频系数)		    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束
	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}

//采集多次ADC值求平均值函数，输入参数为ADC通道和采集次数
u16 Get_adc_Average(u8 chn, u8 times)
{
  u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_adc(chn);
		delay_ms(5);
	}
	return temp_val/times;
}



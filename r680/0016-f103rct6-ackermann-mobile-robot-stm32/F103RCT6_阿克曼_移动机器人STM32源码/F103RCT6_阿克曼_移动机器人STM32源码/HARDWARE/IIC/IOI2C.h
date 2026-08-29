#ifndef __IOI2C_H
#define __IOI2C_H
#include "stm32f10x.h"
#include "system.h"
 /**
 @ SCL  PB10
 @ SDA  PB11
**/ 	   		   
#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF; GPIOB->CRH|=8<<12;} //IO direction setting, input //IO方向设置，输入
#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF; GPIOB->CRH|=3<<12;} //IO direction setting, output //IO方向设置，输出
#define IIC_SCL   PBout(10) //SCL
#define IIC_SDA   PBout(11) //SDA
#define READ_SDA  PBin(11)  //Input SDA 

void IIC_Init(void);                			 
void IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(u8 txd);			
unsigned char IIC_Read_Byte(unsigned char ack);
unsigned char IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);				
#endif


#include "sys.h"
/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V5.0
修改时间：2021-11-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V5.0
Update：2021-11-05

All rights reserved
***********************************************/
u8 Way_Angle=1;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_Direction=0; //蓝牙遥控相关的变量
u8 Flag_Stop=1,Flag_Show=0;                 //停止标志位和 显示标志位 默认停止 显示打开
int Encoder_A,Encoder_B,Encoder_C;             //编码器的脉冲计数
int Encoder_A_EXTI;
int Motor_A,Motor_B,Motor_C;                   //电机PWM变量
int Target_A,Target_B,Target_C;                   //电机目标速度
int Temperature;                            //显示温度
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Show_Data_Mb;                         //全局显示变量，用于显示需要查看的数据
u32 Distance;                               //超声波测距
u8 delay_50,delay_flag;         //默认情况下，不开启避障功能，长按用户按键2s以上可以进入避障模式
int Velocity=30;
u8 Usart3_Receive;
u8 rxbuf[8],Rxbuf[24];
u8 txbuf[8];
int AZ,GZ;
float Pitch,Roll,Yaw; 
u32 count,usart_count;
u8 CAN_ON_Flag;
int temp,temp2,temp3,count_temp,flag;
u8 ON_txbuf[8]={10,12,15,19,24,30,37,0} ;//校验组数据
u8 Velocity_txbuf[8]={11,13,16,20,25,31,38,50} ; 
u8 k;
int main(void)
{ 
	Stm32_Clock_Init(9);            //=====系统时钟设置
	delay_init(72);                 //=====延时初始化
  uart_init(72,9600);           //=====初始化串口1
  uart2_init(36,9600);            //=====串口2初始化
	Timer1_Init(499,7199);
	CAN1_Mode_Init(1,2,3,6,0);  //CAN初始化
	LED_Init();
  LED=1;
	while(1)
	{    	
		int i;
		
		//发送的数据是十进制的
		txbuf[0]=1;  //X轴速度
		txbuf[1]=0;  
		txbuf[2]=0;	 //Y轴速度
		txbuf[3]=0;		
		txbuf[4]=0;  //Z轴速度
		txbuf[5]=0;  
		txbuf[6]=0;  //预留位
		txbuf[7]=0;	 //预留位
	  CAN1_SEND(0X181,txbuf);//CAN发送
		    
/***********CAN发送数据*********************************/
     
/***********串口打印接收数据*********************************/


//				printf("0xff\r\n");         //用于分割开方便查看
//			  printf("%d\t",Rxbuf[0]); //帧头(固定值)
//				printf("%d\t",Rxbuf[1]); //电机状态		  
//				printf("%d\t",Rxbuf[2]); //小车x轴实际速度	  
//				printf("%d\t",Rxbuf[3]); 
//				printf("%d\t",Rxbuf[4]); //小车y轴实际速度
//				printf("%d\t",Rxbuf[5]); 
//				printf("%d\t",Rxbuf[6]); //小车z轴实际速度
//				printf("%d\r\n",Rxbuf[7]); 
//			  printf("%d\t",Rxbuf[8]); //加速度计三轴加速度
//				printf("%d\t",Rxbuf[9]);			  
//				printf("%d\t",Rxbuf[10]);		  
//				printf("%d\t",Rxbuf[11]);
//				printf("%d\t",Rxbuf[12]);
//				printf("%d\t",Rxbuf[13]); 
//				printf("%d\t",Rxbuf[14]); //角速度计三轴角速度
//				printf("%d\r\n",Rxbuf[15]); 
//				printf("%d\t",Rxbuf[16]);
//				printf("%d\t",Rxbuf[17]);			  
//				printf("%d\t",Rxbuf[18]);		  
//				printf("%d\t",Rxbuf[19]);
//				printf("%d\t",Rxbuf[20]); //电池电压
//				printf("%d\t",Rxbuf[21]);
//				printf("%d\t",Rxbuf[22]);	//数据校验位计算，模式1是发送数据校验	  
//				printf("%d\t",Rxbuf[23]);	//帧尾（固定值）  
//			  printf("\r\noxfe\r\n");         //用于分割开方便查看
				
				//发送接收到的24位数据
				for(i=0; i<24; i++)
				{
					USART1->DR = Rxbuf[i];
				while((USART1->SR&0x40)==0);
				}	
 
		} 
}

#include "sys.h"
  /**************************************************************************
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
**************************************************************************/

u8 flag;
int main(void)
{ 
  	Stm32_Clock_Init(9);            //=====系统时钟设置
  	delay_init(72);                 //=====延时初始化
	  uart_init(72,9600);             //=====初始化串口1  
    uart3_init(36,9600);            //=====串口3初始化
	  while(1)
		{     
		  static u8 a;
	    if(a++>100)
			a=1;
			flag=!flag;
		 	if(flag==0)  printf("{A%d:%d:%d:%d}$",a,a,40,a);//打印到APP上面
			else         printf("{B%d:%d:%d:%d}$",a+10,a*2,a-10,a-2);//打印到APP上面
      delay_ms(50);
		} 
}

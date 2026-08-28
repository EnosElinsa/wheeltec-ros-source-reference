#include "key.h"

/**************************************************************************
Function: Key initialization
Input   : none
Output  : none
函数功能：按键初始化
入口参数：无
返回  值：无 
**************************************************************************/
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable port clock //使能端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	
	//Port configuration //端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;	
  //Pull up input //上拉输入	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
	//Initialize GPIOD with the set parameters //根据设定参数初始化GPIOD
  GPIO_Init(GPIOB, &GPIO_InitStructure);					
} 
/**************************************************************************
Function: Buttons to scan
Input   : none
Output  : none
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击 
**************************************************************************/
u8 click(void)
{
	//Press the release sign
	//按键按松开标志
	static u8 flag_key=1;
	
	if(flag_key&&KEY==0)
	{
	 flag_key=0; //The key is pressed //按键按下
	 return 1;	
	}
	else if(1==KEY)			
		flag_key=1;
	return 0; //No key is pressed //无按键按下
}
/**************************************************************************
Function: Long according to the test
Input   : none
Output  : Key state 0: no action 1: long press 3s
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 1：长按3s
**************************************************************************/
u8 Long_Press(void)
{
	static int PressCount;
	int Pressed,PressTimeCount=10;
	if(KEY==0&&PressCount<(PressTimeCount+1)) PressCount++;
	if(KEY==1)
	{
		Delay_ms();
		if(KEY==1)PressCount=0,Pressed=0;
	}
	if((PressTimeCount)==PressCount) Pressed=1;
	
	if(Pressed==1) 
	{
		Pressed=0;
		return 1;
	}
	else return 0;
}
/**************************************************************************
Function: Delay function
Input   : none
Output  : none
函数功能：延迟函数
入口参数：无
返 回 值：无
**************************************************************************/
void Delay_ms(void)
{
   int ii,i;    
   for(ii=0;ii<50;ii++)
   {
	   for(i=0;i<500;i++);
	 }	
}

#include "control.h"		
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
函数功能：所有的控制代码都在这里面
          TIM1控制的定时中断 
**************************************************************************/
int TIM1_UP_IRQHandler(void)  
{    
	if(TIM_GetFlagStatus(TIM1,TIM_FLAG_Update)==SET)//5ms定时中断
	{   
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);                             //===清除定时器1中断标志位	  
		Position=Read_Position(4);                      //===更新位置信息
		Key();                                          //===按键控制目标值
		Moto=Position_PID(Position,Target_Position);    //===位置PID控制器
		Xianfu_Pwm();                                  //===PWM限幅
		Set_Pwm(Moto);
		
	}       	
	 return 0;	  
} 


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto)
{
    	if(moto<0)			AIN2=0,			AIN1=1;
			else 	          AIN2=1,			AIN1=0;
			PWMA=myabs(moto);
}



/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=7100;    //===PWM满幅是7200 限制在7100
	  if(Moto<-Amplitude) Moto=-Amplitude;	
		if(Moto>Amplitude)  Moto=Amplitude;		
}
/**************************************************************************
函数功能：按键修改运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)  
{	
	int tmp,Position_Amplitude=260; 
	tmp=click_N_Double(10);//检测按键 
	if(tmp==1)Target_Position+=Position_Amplitude;  //单击增加位置
	else if(tmp==2)Target_Position-=Position_Amplitude;  //单击增加位置
	
 }
/**************************************************************************
函数功能：取绝对值
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
int Position_PID (int position,int target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=target-position;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
	if(Integral_bias>3000)Integral_bias=3000;
	if(Integral_bias<-3000)Integral_bias=-3000;
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;                                           //增量输出
}

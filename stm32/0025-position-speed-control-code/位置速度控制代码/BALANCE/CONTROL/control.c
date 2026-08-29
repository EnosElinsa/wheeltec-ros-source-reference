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
		Encoder=Read_Velocity(4);                      //===更新位置信息
		Position+=Encoder;								//===速度积分得到位置
		Key();                                          //===按键控制目标值
		Position_Moto=Position_PID(Position,Target_Position);    //===位置PID控制器
		limit_a=Xianfu(Position_Moto,myabs(Target_Velocity));
		Moto=Incremental_PI(Encoder,limit_a);		//===速度PID控制器
		Moto=Xianfu(Moto,7000);                          //===PWM限幅
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
int Xianfu(int value,int Amplitude)
{	
	int temp;
	if(value>Amplitude) temp = Amplitude;
	else if(value<-Amplitude) temp = -Amplitude;
	else temp = value;
	return temp;
			
}
/**************************************************************************
函数功能：按键修改运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)  
{	
	int tmp,tmp1,tmp2,Position_Amplitude=10000,Velocity_Amplitude=10; 
	tmp=click_N_Double(20);//检测按键 
	tmp1=click_X();
	tmp2=click_M();
	if(tmp==1)Target_Position+=Position_Amplitude;  //单击增加位置
	else if(tmp==2)Target_Position-=Position_Amplitude;  //单击增加位置
	
	Target_Position=Xianfu(Target_Position,30000);
	
	
	if(tmp2==1) Target_Velocity+=Velocity_Amplitude;
	if(tmp1==1) Target_Velocity-=Velocity_Amplitude;
	Target_Velocity=Xianfu(Target_Velocity,40);
	
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
	Integral_bias=Xianfu(Integral_bias,myabs(Target_Velocity));
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	
	if(Pwm>10000)Pwm=10000;
	if(Pwm<-10000)Pwm=-10000;
	 return Pwm;                                           //增量输出
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Last_bias,Integral_bias;
	 Bias=Target-Encoder;//计算偏差
	 Integral_bias+=Bias;
	 Integral_bias=Xianfu(Integral_bias,5000);
	 Pwm=Velocity_KP*Bias+
		 Velocity_KI*Integral_bias+
		 Velocity_KD*(Bias-Last_bias);   //增量式PI控制器
	 Last_bias=Bias;	                                   //保存上一次偏差 
	if(Pwm>7000)Pwm=7000;
	if(Pwm<-7000)Pwm=-7000;
	 return Pwm;                                           //增量输出
}


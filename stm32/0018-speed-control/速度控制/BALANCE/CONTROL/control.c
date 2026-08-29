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
		Key();                                          //===按键控制目标值
		Moto=Incremental_PI(Encoder,Target_Velocity);    //===位置PID控制器
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
	int tmp,Velocity_Amplitude=10; 
	tmp=click_N_Double(10);//检测按键 
	if(tmp==1)Target_Velocity+=Velocity_Amplitude;  //单击增加位置
	else if(tmp==2)Target_Velocity-=Velocity_Amplitude;  //单击增加位置
	if(Target_Velocity>40)Target_Velocity=40;
	if(Target_Velocity<-40)Target_Velocity=-40;
	
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
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                                  //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
	 Last_bias=Bias;	                                   //保存上一次偏差 
	 return Pwm;                                           //增量输出
}

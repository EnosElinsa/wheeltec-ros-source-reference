/**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#ifndef __MOTOR_H
#define __MOTOR_H

#include "system.h"

//Control speed pins
//控制转速引脚
#define PWMA   TIM8->CCR2 
#define PWMB   TIM8->CCR1 

//Control positive and negative retractor pins
//控制正反转引脚
#define INA1   PCout(12)  
#define INB1   PDout(2)   
#define INA2   PBout(5)   
#define INB2   PBout(4)    

//Pins that determine whether the motor is allowed to move
//判断电机是否允许运动的引脚
#define EN     PAin(12) 

//Steering gear control pin
//舵机控制引脚
#define SERVO  TIM4->CCR2
//Actuator zero macro definition
//舵机零点宏定义
#define SERVO_INIT 1500     

void Enable_Pin(void);
void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
void Servo_PWM_Init(u16 arr,u16 psc);
#endif

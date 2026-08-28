
#ifndef __MOTOR_H
#define __MOTOR_H

#include "system.h"
  
//Control speed pins
//控制转速引脚
#define PWMD   TIM8->CCR1 
#define PWMC   TIM8->CCR2
#define PWMB   TIM8->CCR3 
#define PWMA   TIM8->CCR4 

//Control positive and negative retractor pins
//控制正反转引脚
#define INC1   PCout(12)  
#define IND1   PDout(2)  
#define INA1   PCout(5)  
#define INB1   PCout(4)  
#define INC2   PBout(5)  
#define IND2   PBout(4)  
#define INA2   PBout(1)  
#define INB2   PBout(0) 

//Pins that determine whether the motor is allowed to move
//判断电机是否允许运动的引脚
#define EN     PAin(12)  

void Enable_Pin(void);
void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
void Servo_PWM_Init(u16 arr,u16 psc);
#endif

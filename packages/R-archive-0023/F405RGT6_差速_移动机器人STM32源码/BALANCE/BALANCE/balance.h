#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"
#define BALANCE_TASK_PRIO		4     //Task priority //任务优先级
#define BALANCE_STK_SIZE 		512   //Task stack size //任务堆栈大小

extern int SLIDE_POSITION,SERVO_BIAS;
extern float Velocity_Smoother_Rate, Angle_Smoother_Rate;
extern int Servo_max, Servo_min;
#define FILTERING_TIMES 20

void Balance_task(void *pvParameters);
void Limit_Pwm(int amplitude);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
float float_abs(float insert);
int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_SERVO (float SlidePosition,float SlideTarget);
void Get_RC(void);
void PS2_control(void);
void Remote_Control(void);
void Set_Pwm(int motor_a,int motor_b,int servo);
void Drive_Motor(float Vx,float Vz);
void Key(void);
void Get_Velocity_Form_Encoder(void);
void robot_mode_check(void);
void Smooth_control(float vx, float step);
int Smooth_steering(int currentPWM, int targetPWM, float step);
int Mean_Filter(int data);
#endif  


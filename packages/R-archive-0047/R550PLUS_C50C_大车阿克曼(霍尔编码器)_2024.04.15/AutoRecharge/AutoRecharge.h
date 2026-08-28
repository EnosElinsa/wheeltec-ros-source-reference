#ifndef __AUTORECHARGE_H
#define __AUTORECHARGE_H
#include "sys.h"
#include "system.h"

extern u8 nav_walk, Allow_Recharge, Charging, RED_STATE;
extern float Charging_Current;
extern float Recharge_Red_Move_X, Recharge_Red_Move_Y, Recharge_Red_Move_Z; 
extern float Recharge_UP_Move_X, Recharge_UP_Move_Y, Recharge_UP_Move_Z; 
extern float Red_Docker_X, Red_Docker_Y, Red_Docker_Z; 
void CAN_Send_AutoRecharge(void);
extern u8 L_A,L_B,R_B,R_A;
extern u8 which_mode;
extern u8 show_in_windows;
extern u8 red_now_state;
extern u8 touch_state;
extern u8 refalsh_time;
#define cur_front_left do{                          \
			Recharge_Red_Move_X = -Red_Docker_X ,\
			Recharge_Red_Move_Z =  0.1f ;\
}while(0)

#define cur_front_right do{                          \
			Recharge_Red_Move_X = -Red_Docker_X ,\
			Recharge_Red_Move_Z = -0.1f ;\
}while(0)


#define front_left do{                          \
			Recharge_Red_Move_X = -Red_Docker_X ,\
			Recharge_Red_Move_Z =  Red_Docker_Z ;\
}while(0)

#define front_right do{                          \
			Recharge_Red_Move_X = -Red_Docker_X ,\
			Recharge_Red_Move_Z = -Red_Docker_Z ;\
}while(0)

#define back_left do{                          \
			Recharge_Red_Move_X = Red_Docker_X ,\
			Recharge_Red_Move_Z = -Red_Docker_Z ;\
}while(0)

#define back_right do{                          \
			Recharge_Red_Move_X = Red_Docker_X ,\
			Recharge_Red_Move_Z = Red_Docker_Z ;\
}while(0)

#define back do{                          \
			Recharge_Red_Move_X = Red_Docker_X ,\
			Recharge_Red_Move_Z = 0 ;\
}while(0)

#define front do{                          \
			Recharge_Red_Move_X = -Red_Docker_X ,\
			Recharge_Red_Move_Z = 0 ;\
}while(0)

#define stop do{                          \
			Recharge_Red_Move_X = 0 ,\
			Recharge_Red_Move_Z = 0 ;\
}while(0)

#endif

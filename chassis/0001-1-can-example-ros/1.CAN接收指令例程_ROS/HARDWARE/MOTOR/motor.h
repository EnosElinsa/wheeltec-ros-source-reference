#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
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
#define PWMA1   TIM1->CCR1  
#define PWMB1   TIM1->CCR2 
#define PWMC1   TIM1->CCR3 

#define PWMA2   TIM3->CCR3  
#define PWMB2   TIM3->CCR1 
#define PWMC2   TIM3->CCR2 
void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
#endif

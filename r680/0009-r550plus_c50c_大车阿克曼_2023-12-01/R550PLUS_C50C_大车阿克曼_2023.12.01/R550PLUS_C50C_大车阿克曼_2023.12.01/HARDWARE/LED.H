#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"

#define LED_TASK_PRIO		2     //Task priority //任务优先级
#define LED_STK_SIZE 		128   //Task stack size //任务堆栈大小

#define Braking PAout(8) 
/*--------Buzzer control pin--------*/
#define Buzzer_PORT GPIOD
#define Buzzer_PIN GPIO_Pin_11
#define Buzzer PDout(11)
/*----------------------------------*/

/*--------LED control pin--------*/
#define LED_PORT GPIOD
#define LED_R_PIN GPIO_Pin_14
#define LED_G_PIN GPIO_Pin_13
#define LED_B_PIN GPIO_Pin_15
#define LED_R PDout(14) 
#define LED_G PDout(13) 
#define LED_B PDout(15) 
/*----------------------------------*/

#define LED_Red(n)    n?(LED_R=0,LED_G=1,LED_B=1):(LED_R=1,LED_G=1,LED_B=1) /* 红 */
#define LED_Purple(n) n?(LED_R=0,LED_G=1,LED_B=0):(LED_R=1,LED_G=1,LED_B=1) /* 紫 */
#define LED_Blue(n)   n?(LED_R=1,LED_G=1,LED_B=0):(LED_R=1,LED_G=1,LED_B=1) /* 蓝 */
#define LED_Cyan(n)   n?(LED_R=1,LED_G=0,LED_B=0):(LED_R=1,LED_G=1,LED_B=1) /* 青 */
#define LED_Green(n)  n?(LED_R=1,LED_G=0,LED_B=1):(LED_R=1,LED_G=1,LED_B=1) /* 绿 */
#define LED_Yellow(n) n?(LED_R=0,LED_G=0,LED_B=1):(LED_R=1,LED_G=1,LED_B=1) /* 黄 */
#define LED_White(n)  n?(LED_R=0,LED_G=0,LED_B=0):(LED_R=1,LED_G=1,LED_B=1) /* 白 */

void LED_Init(void);  
void Buzzer_Init(void); 
void Led_Flash(u16 time);
void led_task(void *pvParameters);
void Braking_Init(void);
extern int Led_Count;
#endif

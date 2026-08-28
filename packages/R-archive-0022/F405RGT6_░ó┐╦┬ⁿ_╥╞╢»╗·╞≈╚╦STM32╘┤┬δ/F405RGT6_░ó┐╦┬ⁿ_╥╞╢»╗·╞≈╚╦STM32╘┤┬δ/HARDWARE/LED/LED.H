#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"

#define LED_TASK_PRIO		3     //Task priority //任务优先级
#define LED_STK_SIZE 		128   //Task stack size //任务堆栈大小

#define LED PBout(13) //LED Pin //LED引脚

void LED_Init(void);  
void Led_Flash(u16 time);
void led_task(void *pvParameters);
extern int Led_Count;
#endif

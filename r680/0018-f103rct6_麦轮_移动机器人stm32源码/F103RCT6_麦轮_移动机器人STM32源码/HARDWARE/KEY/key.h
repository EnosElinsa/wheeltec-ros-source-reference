#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#include "system.h"
void KEY_Init(void);
u8 click(void);
u8 Long_Press(void);
void Delay_ms(void);
#define KEY PBin(14)
#endif 

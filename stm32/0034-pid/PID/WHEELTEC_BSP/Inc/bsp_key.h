#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "sys.h"

#define KEY	 PEin(0) 
#define EN   PDin(3)  
void KEY_Init(void);
u8 KEY_Scan(u16 Frequency,u16 filter_times);
enum {
	key_stateless,
	single_click,
	double_click,
	long_click
};

#endif /* __BSP_KEY_H */

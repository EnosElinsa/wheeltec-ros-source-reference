#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "sys.h"

void LED_Init(void);

#define LED PEout(8) 

#endif /* __BSP_LED_H */


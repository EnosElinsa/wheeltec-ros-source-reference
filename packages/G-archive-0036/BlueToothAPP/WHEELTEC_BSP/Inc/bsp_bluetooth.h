#ifndef __BSP_BLUETOOTH_H
#define __BSP_BLUETOOTH_H

#include "sys.h"

typedef struct{
	uint8_t dirkey; //摇杆方向按键
	uint8_t page;   //APP页面记录
	uint8_t saveflash;//保存flash指令
	uint8_t reportparam;//上报数据请求指令
}WHEELTEC_APPKey_t;

extern WHEELTEC_APPKey_t wheeltecApp;

void uart2_init(u32 bound);
void BlueToothAPPDecode(uint8_t recv);

#endif /* __BSP_BLUETOOTH_H */


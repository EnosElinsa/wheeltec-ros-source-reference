#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
#include "system.h"
 
//CAN1 receives RX0 interrupt enablement
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	1	//0, not enabling;1, can make //0,不使能; 1,使能								    		

u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
u8 CAN1_Send_Num(u32 id,u8 *data);
u8 CAN1_Send_EXTid_Num(u32 id,u8 *data);

extern u8 can_state;
extern u8 liar_adjust;

extern u8 last_state;
#endif


















#include "can_callback.h"

void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;   
	
	u8 temp_rxbuf[8];
	
	//读取CAN1 FIFO0邮箱的数据
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	
	//将数据读出缓冲区使用
	memcpy(temp_rxbuf,RxMessage.Data,8);
	
	//标准帧ID数据处理
	if( RxMessage.IDE==CAN_Id_Standard )
	{
		switch( RxMessage.StdId ) //帧ID号
		{
			case 0x183: //帧ID 0x181为小车控制指令
			{
				//设置CAN控制模式
				if( Get_Control_Mode(_CAN_Control)==0 ) Set_Control_Mode(_CAN_Control);
				robot_control.command_lostcount = 0;//命令超时刷新
				
				//接受小车目标速度
				robot_control.Vx = ((float)((short)((temp_rxbuf[0]<<8)|(temp_rxbuf[1]))))/1000.0f;
				robot_control.Vy = ((float)((short)((temp_rxbuf[2]<<8)|(temp_rxbuf[3]))))/1000.0f;
				robot_control.Vz = ((float)((short)((temp_rxbuf[4]<<8)|(temp_rxbuf[5]))))/1000.0f;
				break;
			}
			
			default:
				break;
		}
	}
}

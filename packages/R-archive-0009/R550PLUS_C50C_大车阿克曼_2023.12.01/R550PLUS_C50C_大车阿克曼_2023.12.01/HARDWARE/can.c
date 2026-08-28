#include "can.h"
#include "system.h"
/**************************************************************************
Function: CAN1 initialization
Input   : tsjw：Resynchronize the jump time unit, Scope: 1 ~ 3;
 			    tbs2：Time unit of time period 2, range :1~8;
 			    tbs1：Time unit of time period 1, range :1~16;
 			    brp ：Baud rate divider, range :1 to 1024;(We're actually going to add 1, which is 1 to 1024) tq=(brp)*tpclk1
 			    mode：0, normal mode;1. Loop mode;
Output  : 0- Initialization successful;Other - initialization failed
Note: none of the entry parameters (except mode) can be 0
函数功能：CAN1初始化
入口参数：tsjw：重新同步跳跃时间单元，范围:1~3;
 			    tbs2：时间段2的时间单元，范围:1~8;
 			    tbs1：时间段1的时间单元，范围:1~16;
 			    brp ：波特率分频器，范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
 			    mode：0,普通模式;1,回环模式;
返回  值：0-初始化成功; 其他-初始化失败
注意：入口参数(除了mode)均不能为0
波特率/Baud rate=Fpclk1/((tbs1+tbs2+1)*brp)，Fpclk1为36M
                =42M/((3+2+1)*6)
						    =1M
**************************************************************************/
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
	//-1是因为这些配置实际数值从0开始
	//brp不需要-1，因为其配置数值是从1开始
 	if(tsjw==0||tbs2==0||tbs1==0||brp==0) return 1;
	tsjw-=1; //Subtract 1 before setting //先减去1.再用于设置
	tbs2-=1;
	tbs1-=1;
	
	#if CAN1_RX0_INT_ENABLE 
	NVIC_InitTypeDef  NVIC_InitStructure;
	#endif
	
	//使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	

	//初始化GPIO
	//CAN1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);           
	
	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
	
	//CAN单元设置
	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
	CAN_InitStructure.CAN_AWUM=DISABLE; //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode= mode;	//模式设置 
	
	//波特率相关配置
	CAN_InitStructure.CAN_SJW=tsjw;	    //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1;     //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;     //Tbs2范围CAN_BS2_1tq ~CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;//分频系数(Fdiv)为brp+1	
	
	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 	
	
	//配置过滤器
	//CAN1
	CAN_FilterInitStructure.CAN_FilterNumber=0;	                   //过滤器0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  //屏蔽模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;               //32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000; 	

	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);            //滤波器初始化
	
	//CAN1 FIFO0 中断使能
	#if CAN1_RX0_INT_ENABLE
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;// 主优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;       // 次优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	//CAN2 FIFO1 中断使能
	#if CAN2_RX1_INT_ENABLE
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);//FIFO1消息挂号中断允许.		    
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	return 0;
}   


// CAN1发送函数
u8 CAN1_Send_Num(u32 id,u8 *data)
{
	//CAN发送数据结构体
	CanTxMsg msg;
	
	u16 i;   //超时时间
	u8 mbox; //
	
	#if 1 //配置标准帧与id
	msg.StdId = id;
	msg.IDE = CAN_Id_Standard;
	#else //配置扩展帧与id
	msg.ExtId = id;
	msg.IDE = CAN_Id_Extended;
	#endif
	
	//发送的报文属于数据帧
	//数据帧 CAN_RTR_DATA
	//遥控帧 CAN_RTR_REMOTE
	msg.RTR = CAN_RTR_DATA;
	
	//发送的数据长度
	// 1~8
	msg.DLC = 8;
	
	//将要发送的数据复制入 msg.Data
	memcpy(msg.Data,data,8);
	
	//CAN1 发送
	CAN_Transmit(CAN1,&msg);
	
	//等待CAN1发送完成
	while( CAN_TransmitStatus(CAN1,mbox)== CAN_TxStatus_Failed && i<0xffff ) i++;
	
	//发送超时
	if(i>=0xffff) return 1;
	
	return 0;
}

u8 CAN1_Send_EXTid_Num(u32 id,u8 *data)
{
	//CAN发送数据结构体
	CanTxMsg msg;
	
	u16 i;   //超时时间
	u8 mbox; //
	
	#if 0 //配置标准帧与id
	msg.StdId = id;
	msg.IDE = CAN_Id_Standard;
	#else //配置扩展帧与id
	msg.ExtId = id;
	msg.IDE = CAN_Id_Extended;
	#endif
	
	//发送的报文属于数据帧
	//数据帧 CAN_RTR_DATA
	//遥控帧 CAN_RTR_REMOTE
	msg.RTR = CAN_RTR_DATA;
	
	//发送的数据长度
	// 1~8
	msg.DLC = 8;
	
	//将要发送的数据复制入 msg.Data
	memcpy(msg.Data,data,8);
	
	//CAN1 发送
	CAN_Transmit(CAN1,&msg);
	
	//等待CAN1发送完成
	while( CAN_TransmitStatus(CAN1,mbox)== CAN_TxStatus_Failed && i<0xffff ) i++;
	
	//发送超时
	if(i>=0xffff) return 1;
	
	return 0;
}

/**************************************************************************
Function: CAN receives interrupt service function, conditional compilation
Input   : none
Output  : none
函数功能：CAN接收中断服务函数，条件编译
入口参数：无
返回  值：无 
**************************************************************************/
#if CAN1_RX0_INT_ENABLE	//Enable RX0 interrupt //使能RX0中断	   
u8 can_state = 0;
u8 liar_adjust=0;
u8 last_state=0;//原本是静态变量
u16 adjust_timeout=0;
u16 double_adjust_timeout=0;

void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;   
	u8 temp_rxbuf[8];
	
	//读取CAN1 FIFO0邮箱的数据
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	
	//把数据读出到缓冲区
	memcpy(temp_rxbuf,RxMessage.Data,8);
	
	//回充装备特征码
	if(RxMessage.ExtId==0x12345678)
	{
		u8 check=0;
		for(u8 i=0;i<8;i++) 
		{
			check += temp_rxbuf[i];
		}
		if( check==8 ) Get_Charging_HardWare=1;
	}
	
	if(RxMessage.StdId==0x181)
	{
		CAN_ON_Flag=1,PS2_ON_Flag=0,APP_ON_Flag=0,Remote_ON_Flag=0,Usart_ON_Flag=0;
		command_lost_count=0; //CAN/串口控制命令丢失计数清零
		//Calculate the three-axis target velocity, unit: m/s
		//计算三轴目标速度，单位：m/s
		Move_X=((float)((short)((temp_rxbuf[0]<<8)|(temp_rxbuf[1]))))/1000; 
		Move_Y=((float)((short)((temp_rxbuf[2]<<8)|(temp_rxbuf[3]))))/1000;
		Move_Z=((float)((short)((temp_rxbuf[4]<<8)|(temp_rxbuf[5]))))/1000;
	}
	
	//////////////// 自动回充数据 ////////////////
	else if(RxMessage.StdId==0x182) //频率：20ms进入1次中断
	{
		charger_check=0;//回充装备活跃状态
		
		//保存与刷新上一次的状态
//		static u8 last_state=0;//原本是静态变量
		static u8 tmp_state=0;
		static u8 last_touchstate=0;
		
		//接触弹片后的状态锁定
		static u8 state_lock = 0;
		
		//接触弹片后离开时的姿态标志
		static u8 change_state = 0;
		
		//接触弹片后离开时的走时内核
		static u16 time_core = 0;
		
		//接触弹片时的滤波消抖变量
		static u16 filter_cur = 0,filter_vol = 0;
		
		//小车充电姿态的调整次数
		static u8 adjust_times = 0;
		static u8 adjust_vol = 0 , adjust_cur = 0;
		
		
		//单边充电标志位
		static u8 liar_charge = 0;
		
		//两边都接触到了，标记调整1次
		if(adjust_vol&&adjust_cur) adjust_vol = 0,adjust_cur = 0,adjust_times++;
		
		//红外状态解算
		L_A = (temp_rxbuf[6]>>5)&0x01;
		L_B = (temp_rxbuf[6]>>4)&0x01;
		R_B = (temp_rxbuf[6]>>3)&0x01;
		R_A = (temp_rxbuf[6]>>2)&0x01;
		
		//用于检查充电区、测压区接触情况
		touch_state = temp_rxbuf[2];
		if( last_touchstate!=0xAA&&touch_state==0xAA ) adjust_timeout=0,liar_adjust++;//单边状态调整记录
		if( (last_touchstate!=0xAA&&touch_state==0xAA) || (last_state!=0xBB&&touch_state==0xBB) ) double_adjust_timeout=0;//任意一边接触都清空双边延迟时间
		last_touchstate = touch_state;
		
		//两边都无接触超过5秒，清空调整次数标定
		if(adjust_times>0) double_adjust_timeout++;
		if( double_adjust_timeout>250 ) adjust_times=0,double_adjust_timeout=0;
		
		//充电边无接触5秒，清空充电边调整次数标定
		if( liar_adjust>0 ) adjust_timeout++;
		if( adjust_timeout>250 ) liar_adjust=0,adjust_timeout=0;//接触间隔大于5秒，判断为异常情况，清空调整次数记录。
		
		//debug信息
		can_state = temp_rxbuf[3];
		
		//检测到已经脱离充电口，则需要复位
		if(liar_charge==1&&touch_state!=0xAA&&touch_state!=0xCF) 
		{
			if( touch_state==0xAB && Voltage>=25.0f )//充满电了
			{
				Allow_Recharge=0;
			}
			adjust_times = 0,liar_charge=0;
		}	
		
		//单边充电，自定义标志位touch_state
		if(liar_charge) touch_state = 0xFC;
		
		/* 针对不同车型设置不同参数 */
		u16 leave_times = 75; //接触弹片后离开的时间 75*20ms = 1.5s
		u8 yuzhi = 40;        //接触弹片后消抖的时间 40*20ms = 800ms
		if(Allow_Recharge==0) 
		{
			adjust_times = 0,liar_charge=0,liar_adjust=0;//关闭自动回充时，标定已调整的次数为0
			if( Car_Mode==2 || Car_Mode==3 ) Velocity_KP=300,Velocity_KI=300; 
			if( Car_Mode==4 ) Velocity_KP=400,Velocity_KI=100; 
			if( Car_Mode==5 ) Velocity_KP=50,Velocity_KI=200; 
		}
		else
		{	
			if( Car_Mode==0 || Car_Mode==1 ) leave_times=80,refalsh_time=100; //原80，50
			if( Car_Mode==2 || Car_Mode==3 ) Velocity_KP=600,Velocity_KI=600,refalsh_time=100; 
			if( Car_Mode==4 || Car_Mode==5 ) 
			{
				//Red_Docker_X=-0.1f, Red_Docker_Y=0, Red_Docker_Z=0.2f; 默认值
				Red_Docker_X=-0.08f, Red_Docker_Z=0.15f;//车体较重，对接速度尽量减慢
				Velocity_KP=600,Velocity_KI=600;
				leave_times=113;//顶配独立悬挂电机响应较慢，需要更长的调节时间
				refalsh_time=150;//红外状态刷新时间(电机响应越慢，建议的时间越长，跟电机减速比、小车负载都有关系)
			}
		}
	
		//将红外融合成一个数值表示状态，方便给状态标号
		red_now_state = L_A << 3 | L_B << 2 | R_B << 1 | R_A << 0 ;
		
		if( Allow_Recharge )//自动回充开启后才记录状态
		{
			//红外状态发生改变时，保存上一个红外状态
			if(red_now_state!=tmp_state) last_state = tmp_state;	
			tmp_state = red_now_state;
		}
		
		////////////////////////////  对接逻辑处理 开始 ////////////////////////////	
		RED_STATE = L_A + L_B + R_B + R_A;
		
		     if(L_A==0&&L_B==0&&R_B==0&&R_A==1)  front_right; //1
			 
		else if(L_A==0&&L_B==0&&R_B==1&&R_A==0)  back_left;  //2
		
		else if(L_A==0&&L_B==0&&R_B==1&&R_A==1)  front_right;  //3

		else if(L_A==0&&L_B==1&&R_B==0&&R_A==0)  //4
		{
			front_left; 
		}	
		
		// L_A==0&&L_B==1&&R_B==0&&R_A==1 不存在 5 
		
		else if(L_A==0&&L_B==1&&R_B==1&&R_A==0) //6
		{
			front_left;   
			if(last_state==2) back;
			
		}	
		
		else if(L_A==0&&L_B==1&&R_B==1&&R_A==1)  back; //7
		
		else if(L_A==1&&L_B==0&&R_B==0&&R_A==0) 
		{
			back_right; //8
			if( last_state==9 ) back;
		}

		else if(L_A==1&&L_B==0&&R_B==0&&R_A==1)   //9
		{
			front_right;
			if( last_state==8||last_state==1 ) back;
		}
		
		else if (L_A==1&&L_B==0&&R_B==1&&R_A==0)  back; //10
		
		else if(L_A==1&&L_B==0&&R_B==1&&R_A==1) // 11
		{
			front_right;
			if(last_state==9) back;
		}
		
		else if(L_A==1&&L_B==1&&R_B==0&&R_A==0)  back_right;  //12	
		
		else if(L_A==1&&L_B==1&&R_B==0&&R_A==1)
		{
			back; //13
		}			
		
		else if(L_A==1&&L_B==1&&R_B==1&&R_A==0)  //14
		{
			front_left; 
			if(last_state==6||last_state==3) back;
		}			

		else if(L_A==1&&L_B==1&&R_B==1&&R_A==1)  back;		  //15
		
		//其他情况，全0或者5
		else
		{
			RED_STATE = 0;//红外识别状态设置为0
			stop;
		}

		
		if(touch_state==0xAA||state_lock==1)//充电装备接触到了充电区
		{
			stop;		

			filter_cur++;   
			if(filter_cur>yuzhi) //延迟消抖
			{
				adjust_cur = 1; //标记接触
				
				filter_cur = yuzhi+1;//滤波变量自锁，防止循环溢出
				
				state_lock = 1;//锁定接触状态，表示在更新状态前，会一直进入本判断
				
				cur_front_right;//调整姿态
				
				//调整次数超过了限制，又接触到了充电区，不再调整了；使用单边充电
				u8 allow_adjust = 2; //对接后允许调整位姿的次数
				if(adjust_times==allow_adjust||( liar_adjust>=3&&adjust_times>0 ))
				{
					stop;		
					liar_charge = 1;//不调整了，开启单边充电
					liar_adjust=0;
				}
			}
			
		}
		else if(touch_state==0xBB||state_lock==2)//接触到了测压区
		{
			stop;
			
			filter_vol++;   
			if(filter_vol>yuzhi) //延迟消抖
			{  
				adjust_vol = 1; //标记接触1次
				
				filter_vol = yuzhi+1;
				state_lock = 2;
				cur_front_left;//调整姿态
			}
		}
		else if(touch_state==0xCF||touch_state==0xFC)
		{
			liar_charge=1;//同时接触测压区和充电区时，开放单边充电，提高稳定性
			liar_adjust=0;
			stop;
		}
		
		//弹片接触状态更新
		if(state_lock!=0)
		{
			time_core++;
			if(time_core> leave_times )
			{
				if(state_lock==2) change_state = 2 ;
				if(state_lock==1) change_state = 1 ;
				state_lock = 0,time_core=0,filter_vol=0,filter_cur=0;
			}				
		}
		else
			time_core =  0;
		
		//弹片状态更新后，屏蔽掉全1以外的所有红外情况
		//接触过弹片又离开，则只接收全红外后退的数据，其他数据一律前进，无需转向
		if(change_state!=0)
		{
			static u8 shielding=0;
			static u8 leaveCount=0;	

			if(++leaveCount==255) leaveCount=0, change_state = 0,shielding=0;//限制总体的调整时间
			
			if(red_now_state==9) {front_right;shielding++;}//特殊状态，靠太边了
			else if( red_now_state==6 ) {front_left;shielding++;}//特殊状态，靠太边了
			else if( red_now_state!=15 ) {front;shielding=0;}
			
			//如果该变量一直自增，说明屏蔽时间太长了，清空调整次数重新开始
			if(shielding>250) adjust_times=0,liar_adjust=0;
			
			if(last_state==15 || last_state==9 || last_state==6 ) leaveCount=0,change_state = 0,shielding=0;
		}
		
		//Z速度转换为角度
		Recharge_Red_Move_Z = Vz_to_Akm_Angle(Recharge_Red_Move_X,Recharge_Red_Move_Z);
		
		////////////////////////////  对接逻辑处理 结束 ////////////////////////////
		
		if(liar_charge==1) Charging = 1;//机器人正在充电
		else Charging = 0;
		
		if(Charging==1) Recharge_Red_Move_X = Recharge_Red_Move_Y = Recharge_Red_Move_Z = 0;
		
		//充电电流换算
		if(temp_rxbuf[7]>128)Charging_Current=-(256-temp_rxbuf[7])*30;
		else Charging_Current=(temp_rxbuf[7]*30);
			
	}
}
#endif

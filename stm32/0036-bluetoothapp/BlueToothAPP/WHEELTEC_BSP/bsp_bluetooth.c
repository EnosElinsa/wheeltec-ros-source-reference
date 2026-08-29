#include "bsp_bluetooth.h"
#include <string.h>

//定义APP相关的控制键值
WHEELTEC_APPKey_t wheeltecApp = { 0 };

void uart2_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //Enable the Usart clock //使能USART时钟

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_USART2);	 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //初始化

	//UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;
	//Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	//Enable the IRQ channel //IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器		
	NVIC_Init(&NVIC_InitStructure);	

	//USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
	USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2 //初始化串口2

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART2, ENABLE);                     //Enable serial port 2 //使能串口2 
}


//JDY-33蓝牙AT指令集过滤
//在蓝牙被连接或断开时，蓝牙模块会主动上报一些AT指令信息到串口,需要将此部分信息过滤掉以免造成数据误判或者干扰
static uint8_t ATCommandFeedBack_JDY33(uint8_t recv)
{
	#define DEBUG_JDY33Command 0
	
	uint8_t isFilter = 0;//是否允许该字符通过,1过滤,0允许
	static uint8_t lastrecv = 0;
	static uint8_t filterIndex = 0;
	
	const char* JDY33_SPPConnect = "+CONNECTING<<XX:XX:XX:XX:XX:XX\r\nCONNECTED\r\r\n";
	const char* JDY33_BLEConnect = "CONNECTED\r\r\n";
	const char* JDY33_DisConnect = "+DISC:SUCCESS\r\r\n\0"; //注意\0不会被统计字长,需要自行把字长+1
	enum{
		JDY33_NORMAL=  0,
		JDY33_SPPCONNECTSTART,
		JDY33_BLECONNECTSTART,
		JDY33_DISCONNECTSTART,
	};
	
	static uint8_t statemachine = JDY33_NORMAL;
	
	switch( statemachine )
	{
		case JDY33_NORMAL:
			if( recv=='C'&&lastrecv=='+' )
			{
				statemachine = JDY33_SPPCONNECTSTART;//接收到特征值,开始匹配
				isFilter = 1;//过滤字符
				filterIndex = 2; //2号开始索引
			}
			else if( recv=='O'&&lastrecv=='C' )
			{
				statemachine = JDY33_BLECONNECTSTART;//接收到特征值,开始匹配
				isFilter = 1;//过滤字符
				filterIndex = 2; //2号开始索引
			}
			else if( recv=='D'&&lastrecv=='+' )
			{
				statemachine = JDY33_DISCONNECTSTART;//接收到特征值,开始匹配
				isFilter = 1;//过滤字符
				filterIndex = 2; //2号开始索引
			}
			else if( recv=='C'&&lastrecv!='C' )//进入状态的判断优先,再到歧义的判断
			{
				//有关C的歧义,"+C"、"CO"不能通过. "?C也不允许通过",才能保证连接时不存在控制命令,若需要控制小车,需要连续的C
				isFilter = 1;//禁止通过
			}
			break;
		case JDY33_SPPCONNECTSTART:
			if( JDY33_SPPConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
			{
				isFilter = 1;//匹配连接字段,若完成匹配则过滤
				
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else if( (filterIndex>=13&&filterIndex<=29) && \
       				  ((recv>='0'&&recv<='9')||(recv>='A'&&recv<='Z')) )//进入到MAC地址匹配阶段.该阶段匹配 0~9 、a~z字段
			{
				isFilter = 1;//MAC地址过滤
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				//都不满足,允许字符通过.并退出过滤模式
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command 
				printf("SPP->No:get:%c,but:%c\r\n",recv,JDY33_SPPConnect[filterIndex]);
				#endif
			}
			
			//索引直至完成过滤列表
			filterIndex++;
			if( filterIndex == strlen(JDY33_SPPConnect) )
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command 
				printf("SPP filter con done!\r\n");
				#endif
			}
			break;
		case JDY33_BLECONNECTSTART:
			if( JDY33_BLEConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
			{
				isFilter = 1;//匹配连接字段,若完成匹配则过滤
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("BLE->No:get:%c,but:%c\r\n",recv,JDY33_BLEConnect[filterIndex]);
				#endif
			}				
			
			//索引直至完成过滤列表
			filterIndex++;
			if( filterIndex == strlen(JDY33_BLEConnect) ) 
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("ble filter dis done!\r\n");
				#endif
			}
			break;
		case JDY33_DISCONNECTSTART:
			if( JDY33_DisConnect[filterIndex] == recv )  //开始过滤连接字符,逐个字节匹配
			{
				isFilter = 1;//匹配连接字段,若完成匹配则过滤
				#if 1== DEBUG_JDY33Command 
				printf("yes:%c\r\n",recv);
				#endif
			}
			else
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("dis->No:get:%c,but:%c\r\n",recv,JDY33_DisConnect[filterIndex]);
				#endif
			}				
			
			//索引直至完成过滤列表
			filterIndex++;
			if( filterIndex == strlen(JDY33_DisConnect)+1 ) //+1为补充空字符'\0'
			{
				statemachine = JDY33_NORMAL;
				#if 1== DEBUG_JDY33Command
				printf("filter dis done!\r\n");
				#endif
			}
			break;
	}
	lastrecv = recv;
	
	return isFilter;
}

//手机APP数据解码
void BlueToothAPPDecode(uint8_t recv)
{
	//APP调参页面辅助参数
	static uint8_t paramFlag=0,param_i=0,param_j=0,paramReceive[50]={0};
	float paramData=0;
	
	//过滤蓝牙的AT指令反馈
	uint8_t ATFilter = 0;
	ATFilter += ATCommandFeedBack_JDY33(recv);
	if( ATFilter > 0 ) return; //过滤
	
	/* APP按键页面切换 */
	if( recv == 'K' ) wheeltecApp.page = 2;      //按键页面
	else if( recv == 'J' ) wheeltecApp.page = 1; //摇杆页面
	else if( recv == 'I' ) wheeltecApp.page = 0; //重力球页面
	
	//方向按键
	wheeltecApp.dirkey = recv-0x40;
	
//	switch( wheeltecApp.dirkey )
//	{
//		case 1:   break;
//		case 2:   break;
//		case 3:   break; 
//		case 4:   break;
//		case 5:   break;
//		case 6:   break;
//		case 7:   break;
//		case 8:   break;
//		case 9:   break;
//		case 10:  break;
//		default:  break;
//	}
	
	//APP参数页面 数据格式: {?:?}
	if(recv==0x7B) paramFlag=1;        //The start bit of the APP parameter instruction //APP参数指令起始位
	else if(recv==0x7D) paramFlag=2;   //The APP parameter instruction stops the bit    //APP参数指令停止位
	
	if(paramFlag==1) //Collect data //采集数据
	{
		paramReceive[(param_i%50)]=recv;
		param_i++;
	}
	else if(paramFlag==2) //Analyze the data //分析数据
	{
		if(paramReceive[3]==0x50)  // {Q:P} 获取设备参数
		{
			wheeltecApp.reportparam = 1;//获取参数请求
		}
			
		else if( paramReceive[3]==0x57 ) // {Q:W} 设置掉电保存参数
		{
			wheeltecApp.saveflash = 1;//保存Flash参数请求
		}					
		
		else  if(paramReceive[1]!=0x23)  // {0:xxx} {1:xxx} {2:xxx} 单通道数值设置
		{
			for(param_j=param_i; param_j>=4; param_j--)
			{
				paramData+=(paramReceive[param_j-1]-48)*pow(10,param_i-param_j);
			}
			switch(paramReceive[1])
			{
				case 0x30: TestAPP_Param1 = paramData;break;
				case 0x31: TestAPP_Param2 = paramData;break;
				case 0x32: break;
				case 0x33: break;
				case 0x34: break;
				case 0x35: break;
				case 0x36: break;
				case 0x37: break;
				case 0x38: break;
			}
		}
		else if( paramReceive[1]==0x23 ) //APP上点击“发送所有数据”处理方法  // {#xxx:xxx:xxx...xxx}
		{
			float num=0;
			uint8_t dataIndex=0;
			float dataArray[9]={0};

			if( param_i<=50 ) //数据在可接受范围
			{
				paramReceive[param_i]='}'; //补充帧尾

				for(uint8_t kk=0; paramReceive[kk]!='}'; kk++)
				{
					if( paramReceive[kk]>='0' && paramReceive[kk]<='9' )
					{
						num = num*10 + ( paramReceive[kk] - '0' );
					}
					else if( paramReceive[kk]==':' )
					{
						dataArray[dataIndex++] = num;
						num = 0;
					}

				}
				//处理最后一个数据
				dataArray[dataIndex] = num;
				
				//数据使用
				TestAPP_Param1=dataArray[0];
				TestAPP_Param2=dataArray[1];
			}
		}
		
		//Relevant flag position is cleared
		//相关标志位清零
		paramFlag=0;param_i=0;param_j=0;paramData=0;
		memset(paramReceive, 0, sizeof(uint8_t)*50); //Clear the array to zero//数组清零
	}
}



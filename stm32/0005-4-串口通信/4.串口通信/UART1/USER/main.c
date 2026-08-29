#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

void USART1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStrue; //定义一个引脚初始化的结构体
	USART_InitTypeDef USART_InitStrue; //定义一个串口初始化的结构体
	NVIC_InitTypeDef NVIC_InitStrue; //定义一个中断优先级初始化的结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //GPIOA时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE); //串口1时钟使能
	
	GPIO_InitStrue.GPIO_Mode=GPIO_Mode_AF_PP; //A9引脚作为串口1发送数据引脚，推挽复用输出
	GPIO_InitStrue.GPIO_Pin=GPIO_Pin_9; //引脚9
	GPIO_InitStrue.GPIO_Speed=GPIO_Speed_10MHz; //作为串口发送数据引脚时该速度可以为任意
  GPIO_Init(GPIOA,&GPIO_InitStrue); //根据上面设置好的GPIO_InitStructure参数进行初始化
	
	GPIO_InitStrue.GPIO_Mode=GPIO_Mode_IN_FLOATING; //A10引脚作为串口1接收数据引脚，浮空输入或带上拉输入
	GPIO_InitStrue.GPIO_Pin=GPIO_Pin_10; //引脚10
	GPIO_InitStrue.GPIO_Speed=GPIO_Speed_10MHz; //作为串口接收数据引脚时该速度可以为任意
  GPIO_Init(GPIOA,&GPIO_InitStrue); //根据上面设置好的GPIO_InitStructure参数进行初始化
	
	USART_InitStrue.USART_BaudRate=9600; //定义串口波特率为9600bit/s
	USART_InitStrue.USART_HardwareFlowControl=USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStrue.USART_Mode=USART_Mode_Tx|USART_Mode_Rx; //发送接收兼容模式
	USART_InitStrue.USART_Parity=USART_Parity_No; //无奇偶校验位
	USART_InitStrue.USART_StopBits=USART_StopBits_1; //一个停止位
	USART_InitStrue.USART_WordLength=USART_WordLength_8b; //字长为8位数据格式
	USART_Init(USART1,&USART_InitStrue);//根据上面设置USART_InitStrue参数初始化串口1
	
	USART_Cmd(USART1,ENABLE); //使能串口1
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE); //使能接收中断void USART1_IRQHandler(void)
	
	NVIC_InitStrue.NVIC_IRQChannel=USART1_IRQn; //属于串口1中断
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE; //中断使能
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1; //抢占优先级为1级，值越小优先级越高，0级优先级最高
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=1; //响应优先级为1级，值越小优先级越高，0级优先级最高
	NVIC_Init(&NVIC_InitStrue); ////根据NVIC_InitStrue的参数初始化VIC寄存器，设置串口1中断优先级
}


//串口中断函数
void USART1_IRQHandler(void)
{
	  u8 Res; //串口一次只能读取1个字符
		if(USART_GetITStatus(USART1, USART_IT_RXNE)) //获取串口1接收状态标志位
		{
		 Res =USART_ReceiveData(USART1);	//读取接收到的数据
		 USART_SendData(USART1, Res);  //将接收到的数据原样发送
		 while(USART_GetFlagStatus(USART1, USART_FLAG_TC)!=SET); //等待发送结束
	   //USART1->DR=USART_RX_BUF[t]; //寄存器版本串口发送数据方法，数据寄存器 USART_DR可以存放要发送的数据
		 //while((USART1->SR&0X40)==0); //USART_SR 寄存器的TC位置0时数据发送结束
    } 
}

int main(void)
{	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断优先级分组，即优先级分级个数
	                                                //NVIC_PriorityGroup_2，代表抢占优先级位数位2，可以分[0, 1, 2, 3]四级优先级
	                                                //                          响应优先级位数位2，可以分[0, 1, 2, 3]四级优先级
	USART1_Init(); //调用串口1初始化函数
	while(1)
	{
	  //本次例程无主函数，用户可自定义
	}
}


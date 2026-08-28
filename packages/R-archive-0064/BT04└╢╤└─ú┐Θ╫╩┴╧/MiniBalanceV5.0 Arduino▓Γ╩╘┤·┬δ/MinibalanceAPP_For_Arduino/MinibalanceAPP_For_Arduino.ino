/****************************************************************************
   作者：平衡小车之家
   产品名称：Minibalance For Arduino
****************************************************************************/
int a,flag;
/**************************************************************************
函数功能：初始化 相当于STM32里面的Main函数 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void setup() {
  Serial.begin(9600);       //开启串口，设置波特率为 9600
}
/**************************************************************************
函数功能：获取遥控控制信号
入口参数：无
返回  值：无
**************************************************************************/
void RemoteControl(){ 
  if (Serial.available())   //如果有收到数据
{
    switch (Serial.read())
    {
//     //这是MinibalanceV1.0的APP发送指令
//      case 0x01: Flag_Qian = 1, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0;   break;              //前进
//      case 0x02: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
//      case 0x03: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
//      case 0x04: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
//      case 0x05:  Flag_Qian = 0, Flag_Hou = 1, Flag_Left = 0, Flag_Right = 0;   break;             //后退
//      case 0x06: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;  break;               //左转
//      case 0x07: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0; break;                //左转
//      case 0x08: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;   break;              //左转
//      //这是MinibalanceV3.5的APP发送指令
//      case 0x41: Flag_Qian = 1, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0;   break;              //前进
//      case 0x42: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;             //右转
//      case 0x43: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;             //右转
//      case 0x44: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
//      case 0x45:  Flag_Qian = 0, Flag_Hou = 1, Flag_Left = 0, Flag_Right = 0;   break;             //后退
//      case 0x46: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;  break;               //左转
//      case 0x47: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0; break;               //左转
//      case 0x48: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;   break;             //左转
//      default: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0; break;                //停止
    }
  }
}
/**************************************************************************
函数功能：主循环程序体
入口参数：无
返回  值：无
**************************************************************************/
void loop() {
    a++;
    if(a>100)  a=0;
    flag=!flag;
   if(flag==0)
    {
    Serial.print("{A");
    Serial.print(a);   //左轮编码器
    Serial.print(":");
    Serial.print(a);  //右轮编码器
    Serial.print(":");
    Serial.print(40);  //电池电压
    Serial.print(":");
    Serial.print(a);  //平衡倾角
    Serial.print("}$");
    }
      else
    {
    Serial.print("{B");
    Serial.print(a+10);   
    Serial.print(":");
    Serial.print(a);  
    Serial.print(":");
    Serial.print(a*2); 
    Serial.print(":");
    Serial.print(a-20); 
    Serial.print("}$");
    }
    delay(50);
    RemoteControl();   //接收遥控器的控制指令
}



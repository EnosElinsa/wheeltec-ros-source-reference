#ifndef MPU6050_H
#define MPU6050_H
#include "ioi2c.h"												  	  
#include "usartx.h" 
void MPU6050_task(void *pvParameters);

#define MPU_I2CMST_DELAY_REG	0X67	//IIC host delay management register //IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//Signal channel reset register //信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG	0X69	//Motion detection control register //运动检测控制寄存器
#define MPU_USER_CTRL_REG		  0X6A	//User control register //用户控制寄存器
#define MPU_PWR_MGMT1_REG		  0X6B	//Power Management Register 1 //电源管理寄存器1
#define MPU_PWR_MGMT2_REG		  0X6C	//Power Management Register 2 //电源管理寄存器2 
#define MPU_FIFO_CNTH_REG		  0X72	//The FIFO count register is eight bits higher //FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		  0X73	//The FIFO count register is eight bits lower //FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			  0X74	//FIFO read-write register //FIFO读写寄存器
#define MPU_DEVICE_ID_REG		  0X75	//Device ID register //器件ID寄存器
#define MPU_ADDR				      0X68

//The internal register of MPU6500
//MPU6500的内部寄存器
#define MPU_SELF_TESTX_REG		0X0D	//Check register X //自检寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//Check register Y //自检寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//Check register Z //自检寄存器Z
#define MPU_SELF_TESTA_REG		0X10	//Check register A //自检寄存器A
#define MPU_SAMPLE_RATE_REG		0X19	//Sampling frequency divider //采样频率分频器
#define MPU_CFG_REG				    0X1A	//Configure the register //配置寄存器
#define MPU_GYRO_CFG_REG		  0X1B	//The gyroscope configuration register //陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		  0X1C	//Accelerometer configuration register //加速度计配置寄存器
#define MPU_MOTION_DET_REG		0X1F	//Motion detection threshold setting register //运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG			  0X23	//FIFO Enable register //FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG		0X24	//IIC host control register //IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC slave 0 device address register //IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG			  0X26	//IIC slave 0 data address register //IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC slave 0 control register //IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC slave 1 device address register //IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG			  0X29	//IIC slave 1 data address register //IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC slave 1 control register //IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC slave 2 device address register //IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG			  0X2C	//IIC slave 2 data address register //IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC slave 2 control register //IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC slave 3 device address register //IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG			  0X2F	//IIC slave 3 data address register //IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC slave 3 control register //IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC slave 4 device address register //IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG			  0X32	//IIC slave 4 data address register//IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC slave 4 writes data register //IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC slave 4 control register //IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC slave 4 reads the data register //IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG		0X36	//IIC host status register //IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		  0X37	//Interrupt/bypass set register //中断/旁路设置寄存器
#define MPU_INT_EN_REG			  0X38	//Interrupt enable register//中断使能寄存器
#define MPU_INT_STA_REG		  	0X3A	//Interrupt status register//中断状态寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//Acceleration value, X-axis high 8-bit register //加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//Acceleration value, X-axis low 8-bit register //加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//Acceleration value,Y axis high 8 bit registe //加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//Acceleration value,Y axis low 8-bit register //加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//Acceleration value, Z-axis high 8-bit register //加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//Acceleration value, Z-axis low 8-bit register //加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG		  0X41	//High temperature 8-bit register //温度值高八位寄存器
#define MPU_TEMP_OUTL_REG		  0X42	//Low temperature 8 bit register //温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//Gyroscope value, X-axis high 8-bit register//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//Gyroscope value, X-axis low 8-bit register //陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//Gyroscope value, Y axis high 8 bit register//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//Gyroscope value, Y axis low 8-bit register /陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//Gyroscope value, Z-axis high 8-bit register//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//Gyroscope value, Z-axis low 8-bit register//陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG		0X63	//IIC slave 0 data register //IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC slave 1 data register //IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC slave 2 data register //IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC slave 3 data register //IIC从机3数据寄存器


#define MPU6050_TASK_PRIO		  3 
#define MPU6050_STK_SIZE 		  256 

unsigned char MPU6050_Init(void); 								
unsigned char MPU6050_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
unsigned char MPU6050_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf); 
unsigned char MPU6050_Write_Byte(u8 reg,u8 data);				
unsigned char MPU6050_Read_Byte(u8 reg);						

unsigned char MPU6050_Set_Gyro_Fsr(u8 fsr);
unsigned char MPU6050_Set_Accel_Fsr(u8 fsr);
unsigned char MPU6050_Set_LPF(u16 lpf);
unsigned char MPU6050_Set_Rate(u16 rate);
unsigned char MPU6050_Set_Fifo(u8 sens);

extern	short gyro[3], accel[3];
extern  short Deviation_gyro[3],Original_gyro[3];
extern  short Deviation_accel[3],Original_accel[3]; 
extern int Flag_Mpu6050;  
extern int Deviation_Count;
unsigned char MPU6050_Get_Gyroscope(short *gyro);
unsigned char MPU6050_Get_Accelerometer(short *accel);

#endif





































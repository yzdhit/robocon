
#ifndef __SELECT_H
#define __SELECT_H

#define		SPI_EINT0		0		//如果要使能SPI读取外部中断的话设置为1
#define		EN_LCD_ARM		0 		//如果使用串行液晶的话设置为1，反之为0，如果不使用液晶，则都设为0
#define 	EN_BT			1		//使能蓝牙模块标志
#define 	EN_RADAR		0		//使能雷达模块标志
#define     EN_GYRO			1		//使能陀螺
#define 	EN_SPI			0		//使能SPI通信接口
#define   EN_UART1_PC_ARM  1  //使能博士接受中断，在设置此值为1时，串口1将用来使用兰博士的接受中断
#define     EN_UART1        1       //使能UART1
#define     EN_UART2        1       //使能UART2
#define     EN_UART3        0       //使能UART3
#define     EN_UART4        0       //使能UART4
#define     EN_UART5        0       //使能UART5
#define     EN_UART6        1       //使能UART6
#define 	BASE_TYPE		1		//机器人底盘类型，0：差速，1：全向轮
#endif



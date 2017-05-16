
/*******************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：MotorConfig.h	
最近修改日期：2012.1.30
版本：4.0
--------------------------------------------------------------------
模块描述：与电机控制相关的配置
函数列表：

--------------------------------------------------------------------
修改记录：
作者        时间            版本        说明
任伟        2010.4.26      	1.0         建立此文件
詹军成      2010.11.30      2.0         规范化此文件
任伟		2011.07.10		3.0			整合电机的控制宏，使该模块变得更加面向对象化，加入了电机配置宏函数
										CONFIG_MOTOR()，使电机的使用配置更加人性化
王岩		2012.01.30		4.0			重新配置IO
********************************************************************/
#ifndef __MOTOR_DEF_H
#define __MOTOR_DEF_H

#include "HITCRT_Types.h"
#include "FPGA_REGS.h"
#include "stm32f4xx.h"
			  
/*--------------定义电机方向控制引脚和正反转---------------------*/
//                             A                       B
#define M0_FORWARD()	GPIOB->ODR |= GPIO_Pin_12; GPIOB->ODR &= ~GPIO_Pin_10;
#define M0_REVERSE()	GPIOB->ODR |= GPIO_Pin_10; GPIOB->ODR &= ~GPIO_Pin_12;
#define M0_BREAK()		PWM_OUT_MOTOR(0,0)


#define M1_FORWARD()	GPIOF->ODR |= GPIO_Pin_14; GPIOB->ODR &= ~GPIO_Pin_11;
#define M1_REVERSE()	GPIOB->ODR |= GPIO_Pin_11; GPIOF->ODR &= ~GPIO_Pin_14;
#define M1_BREAK()		PWM_OUT_MOTOR(1,0)


#define M2_FORWARD()	GPIOB->ODR |= GPIO_Pin_14; GPIOG->ODR &= ~GPIO_Pin_8;
#define M2_REVERSE()	GPIOG->ODR |= GPIO_Pin_8; GPIOB->ODR &= ~GPIO_Pin_14;
#define M2_BREAK()		PWM_OUT_MOTOR(2,0)

#define M3_FORWARD()	GPIOB->ODR |= GPIO_Pin_13; GPIOB->ODR &= ~GPIO_Pin_15;
#define M3_REVERSE()	GPIOB->ODR |= GPIO_Pin_15; GPIOB->ODR &= ~GPIO_Pin_13;
#define M3_BREAK()		PWM_OUT_MOTOR(3,0)

#define M4_FORWARD()	GPIOC->ODR |= GPIO_Pin_8; GPIOC->ODR &= ~GPIO_Pin_9;
#define M4_REVERSE()	GPIOC->ODR |= GPIO_Pin_9; GPIOC->ODR &= ~GPIO_Pin_8;
#define M4_BREAK()		PWM_OUT_MOTOR(4,0)

#define M5_FORWARD()	GPIOG->ODR |= GPIO_Pin_7; GPIOA->ODR &= ~GPIO_Pin_8; 
#define M5_REVERSE()	GPIOA->ODR |= GPIO_Pin_8; GPIOG->ODR &= ~GPIO_Pin_7;
#define M5_BREAK()		PWM_OUT_MOTOR(5,0)

#define M6_FORWARD()	GPIOG->ODR |= GPIO_Pin_14; GPIOG->ODR &= ~GPIO_Pin_12;
#define M6_REVERSE()	GPIOG->ODR |= GPIO_Pin_12; GPIOG->ODR &= ~GPIO_Pin_14;
#define M6_BREAK()		PWM_OUT_MOTOR(6,0)
	
#define M7_FORWARD()	GPIOG->ODR |= GPIO_Pin_13; GPIOG->ODR &= ~GPIO_Pin_11;
#define M7_REVERSE()	GPIOG->ODR |= GPIO_Pin_11; GPIOG->ODR &= ~GPIO_Pin_13;
#define M7_BREAK()		PWM_OUT_MOTOR(7,0)

#define M8_FORWARD()	GPIOG->ODR |= GPIO_Pin_10; GPIOD->ODR &= ~GPIO_Pin_2;
#define M8_REVERSE()	GPIOD->ODR |= GPIO_Pin_2; GPIOG->ODR &= ~GPIO_Pin_10;
#define M8_BREAK()		PWM_OUT_MOTOR(8,0)

#define M9_FORWARD()	GPIOG->ODR |= GPIO_Pin_9; GPIOC->ODR &= ~GPIO_Pin_12;
#define M9_REVERSE()	GPIOC->ODR |= GPIO_Pin_12; GPIOG->ODR &= ~GPIO_Pin_9;
#define M9_BREAK()		PWM_OUT_MOTOR(9,0)

#define M10_FORWARD()	GPIOC->ODR |= GPIO_Pin_11; GPIOA->ODR &= ~GPIO_Pin_11;
#define M10_REVERSE()	GPIOA->ODR |= GPIO_Pin_11; GPIOC->ODR &= ~GPIO_Pin_11;
#define M10_BREAK()		PWM_OUT_MOTOR(10,0)

#define M11_FORWARD()	GPIOC->ODR |= GPIO_Pin_10; GPIOA->ODR &= ~GPIO_Pin_12;
#define	M11_REVERSE()	GPIOA->ODR |= GPIO_Pin_12; GPIOC->ODR &= ~GPIO_Pin_10;
#define M11_BREAK()		PWM_OUT_MOTOR(11,0)

#define PWM_MOTOR_INIT()	MOTOR_CHANNEL_SEL = 0x0fff;	
#define PWM_SERVO_INIT()	SERVO_CHANNEL_SEL = 0x00ff;	 

/*输出电机PWM，x为PWM通道数（0到11），y为PWM占空比*/
#define PWM_OUT_MOTOR(x,y)	PWM_MOTOR_FPGA(x) = y

/*输出舵机PWM，x为PWM通道数（0-11），y为PWM占空比*/
#define PWM_OUT_SERVO(x,y)	PWM_SERVO_FPGA(x) = y

#define ForwardMotor(_MC)			M##_MC##_FORWARD()
#define ReverseMotor(_MC)			M##_MC##_REVERSE()

#define ForwardMotorFIT(_MC)		M##_MC##_FORWARD()
#define ForwardMotorOPST(_MC)		M##_MC##_REVERSE()	
#define ReverseMotorFIT(_MC)		M##_MC##_REVERSE()
#define ReverseMotorOPST(_MC)		M##_MC##_FORWARD()

#define BreakMotor(_MC) 	PWM_MOTOR_FPGA(_MC) = 0

		

#endif

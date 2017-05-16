#ifndef  	__FPGA_REGS_H
#define 	__FPGA_REGS_H

#include "HITCRT_Types.h"
#include "FSMC.h"
//#define BANK1_SRAM4_ADDR    (UINT32*)0x6c000000

/*码盘偏移地址,0x00~0x1f*/
#define OFST_CODER				0x00
#define OFST_CODER_SEL      	0x1d 
#define OFST_CODER_RST			0x1e  
/*电机PWM地址,0x20~0x2c*/
#define OFST_PWM_MOTOR			0x20
#define OFST_PWM_MOTOR_SEL      0x2f 
/*舵机PWM地址,0x30~0x38*/
#define OFST_PWM_SERVO         	0x30
#define OFST_PWM_SERVO_SEL		0x3f 
/*FPGA初始化*/
#define     OFST_NRST_FPGA      0x40
#define     OFST_CE_FPGA        0x41
#define     OFST_TEST_WR        0x42
#define     OFST_TEST_RD        0x43
/*手柄地址,0x46~0x48*/
#define OFST_JOYSTICK			0x50
/*当手柄为红灯模式时ADDR_JS_STATE值为0x73，否则为0x41*/
#define ADDR_JS_STATE			0x53
/*键盘地址,0x2d*/               
#define OFST_PSKEY              0x54
/*LCD第一行第一列地址*/
#define	OFST_LCD				0x56
#define OFST_NRST_LCD   		0x57 
/*--------------GRYO-------------*/
#define OFST_GRYO				0x58
/*--------------FPGA初始化--------------*/
#define FPGA_NRST	(*(volatile USHORT16*)(BANK1_SRAM3_ADDR | (OFST_NRST_FPGA << 10)))
#define FPGA_CE	(*(volatile USHORT16*)(BANK1_SRAM3_ADDR | (OFST_CE_FPGA << 10)))
#define FPGA_WR	(*(volatile USHORT16*)(BANK1_SRAM3_ADDR | (OFST_TEST_WR << 10)))
#define FPGA_RD	(*(volatile USHORT16*)(BANK1_SRAM3_ADDR | (OFST_TEST_RD << 10)))
#define INIT_FPGA()  FPGA_NRST = 0x000;FPGA_NRST = 0xffff;while(FPGA_NRST != 0xffff){FPGA_NRST = 0xffff;}while(FPGA_CE != 0xffff){FPGA_CE = 0xffff;}

#define MAX_LEN_NUM		16		            //液晶显示数字时的最大长度（单位：字节）
/*--------------LCD12864--------------*/
#define LCD_FPGA_CTRL	(*(volatile USHORT16*)(BANK1_SRAM3_ADDR | (OFST_LCD << 10)))
#define LCD_FPGA_NRST	(*(volatile USHORT16*)(BANK1_SRAM3_ADDR | (OFST_NRST_FPGA << 10)))
/*-------------码盘-------------*/
#define ADDR_CODE_L(CH) (*(volatile USHORT16*)(BANK1_SRAM3_ADDR + ((OFST_CODER + CH) << 10)))
#define ADDR_CODE_H(CH) (*(volatile USHORT16*)(BANK1_SRAM3_ADDR + (((OFST_CODER + 16) + CH) << 10)))
#define READ_CODER(CH)	(ADDR_CODE_L(CH) + (ADDR_CODE_H(CH) << 16))
#define CODER_FPGA_SEL	(*((volatile USHORT16*)	(BANK1_SRAM3_ADDR + (OFST_CODER_SEL<< 10))))
#define CODER_FPGA_RST	(*((volatile USHORT16*)	(BANK1_SRAM3_ADDR + (OFST_CODER_RST<< 10))))
#define CODER_INIT()	CODER_FPGA_SEL = 0xffff;	CODER_FPGA_RST = 0x000;CODER_FPGA_RST = 0xffff
/*---------------PS2手柄------------------*/
#define JOYSTICK_RESERVED 	(*((volatile USHORT16*)	(BANK1_SRAM3_ADDR | ((OFST_JOYSTICK + 0)<< 10))))
#define JOYSTICK_LEFT 		(*((volatile USHORT16*)	(BANK1_SRAM3_ADDR | ((OFST_JOYSTICK + 1)<< 10))))
#define JOYSTICK_RIGTH 		(*((volatile USHORT16*)	(BANK1_SRAM3_ADDR | ((OFST_JOYSTICK + 2)<< 10))))
#define JOYSTICK_STATE 		(*((volatile USHORT16*)	(BANK1_SRAM3_ADDR | ((OFST_JOYSTICK + 3)<< 10))))
/*---------------PS2键盘------------------*/
#define PSKEY  (*((volatile USHORT16*)(BANK1_SRAM3_ADDR | ((OFST_PSKEY + 0)<< 10))))
/*--------------电机PWM--------------*/
#define PWM_MOTOR_FPGA(x) (*(volatile USHORT16*)(BANK1_SRAM3_ADDR + ((OFST_PWM_MOTOR + x) << 10)))
#define MOTOR_CHANNEL_SEL (*(volatile USHORT16*)(BANK1_SRAM3_ADDR + (OFST_PWM_MOTOR_SEL << 10)))

/*------------舵机PWM--------------*/
#define PWM_SERVO_FPGA(x) (*(volatile USHORT16*)(BANK1_SRAM3_ADDR + ((OFST_PWM_SERVO+ x) << 10)))
#define SERVO_CHANNEL_SEL (*(volatile USHORT16*)(BANK1_SRAM3_ADDR + ((OFST_PWM_SERVO_SEL + 16) << 10)))

/*-------------陀螺定位系统位姿--------------*/
#define GRYO_Q (*((volatile SSHORT16*)(BANK1_SRAM3_ADDR + ((OFST_GRYO + 0)<< 10))))
#define GRYO_X (*((volatile USHORT16*)(BANK1_SRAM3_ADDR + ((OFST_GRYO + 1)<< 10))))
#define GRYO_Y (*((volatile USHORT16*)(BANK1_SRAM3_ADDR + ((OFST_GRYO + 2)<< 10))))



//手柄按键编号定义
#define JS_UP(usValue)			(!(usValue & 0x1000))
#define JS_RIGHT(usValue)		(!(usValue & 0x2000))
#define JS_DOWN(usValue)		(!(usValue & 0x4000))
#define JS_LEFT(usValue)		(!(usValue & 0x8000))

#define JS_L2(usValue)			(!(usValue & 0x0001))
#define JS_R2(usValue)			(!(usValue & 0x0002))
#define JS_L1(usValue)			(!(usValue & 0x0004))
#define JS_R1(usValue)			(!(usValue & 0x0008))

#define JS_B1(usValue)			(!(usValue & 0x0010))
#define JS_B2(usValue)			(!(usValue & 0x0020))
#define JS_B3(usValue)			(!(usValue & 0x0040))
#define JS_B4(usValue)			(!(usValue & 0x0080))

#define JS_START(usValue)		(!(usValue & 0x0800)) 
#define JS_SELECT(usValue)		(!(usValue & 0x0100))

//ps键盘的按键定义
#if 0
#define KEY_N0(usKey)          (!(usKey& 0x0004))
#define KEY_N1(usKey)          (!(usKey& 0x0080))
#define KEY_N2(usKey)          (!(usKey& 0x0040))
#define KEY_N3(usKey)          (!(usKey& 0x0020))
#define KEY_N4(usKey)          (!(usKey& 0x0400))
#define KEY_N5(usKey)          (!(usKey& 0x0200))
#define KEY_N6(usKey)          (!(usKey& 0x0100))
#define KEY_N7(usKey)          (!(usKey& 0x2000))
#define KEY_N8(usKey)          (!(usKey& 0x1000))
#define KEY_N9(usKey)          (!(usKey& 0x0800))

#define KEY_F3(usKey)          (!(usKey& 0x8000))
#define KEY_F4(usKey)          (!(usKey& 0x4000))
#define KEY_F5(usKey)          (!(usKey& 0x0010))
#define KEY_F6(usKey)          (!(usKey& 0x0001))
#define KEY_F7(usKey)          (!(usKey& 0x0002))
#define KEY_F8(usKey)          (!(usKey& 0x0008))
#endif

#define KEY_N0(usKey)    (usKey== 0xff70)
#define KEY_N1(usKey)    (usKey== 0xff69)
#define KEY_N2(usKey)    (usKey== 0xff72)
#define KEY_N3(usKey)    (usKey== 0xff7a)
#define KEY_N4(usKey)    (usKey== 0xff6b)


#define KEY_N5(usKey)    (usKey == 0xff73)
#define KEY_N6(usKey)    (usKey == 0xff74)
#define KEY_N7(usKey)    (usKey == 0xff6c )
#define KEY_N8(usKey)    (usKey == 0xff75)
#define KEY_N9(usKey)    (usKey == 0xff7d)

#define KEY_F1(usKey)  (usKey == 0xff77)
#define KEY_F2(usKey)  (usKey == 0xff4A)
#define KEY_F3(usKey)  (usKey == 0xff7C)
#define KEY_F4(usKey)  (usKey == 0xff7B)
#define KEY_F5(usKey)  (usKey == 0xff79)
#define KEY_F6(usKey)  (usKey == 0xff5A)
#define KEY_F7(usKey)  (usKey == 0xff71)
#define KEY_F8(usKey)  (usKey == 0xff66)

#endif

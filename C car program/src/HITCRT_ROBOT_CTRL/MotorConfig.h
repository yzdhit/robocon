//#ifndef __MOTOR_CONFIG_H
//#define __MOTOR_CONFIG_H

#include "HITCRT_RobotTypes.h"
#include "HITCRT_API.h"
#include "math.h"

//定义电机的转速与码盘读数转换系数，考虑码盘线数、码盘倍频、电机减速比等
//fpKCodeToSpd = 60 * 10^6 / (n_c * l_c * i)    
//n_c:码盘倍频数；l_c:码盘线数；i:电机转速比；
//因为在速度反馈函数中求出的值为r/min，而定时器计数的量级是微秒，所以乘以60*10^6
#define K_V_C_M3257			8265.3f
#define K_V_C_M3863			2066.3f 
#define K_V_C_M2657			2022.63f
#define	K_V_C_MAXON_BK		5714.2857f
#define K_V_C_MAXON_BK_4F	1428.5714f	//在4倍频率采样下的MAXON转换系数
#define K_V_C_MAXON_RISE	7142.857f	//提升MAXON小电机
/*******************************************************************************************************************************
宏函数名称：CONFIG_MOTR()
功能：配置电机的名称，通道，匹配电机转向
Name:电机名称
_MC:电机通道
_DIR_SEL:电机方向配置，FIT：与原方向匹配，OPST：与原方向相反
函数名称：TurnName()
函数功能：根据输入占空比，控制电机的转向和转速
ssPwmDuty：输入的PWM占空比，正值代表电机正转，负值代表电机反转
函数名称：Occupy_MC()
函数功能：防止某通道电机被重复配置
********************************************************************************************************************************/
#define CONFIG_MOTOR(Name,MC,DIR_SEL)						\
void Turn##Name(SSHORT16 ssPwmDuty)							\
{															\
	if(ssPwmDuty >= 0)										\
	{														\
		ForwardMotor##DIR_SEL(MC);							\
	}														\
	else													\
	{														\
		ReverseMotor##DIR_SEL(MC);							\
	}														\
	if(ssPwmDuty >= 0)										\
	{														\
		PWM_MOTOR_FPGA(MC)= ssPwmDuty;						\
	}														\
	else													\
	{														\
		PWM_MOTOR_FPGA(MC)= -ssPwmDuty;						\
	}														\
}															\
void Occupy##MC(void)	{}	
	
#define PWM_DUTY_INIT	0
#define MAX_PWM_DUTY	2500
#define CODER_NUM_INIT	0
#define POT_INIT		0
#define COFF_POT_INIT	0
#define VELT_INIT		0
#define COFF_VELT_INIT	0
#define VELT_CODER_INIT		{0,0,0,{0,0,0,FIRST},FIRST}
#define	PID_INIT			{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}

#define SET_MOTOR_CTRL_PARA(NAME, MC, CC)		\
{												\
	PWM_DUTY_INIT,								\
	NAME##_MAX_PWM_DUTY,						\
	NAME##_CODER_NUM_INIT,						\
	NAME##_POT_INIT,							\
	NAME##_POT_INIT,							\
	NAME##_POT_INIT,							\
	NAME##_POT_SEN,								\
	NAME##_COFF_POT_INIT,						\
	NAME##_VELT_INIT,							\
	NAME##_VELT_INIT,							\
	NAME##_MAX_VELT,							\
	NAME##_COFF_VELT_INIT,						\
	VELT_CODER_INIT,							\
	NAME##_VELT_PID_PARA,						\
	NAME##_POT_PID_PARA,						\
	&Turn##NAME,								\
	(ST_ACTION_PATH*)NULL,                      \
	(ST_ACTION_PATH*)NULL,						\
	NAME##_MOTOR_NAME,							\
	MC,											\
	CC,											\
	MOTOR_BREAK,								\
	PATH_END									\
}
//	PATH_END									
/*******************************************************************************************************************************
宏函数名称：INSTANCE_MOTOR(Name,stMotor,MC,CC,DIR_SEL)
功能：实例化电机控制结构体，分配电机通道，匹配电机转向，初始化电机控制的各个参数
Name:电机名称
stMotor:电机结构体的名称
_MC:电机通道
_DIR_SEL:电机方向配置，FIT：与原方向匹配，OPST：与原方向相反
_CC:码盘通道
********************************************************************************************************************************/	
#define INSTANCE_MOTOR(NAME, stDev, MC, CC, DIR_SEL)	\
CONFIG_MOTOR(NAME, MC, DIR_SEL)							\
ST_MOTOR_CTRL stDev = SET_MOTOR_CTRL_PARA(NAME, MC, CC)

//#endif

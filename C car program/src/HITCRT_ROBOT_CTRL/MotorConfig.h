//#ifndef __MOTOR_CONFIG_H
//#define __MOTOR_CONFIG_H

#include "HITCRT_RobotTypes.h"
#include "HITCRT_API.h"
#include "math.h"

//��������ת�������̶���ת��ϵ���������������������̱�Ƶ��������ٱȵ�
//fpKCodeToSpd = 60 * 10^6 / (n_c * l_c * i)    
//n_c:���̱�Ƶ����l_c:����������i:���ת�ٱȣ�
//��Ϊ���ٶȷ��������������ֵΪr/min������ʱ��������������΢�룬���Գ���60*10^6
#define K_V_C_M3257			8265.3f
#define K_V_C_M3863			2066.3f 
#define K_V_C_M2657			2022.63f
#define	K_V_C_MAXON_BK		5714.2857f
#define K_V_C_MAXON_BK_4F	1428.5714f	//��4��Ƶ�ʲ����µ�MAXONת��ϵ��
#define K_V_C_MAXON_RISE	7142.857f	//����MAXONС���
/*******************************************************************************************************************************
�꺯�����ƣ�CONFIG_MOTR()
���ܣ����õ�������ƣ�ͨ����ƥ����ת��
Name:�������
_MC:���ͨ��
_DIR_SEL:����������ã�FIT����ԭ����ƥ�䣬OPST����ԭ�����෴
�������ƣ�TurnName()
�������ܣ���������ռ�ձȣ����Ƶ����ת���ת��
ssPwmDuty�������PWMռ�ձȣ���ֵ��������ת����ֵ��������ת
�������ƣ�Occupy_MC()
�������ܣ���ֹĳͨ��������ظ�����
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
�꺯�����ƣ�INSTANCE_MOTOR(Name,stMotor,MC,CC,DIR_SEL)
���ܣ�ʵ����������ƽṹ�壬������ͨ����ƥ����ת�򣬳�ʼ��������Ƶĸ�������
Name:�������
stMotor:����ṹ�������
_MC:���ͨ��
_DIR_SEL:����������ã�FIT����ԭ����ƥ�䣬OPST����ԭ�����෴
_CC:����ͨ��
********************************************************************************************************************************/	
#define INSTANCE_MOTOR(NAME, stDev, MC, CC, DIR_SEL)	\
CONFIG_MOTOR(NAME, MC, DIR_SEL)							\
ST_MOTOR_CTRL stDev = SET_MOTOR_CTRL_PARA(NAME, MC, CC)

//#endif

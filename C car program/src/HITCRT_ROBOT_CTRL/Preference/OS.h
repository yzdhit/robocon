/**********************************************************************************************************************************************************
��Ȩ������HITCRT(�����󾺼�������Э��)
�ļ�����OS.h
����޸����ڣ�2011.07.13
�汾��1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
ģ����������ͷ�ļ��а�������ϵͳ������ص�������Ϣ
�����б� 
DECLARE_OS_TASK(NAME)������ϵͳ���������꺯��
CREATE_OS_TASK(NAME)������ϵͳ�������꺯��
----------------------------------------------------------------------------------------------------------------------------------------------------------
�޶���¼��
����        ʱ��         �汾   ��ϵ��ʽ  					˵��							
��ΰ        2011.07.07   1.0    renwei_0991@126.com  		���ֽ�����ģ�飬����һЩ�꺯���������������񣬷�������Ľ����������պ�����ϵͳ����չ
**********************************************************************************************************************************************************/
#ifndef	__OS_H
#define __OS_H

/*����������Ķ�ջ����*/
#define	InitTask_SIZE				256						
#define BaseCtrlTask_SIZE			512
#define ActionMotorCtrlTask_SIZE    512
#define LcdDispTask_SIZE			512
#define ReadKeyTask_SIZE			512
#define NavTask_SIZE				1024
#define	DispatchTask_SIZE			1024
#define	CanTask_SIZE			256
#define DetectTask_SIZE         256

/*��������������ȼ�*/
#define	InitTask_PRIO				3						
#define BaseCtrlTask_PRIO			10
#define ActionMotorCtrlTask_PRIO        8
#define LcdDispTask_PRIO			11
#define ReadKeyTask_PRIO			14
#define NavTask_PRIO				7
#define DispatchTask_PRIO			6
#define CanTask_PRIO				4
#define DetectTask_PRIO	                     5

/****************************************************************************************************
�꺯�����ƣ�DECLARE_OS_TASK(NAME)
�������ܣ���������ϵͳ�����ջ�Լ�����ϵͳ������
��ڲ�����NAME����������
****************************************************************************************************/
#define	DECLARE_OS_TASK(NAME)		\
OS_STK	NAME##Stk[##NAME##_SIZE];	\
void	NAME (void *pdata)

/****************************************************************************************************
�꺯�����ƣ�DECLARE_OS_TASK(NAME)
�������ܣ������������ƴ�������ϵͳ���񣬸������Ʒ����ջ�Լ��������ȼ�
��ڲ�����NAME����������
****************************************************************************************************/
#define CREATE_OS_TASK(NAME)		\
OSTaskCreate(NAME, (void *)0, &##NAME##Stk[NAME##_SIZE-1], NAME##_PRIO)

//�Դ������������˵��
DECLARE_OS_TASK(InitTask);
DECLARE_OS_TASK(BaseCtrlTask);
DECLARE_OS_TASK(ActionMotorCtrlTask);
DECLARE_OS_TASK(LcdDispTask);
DECLARE_OS_TASK(ReadKeyTask);
DECLARE_OS_TASK(NavTask);
DECLARE_OS_TASK(DispatchTask);
DECLARE_OS_TASK(CanTask);
DECLARE_OS_TASK(DetectTask);


#endif

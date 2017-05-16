/**********************************************************************************************************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：OS.h
最近修改日期：2011.07.13
版本：1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
模块描述：该头文件中包含操作系统任务相关的配置信息
函数列表： 
DECLARE_OS_TASK(NAME)，操作系统任务申明宏函数
CREATE_OS_TASK(NAME)，操作系统任务建立宏函数
----------------------------------------------------------------------------------------------------------------------------------------------------------
修订记录：
作者        时间         版本   联系方式  					说明							
任伟        2011.07.07   1.0    renwei_0991@126.com  		划分建立此模块，采用一些宏函数来建立申明任务，方便任务的建立，方便日后整个系统的扩展
**********************************************************************************************************************************************************/
#ifndef	__OS_H
#define __OS_H

/*定义主任务的堆栈长度*/
#define	InitTask_SIZE				256						
#define BaseCtrlTask_SIZE			512
#define ActionMotorCtrlTask_SIZE    512
#define LcdDispTask_SIZE			512
#define ReadKeyTask_SIZE			512
#define NavTask_SIZE				1024
#define	DispatchTask_SIZE			1024
#define	CanTask_SIZE			256
#define DetectTask_SIZE         256

/*定义主任务的优先级*/
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
宏函数名称：DECLARE_OS_TASK(NAME)
函数功能：申明操作系统任务堆栈以及操作系统的任务
入口参数：NAME，任务名称
****************************************************************************************************/
#define	DECLARE_OS_TASK(NAME)		\
OS_STK	NAME##Stk[##NAME##_SIZE];	\
void	NAME (void *pdata)

/****************************************************************************************************
宏函数名称：DECLARE_OS_TASK(NAME)
函数功能：根据任务名称创建操作系统任务，根据名称分配堆栈以及任务优先级
入口参数：NAME，任务名称
****************************************************************************************************/
#define CREATE_OS_TASK(NAME)		\
OSTaskCreate(NAME, (void *)0, &##NAME##Stk[NAME##_SIZE-1], NAME##_PRIO)

//对创建的任务进行说明
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

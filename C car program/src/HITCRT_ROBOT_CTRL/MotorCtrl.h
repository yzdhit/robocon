#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H
#include "HITCRT_Types.h"
#include "HITCRT_RobotTypes.h"



#define AMQ 12//执行电机的数量

/****************************************************************************************************
函数名称:CopyFloatSlowly(FP32 * pfpDest, FP32 *pfpSrc, FP32 fpCopyStep)
函数功能: 缓慢拷贝，防止速度等变量的突变
输入参数: pfpDest 目标数据的指针
			   pfpSrc源数据的指针
			   每次拷贝的步长
输出参数:无
备注: 需要反复调用才能完全copy完成。否则得到的数据不对
****************************************************************************************************/
extern void CopyFloatSlowly(FP32 * pfpDest, FP32 *pfpSrc, const FP32 fpCopyStep);
extern void CopyIntSlowly(SINT32 * psiDest, SINT32 * psiSrc,  const UINT32 puiCopyStep);
extern void CopyIntSlowlyEx(SSHORT16* psiDest, SINT32 siSrc,  const UINT32 puiCopyStep);
extern void CopyIntSlowlyEx1(SINT32* psiDest, SINT32 siSrc,  const UINT32 puiCopyStep);


/****************************************************************************************************
函数名称:CalVelt()
函数功能: 根据码盘过线数及其对应的时间计算速度
输入参数: pstVeltCoder，计算速度的码盘结构体
			   cuCoderchan，使用的码盘通道
			   fpCoffC2V，实际速度与码盘过线速度的转换系数
输出参数: 反馈速度
备注: 该函数是根据T法计算码盘反馈速度
****************************************************************************************************/
extern FP32 CalVelt(ST_MOTOR_CTRL * pstRobotM);

/****************************************************************************************************
函数名称:VeltLoopCtrl()
函数功能: 对电机进行速度闭环控制
输入参数:  pstRobotM，要控制的机器人上的电机
备注: 
****************************************************************************************************/
extern void VeltLoopCtrl (ST_MOTOR_CTRL * pstRobotM);

/****************************************************************************************************
函数名称:PotLoopCtrl()
函数功能: 对电机进行位置闭环控制，位置反馈传感器为码盘
输入参数:  pstRobotM，要控制的机器人上的电机
备注: 
****************************************************************************************************/
extern void PotLoopCtrl (ST_MOTOR_CTRL *pstRobotM);

/****************************************************************************************************
函数名称:OpenLoopCtrl()
函数功能: 电机开环控制
输入参数:  pstRobotM，要控制的电机
备注: 
****************************************************************************************************/
extern void OpenLoopCtrl(ST_MOTOR_CTRL *pstRobotM);
/****************************************************************************************************
函数名称:AutoPotLoopCtrl(ST_MOTOR_CTRL *pstRobotM) 
函数功能: 电机位置自动控制，包含速度规划，位置闭环，结束开环控制，路径切换等功能
输入参数:  pstRobotM，要控制的机器人的电机
备注: 
****************************************************************************************************/
extern void AutoPotLoopCtrl(ST_MOTOR_CTRL *pstRobotM);

extern void CalPosPID(ST_PID * pstPid);

extern void PotDoubleLoopCtrl(ST_MOTOR_CTRL * pstMotor);

/****************************************************************************************************
函数名称: DiffAutoPotLoopCtrl(ST_MOTOR_CTRL * pstMotor)
函数功能: 电机位置自动控制，速度规划、位置闭环、结束开环、路径切换
输入参数:  pstMotor，要控制的机器人的电机
备注: 
****************************************************************************************************/

extern void DiffAutoPotLoopCtrl(ST_MOTOR_CTRL * pstMotor);


/****************************************************************************************************
函数名称:DiffAutoVeltDis(ST_MOTOR_CTRL *pstMotor)
函数功能: 适应差速底盘的电机速度分配，主要是按照角度分配，而不是按照脉冲分配
输入参数:  pstMotor，执行电机的指针
备注: 注意是从0-  AMQ-1
****************************************************************************************************/
extern void DiffAutoVeltDis(ST_MOTOR_CTRL *pstMotor);

extern void SetPathEndPosEx(ST_MOTOR_CTRL *pMotor,ST_ACTION_PATH * pPath,FP32 siEndPos);
extern void SetPathEndPosOnly(ST_MOTOR_CTRL *pMotor, ST_ACTION_PATH * pPath,SINT32 siEndPos);
extern void SetPathEndPosWithVEx2(ST_MOTOR_CTRL *pMotor,ST_ACTION_PATH * pPath,FP32 siEndPos);


#endif


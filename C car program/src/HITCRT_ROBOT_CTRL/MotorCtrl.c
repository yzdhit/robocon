/*****************************************************************************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：MotorCtrl.c
最近修改日期：2010.07.10
版本：1.0
-----------------------------------------------------------------------------------------------------------------------------
模块描述：电机控制模块
函数列表： 
1. FP32 CalVelt(ST_VELT_CODER * pstVeltCoder, UCHAR8 ucCoderChan, FP32 fpCoffC2V)		电机速度计算函数
2. void VeltLoopCtrl (ST_MOTOR_CTRL * pstRobotM)									电机速度闭环控制函数
3. void PotLoopCtrl (ST_MOTOR_CTRL *pstRobotM)										电机位置闭环控制函数
------------------------------------------------------------------------------------------------------------------------------
修改记录：
作者        时间            版本     说明
任伟        2010.07.10      1.0      	 建立此文件，控制方法原理与以前的方法一样，加强了函数的封装性
******************************************************************************************************************************/

#include "HITCRT_RobotTypes.h"
#include "HITCRT_API.h"
#include "HITCRT_Algorithm.h"
#include "math.h"
#include "PrivateFunction.h"
#include "cordic.h"
#include "stdlib.h"
#include "MotorCtrl.h"
#include "HITCRT_Coff.h"



/****************************************************************************************************
函数名称:CopyFloatSlowly(FP32 * pfpDest, FP32 *pfpSrc, FP32 fpCopyStep)
函数功能: 缓慢拷贝，防止速度等变量的突变
输入参数: pfpDest 目标数据的指针
			   pfpSrc源数据的指针
			   每次拷贝的步长
输出参数:无
备注: 需要反复调用才能完全copy完成。否则得到的数据不对
****************************************************************************************************/
void CopyFloatSlowly(FP32 * pfpDest, FP32 *pfpSrc, const FP32 fpCopyStep)
{
	if(fabs(*pfpDest - *pfpSrc)<=fpCopyStep)
	{
		 *pfpDest = *pfpSrc;
	}
	else if(*pfpDest > *pfpSrc)
	{
		*pfpDest -= fpCopyStep;
	}
	else
	{
		*pfpDest += fpCopyStep;
	}
}



void CopyIntSlowly(SINT32 * psiDest, SINT32 * psiSrc,  const UINT32 puiCopyStep)
{
	if(abs(*psiDest - *psiSrc) <= puiCopyStep)
	{
		*psiDest = *psiSrc;
	}
	else if(*psiDest > *psiSrc)
	{
		*psiDest -= puiCopyStep;
	}
	else
	{
		*psiDest += puiCopyStep;
	}
}




void CopyIntSlowlyEx(SSHORT16* psiDest, SINT32 siSrc,  const UINT32 puiCopyStep)
{
	if(abs(*psiDest - siSrc) <= puiCopyStep)
	{
		*psiDest = siSrc;
	}
	else if(*psiDest > siSrc)
	{
		*psiDest -= puiCopyStep;
	}
	else
	{
		*psiDest += puiCopyStep;
	}
}



void CopyIntSlowlyEx1(SINT32* psiDest, SINT32 siSrc,  const UINT32 puiCopyStep)
{
	if(abs(*psiDest - siSrc) <= puiCopyStep)
	{
		*psiDest = siSrc;
	}
	else if(*psiDest > siSrc)
	{
		*psiDest -= puiCopyStep;
	}
	else
	{
		*psiDest += puiCopyStep;
	}
}

/****************************************************************************************************
函数名称:CalVelt()
函数功能: 根据码盘过线数及其对应的时间计算速度
输入参数: pstVeltCoder，计算速度的码盘结构体
			   cuCoderchan，使用的码盘通道
			   fpCoffC2V，实际速度与码盘过线速度的转换系数
输出参数: 反馈速度
备注: 该函数是根据T法计算码盘反馈速度
****************************************************************************************************/
FP32 CalVelt(ST_MOTOR_CTRL * pstRobotM)
{
	if (pstRobotM->stVeltCoder.emRunState == FIRST)
	{	
		pstRobotM->stVeltCoder.siCurCodeNum = READ_CODER(pstRobotM->ucCoderChan);
		pstRobotM->stVeltCoder.siPreCodeNum = pstRobotM->stVeltCoder.siCurCodeNum;
		pstRobotM->stVeltCoder.stIntlTime.uiCurTime = ((UINT32)TIM3->CNT << 16) | TIM2->CNT;//READ_TIMER_FPGA();
		pstRobotM->stVeltCoder.stIntlTime.uiPreTime = pstRobotM->stVeltCoder.stIntlTime.uiCurTime;
		pstRobotM->stVeltCoder.stIntlTime.uiIntlTime = 0;
		pstRobotM->stVeltCoder.emRunState = RUN;
		
		return 0;
	}
	else
	{
		pstRobotM->stVeltCoder.siCurCodeNum = READ_CODER(pstRobotM->ucCoderChan);		//读取当前码盘读数
		pstRobotM->stVeltCoder.siDetCodeNum = pstRobotM->stVeltCoder.siCurCodeNum - pstRobotM->stVeltCoder.siPreCodeNum;	//计算码盘走线数目
		pstRobotM->stVeltCoder.siPreCodeNum = pstRobotM->stVeltCoder.siCurCodeNum;		//保留本次码盘读数
		
		pstRobotM->stVeltCoder.stIntlTime.uiCurTime = ((UINT32)TIM3->CNT << 16) | TIM2->CNT;		//读取当前时间
		pstRobotM->stVeltCoder.stIntlTime.uiIntlTime = pstRobotM->stVeltCoder.stIntlTime.uiCurTime - pstRobotM->stVeltCoder.stIntlTime.uiPreTime;	//计算时间间隔
		pstRobotM->stVeltCoder.stIntlTime.uiPreTime = pstRobotM->stVeltCoder.stIntlTime.uiCurTime;	//保留本次时间

		return (pstRobotM->fpCoffVelt * pstRobotM->stVeltCoder.siDetCodeNum / pstRobotM->stVeltCoder.stIntlTime.uiIntlTime);
	}
}

/****************************************************************************************************
函数名称:VeltLoopCtrl()
函数功能: 对电机进行速度闭环控制
输入参数:  pstRobotM，要控制的机器人上的电机
备注: 
****************************************************************************************************/
void VeltLoopCtrl (ST_MOTOR_CTRL * pstRobotM)
{
	pstRobotM->stVeltPid.fpE = pstRobotM->fpVeltDes - pstRobotM->fpVeltFB;		//计算偏差
	pstRobotM->stVeltPid.fpE = ClipFloat(pstRobotM->stVeltPid.fpE,-pstRobotM->stVeltPid.fpEthreshold, pstRobotM->stVeltPid.fpEthreshold);
	CalPIDIS (&pstRobotM->stVeltPid);		//进行积分分离式PID运算

	pstRobotM->ssPwmDuty = Clip (Round (pstRobotM->stVeltPid.fpU), -pstRobotM->usMaxPwmDuty, pstRobotM->usMaxPwmDuty);	//限制PWM

	pstRobotM->TurnMotor (pstRobotM->ssPwmDuty);		//给电机控制信号，包括占空比和方向
}

/****************************************************************************************************
函数名称:PotLoopCtrl()  
函数功能: 对电机进行位置闭环控制，位置反馈传感器为码盘
输入参数:  pstRobotM，要控制的机器人的电机
备注: 
****************************************************************************************************/
void PotLoopCtrl (ST_MOTOR_CTRL *pstRobotM)
{
	pstRobotM->fpPotFB = pstRobotM->fpCoffPot * READ_CODER(pstRobotM->ucCoderChan);		//计算反馈位置
	
	pstRobotM->stPotPid.fpE = pstRobotM->fpPotDes - pstRobotM->fpPotFB;		//计算偏差

	CalPIDWTCOL(&pstRobotM->stPotPid);			//计算PID

	pstRobotM->ssPwmDuty = Clip (Round (pstRobotM->stPotPid.fpU), -pstRobotM->usMaxPwmDuty, pstRobotM->usMaxPwmDuty);	//限制PWM

	pstRobotM->TurnMotor (pstRobotM->ssPwmDuty);		//给电机控制信号，包括占空比和方向
}

/****************************************************************************************************
函数名称:OpenLoopCtrl()
函数功能: 电机开环控制
输入参数:  pstRobotM，要控制的电机
备注: 
****************************************************************************************************/
void OpenLoopCtrl(ST_MOTOR_CTRL *pstRobotM)
{
	pstRobotM->TurnMotor(pstRobotM->ssPwmDuty);
}




void AutoVeltDis(ST_MOTOR_CTRL *pstRobotM)
{	   
	static SSHORT16 s_ssPathMaxV[AMQ];		//路径最大速度
	static SINT32 s_siAcUpLenth[AMQ];	//加速向量里程
	static SINT32 s_siAcDownLenth[AMQ];	//减速向量里程
	static SINT32 s_siLenth[AMQ];//总的路程
	SINT32 siAcLenthTmp, siCurCode;
	static FP32 s_fpAccUpTmp[AMQ], s_fpAccDownTmp[AMQ];
	/*更新路径*/
	if (pstRobotM->pstCurPath != pstRobotM->pstPrePath)                                       
	{	
		if(pstRobotM->pstCurPath->ssStartV < pstRobotM->pstCurPath->ssMaxV)	//判定是否需要有加速路段
		{	
			s_fpAccUpTmp[pstRobotM->ucMotorChan] = pstRobotM->pstCurPath->fpAccUp * 1.0;//单位转换fpAccUp 
			s_siAcUpLenth[pstRobotM->ucMotorChan] = (SQUARE(pstRobotM->pstCurPath->ssMaxV) - SQUARE(pstRobotM->pstCurPath->ssStartV)) / s_fpAccUpTmp[pstRobotM->ucMotorChan] / 2.0;	//理论加速距离
		}
		else
		{
			s_siAcUpLenth[pstRobotM->ucMotorChan] = 0;
		}
			
		if(pstRobotM->pstCurPath->ssEndV < pstRobotM->pstCurPath->ssMaxV)	//判定是否需要有减速路段
		{
			s_fpAccDownTmp[pstRobotM->ucMotorChan] = pstRobotM->pstCurPath->fpAccDown * 1.0;//a
			s_siAcDownLenth[pstRobotM->ucMotorChan] = (SQUARE(pstRobotM->pstCurPath->ssMaxV) - SQUARE(pstRobotM->pstCurPath->ssEndV)) / s_fpAccDownTmp[pstRobotM->ucMotorChan] / 2.0;	//理论减速距离
		}
		else
		{
			s_siAcDownLenth[pstRobotM->ucMotorChan] = 0;
		}
		
		siAcLenthTmp = s_siAcUpLenth[pstRobotM->ucMotorChan] + s_siAcDownLenth[pstRobotM->ucMotorChan];	//计算总加减速距离
		s_siLenth[pstRobotM->ucMotorChan] = abs(pstRobotM->pstCurPath->siEndCode - pstRobotM->pstCurPath->siStartCode);
			
		if(siAcLenthTmp > s_siLenth[pstRobotM->ucMotorChan])	//加减速距离超过总行程，重新计算最高速度
		{
	//		StartTimer(&stTimer);
			s_ssPathMaxV[pstRobotM->ucMotorChan] = sqrt(2 * s_siLenth[pstRobotM->ucMotorChan] * s_fpAccUpTmp[pstRobotM->ucMotorChan] * s_fpAccDownTmp[pstRobotM->ucMotorChan] / (s_fpAccUpTmp[pstRobotM->ucMotorChan] + s_fpAccDownTmp[pstRobotM->ucMotorChan]) 
									+ SQUARE(pstRobotM->pstCurPath->ssStartV) + SQUARE(pstRobotM->pstCurPath->ssEndV));
	//		EndTimer(&stTimer);
			s_siAcUpLenth[pstRobotM->ucMotorChan] = (SQUARE(s_ssPathMaxV[pstRobotM->ucMotorChan]) - SQUARE(pstRobotM->pstCurPath->ssStartV)) / s_fpAccUpTmp[pstRobotM->ucMotorChan] / 2.0;
			s_siAcDownLenth[pstRobotM->ucMotorChan] = s_siLenth[pstRobotM->ucMotorChan] - s_siAcUpLenth[pstRobotM->ucMotorChan];
		}
		else
		{
			s_ssPathMaxV[pstRobotM->ucMotorChan] = pstRobotM->pstCurPath->ssMaxV;
		}
	}
	
	siCurCode = READ_CODER(pstRobotM->ucCoderChan);//读取当前位置码盘值
	
	if(pstRobotM->pstCurPath->scOver & (1 << 0))	//如果码盘计数增加运行
	{
		if(siCurCode - pstRobotM->pstCurPath->siStartCode < s_siAcUpLenth[pstRobotM->ucMotorChan])	//向量里程处于加速里程内
		{
			if(siCurCode >= pstRobotM->pstCurPath->siStartCode > 0)
			{
				pstRobotM->fpVeltDes = sqrt(2.0 * s_fpAccUpTmp[pstRobotM->ucMotorChan] * (siCurCode - pstRobotM->pstCurPath->siStartCode) + SQUARE(pstRobotM->pstCurPath->ssStartV));
			}
			else
			{
				pstRobotM->fpVeltDes = pstRobotM->pstCurPath->ssStartV;
			}
		}
		else if(siCurCode - pstRobotM->pstCurPath->siStartCode >= s_siAcUpLenth[pstRobotM->ucMotorChan] && pstRobotM->pstCurPath->siEndCode - siCurCode >= s_siAcDownLenth[pstRobotM->ucMotorChan])	//向量里程处于匀速里程内
		{
			pstRobotM->fpVeltDes = s_ssPathMaxV[pstRobotM->ucMotorChan];
		}
		else if(pstRobotM->pstCurPath->siEndCode - siCurCode < s_siAcDownLenth[pstRobotM->ucMotorChan])
		{
			if(pstRobotM->pstCurPath->siEndCode - siCurCode >= 0)
			{
				pstRobotM->fpVeltDes = sqrt(2.0 * s_fpAccDownTmp[pstRobotM->ucMotorChan] * (pstRobotM->pstCurPath->siEndCode - siCurCode) + SQUARE(pstRobotM->pstCurPath->ssEndV));
			}
			else
			{
				pstRobotM->fpVeltDes = pstRobotM->pstCurPath->ssEndV;
			}	
		}
	}
	else//如果码盘计数减少运行
	{
		if(siCurCode - pstRobotM->pstCurPath->siStartCode > -s_siAcUpLenth[pstRobotM->ucMotorChan])	//向量里程处于加速里程内
		{
			if(siCurCode <= pstRobotM->pstCurPath->siStartCode > 0)
			{
				pstRobotM->fpVeltDes = -sqrt(2.0 * s_fpAccUpTmp[pstRobotM->ucMotorChan] * (pstRobotM->pstCurPath->siStartCode - siCurCode) + SQUARE(pstRobotM->pstCurPath->ssStartV));
			}
			else
			{
				pstRobotM->fpVeltDes = -pstRobotM->pstCurPath->ssStartV;
			}
		}
		else if(siCurCode - pstRobotM->pstCurPath->siStartCode <= -s_siAcUpLenth[pstRobotM->ucMotorChan] && pstRobotM->pstCurPath->siEndCode - siCurCode <= -s_siAcDownLenth[pstRobotM->ucMotorChan])	//向量里程处于匀速里程内
		{
			pstRobotM->fpVeltDes = -s_ssPathMaxV[pstRobotM->ucMotorChan];
		}
		else if(pstRobotM->pstCurPath->siEndCode - siCurCode >= -s_siAcDownLenth[pstRobotM->ucMotorChan])
		{
			if(pstRobotM->pstCurPath->siEndCode <= siCurCode)
			{
				pstRobotM->fpVeltDes = -sqrt(2.0 * s_fpAccDownTmp[pstRobotM->ucMotorChan] * (siCurCode - pstRobotM->pstCurPath->siEndCode) + SQUARE(pstRobotM->pstCurPath->ssEndV));
			}
			else
			{
				pstRobotM->fpVeltDes = -pstRobotM->pstCurPath->ssEndV;
			}
			
		}
	}
}
//#endif
/****************************************************************************************************
函数名称:AutoPotLoopCtrl(ST_MOTOR_CTRL *pstRobotM) 
函数功能: 电机位置自动控制，包含速度规划，位置闭环，结束开环控制，路径切换等功能
输入参数:  pstRobotM，要控制的机器人的电机
备注: 
****************************************************************************************************/
void AutoPotLoopCtrl(ST_MOTOR_CTRL *pstRobotM)
{
	if(pstRobotM->emPathState == PATH_RUN)//路径当中
	{
		AutoVeltDis(pstRobotM);//自动分配速度
		pstRobotM->fpVeltDes = Clip(pstRobotM->fpVeltDes, -pstRobotM->fpMaxVelt, pstRobotM->fpMaxVelt);//对速度进行限制
		VeltLoopCtrl(pstRobotM);//速度闭环控制  
		//路径结束判断
		if(pstRobotM->pstCurPath->scOver & (1 << 4))//判断是否结束时位置闭环,提前判断
		{
			if ((pstRobotM->pstCurPath->scOver & (1 << 0)) && pstRobotM->stVeltCoder.siCurCodeNum >= pstRobotM->pstCurPath->siEndCode - pstRobotM->fpPotSen)   //当前码盘值大于终点码盘值时更新路段
			{
				pstRobotM->emPathState = PATH_END;
			}
			else if((!(pstRobotM->pstCurPath->scOver & (1 << 0))) && pstRobotM->stVeltCoder.siCurCodeNum <= pstRobotM->pstCurPath->siEndCode + pstRobotM->fpPotSen  )//当前码盘值小于终点码盘值时更新路段
			{
				pstRobotM->emPathState = PATH_END;
			}
		}
		else//结束时不需要位置闭环
		{
			if ((pstRobotM->pstCurPath->scOver & (1 << 0)) && pstRobotM->stVeltCoder.siCurCodeNum >= pstRobotM->pstCurPath->siEndCode)   //当前码盘值大于终点码盘值时更新路段
			{
				pstRobotM->emPathState = PATH_END;
			}
			else if((!(pstRobotM->pstCurPath->scOver & (1 << 0))) && pstRobotM->stVeltCoder.siCurCodeNum <= pstRobotM->pstCurPath->siEndCode)//当前码盘值小于终点码盘值时更新路段
			{
				pstRobotM->emPathState = PATH_END;
			}
		}
	}
	else
	{
		if(pstRobotM->pstCurPath->scOver & (1 << 4))//判断是否结束时位置闭环
		{
			pstRobotM->fpPotDes = pstRobotM->fpCoffPot * pstRobotM->pstCurPath->siEndCode;
			PotLoopCtrl(pstRobotM);
		}
		else if(pstRobotM->pstCurPath->scOver & (1 << 5))//判断是否结束时PWMDuty=0
		{
			pstRobotM->ssPwmDuty = 0;
			pstRobotM->TurnMotor(pstRobotM->ssPwmDuty);
		}
		else if(pstRobotM->pstCurPath->scOver & (1 << 6))//判断是否结束时PWMDuty=EndPWMDuty
		{
			pstRobotM->ssPwmDuty = pstRobotM->pstCurPath->EndPwmDuty;
			pstRobotM->TurnMotor(pstRobotM->ssPwmDuty);
		}
	}
}
//#endif


void CalPosPID(ST_PID * pstPid)
  {


	UCHAR8 ucK = 0;
	if(fabs(pstPid->fpE)<pstPid->fpEDead)
	{
		pstPid->fpE = 0;
	}
	
	
	if(fabs(pstPid->fpE)>pstPid->fpELimit)
	{
			ucK = 0;
	}
	else
	{
			ucK = 1;
	}

	pstPid->fpU += pstPid->fpP*(pstPid->fpE - pstPid->fpPreE) +ucK* pstPid->fpI*pstPid->fpE + pstPid->fpD*(pstPid->fpE - 2*pstPid->fpPreE + pstPid->fpPrePreE);
	
		pstPid->fpU = ClipFloat(pstPid->fpU,-pstPid->fpULimit,pstPid->fpULimit);
	

	pstPid->fpPrePreE = pstPid->fpPreE;
	pstPid->fpPreE = pstPid->fpE;

}

void PotDoubleLoopCtrl(ST_MOTOR_CTRL * pstMotor)
{
	//位置环
	pstMotor->fpPotFB = pstMotor->fpCoffPot * READ_CODER(pstMotor->ucCoderChan);
	pstMotor->stPotPid.fpE = pstMotor->fpPotDes - pstMotor->fpPotFB;		//计算偏差
	pstMotor->stPotPid.fpE = ClipFloat(pstMotor->stPotPid.fpE, -pstMotor->stPotPid.fpEthreshold,pstMotor->stPotPid.fpEthreshold);
	CalPosPID(&pstMotor->stPotPid);

	//速度环
	pstMotor->fpVeltDes = pstMotor->stPotPid.fpU;
	pstMotor->fpVeltFB = CalVelt(pstMotor);
	VeltLoopCtrl(pstMotor);	
}


/****************************************************************************************************
函数名称:DiffAutoVeltDis(ST_MOTOR_CTRL *pstMotor)
函数功能: 适应差速底盘的电机速度分配，主要是按照角度分配，而不是按照脉冲分配
输入参数:  pstMotor，执行电机的指针
备注: 注意是从0-  AMQ-1
****************************************************************************************************/
void DiffAutoVeltDis(ST_MOTOR_CTRL *pstMotor)
{
	static FP32 s_fpPathMaxV[AMQ];  //实际为角速度，单位为r/min
	static FP32 s_fpAccUpLen[AMQ];  //实际为角度，单位为度,加速度为rad/s^2
	static FP32 s_fpAccDownLen[AMQ];
	static FP32 s_fpTotalLen[AMQ];
	SINT32 siCoderNum = 0;

	if(pstMotor->pstCurPath != pstMotor->pstPrePath)
	{
		pstMotor->pstPrePath = pstMotor->pstCurPath;
		if(pstMotor->pstCurPath->ssStartV < pstMotor->pstCurPath->ssMaxV)
		{
			s_fpAccUpLen[pstMotor->ucMotorChan] = ((SQUARE(pstMotor->pstCurPath->ssMaxV*PI_DIV_30) - SQUARE(pstMotor->pstCurPath->ssStartV*PI_DIV_30) )/(2.0*pstMotor->pstCurPath->fpAccUp))*DEG;
		}
		else
		{
			s_fpAccUpLen[pstMotor->ucMotorChan] = 0;
		}


		if(pstMotor->pstCurPath->ssEndV < pstMotor->pstCurPath->ssMaxV)
		{
			s_fpAccDownLen[pstMotor->ucMotorChan] =  ((SQUARE(pstMotor->pstCurPath->ssMaxV*PI_DIV_30) - SQUARE(pstMotor->pstCurPath->ssEndV*PI_DIV_30) )/(2.0*pstMotor->pstCurPath->fpAccDown))*DEG;
		}
		else
		{
			s_fpAccDownLen[pstMotor->ucMotorChan] = 0;
		}

		s_fpTotalLen[pstMotor->ucMotorChan] = abs(pstMotor->pstCurPath->siEndCode - pstMotor->pstCurPath->siStartCode);

		if((s_fpAccUpLen[pstMotor->ucMotorChan]  + s_fpAccDownLen[pstMotor->ucMotorChan] ) > s_fpTotalLen[pstMotor->ucMotorChan])
		{
			s_fpPathMaxV[pstMotor->ucMotorChan] = sqrt((pstMotor->pstCurPath->fpAccDown*SQUARE(pstMotor->pstCurPath->ssStartV*PI_DIV_30)
				                                                + pstMotor->pstCurPath->fpAccUp*SQUARE(pstMotor->pstCurPath->ssEndV*PI_DIV_30) 
				                                                +2*pstMotor->pstCurPath->fpAccUp*pstMotor->pstCurPath->fpAccDown*s_fpTotalLen[pstMotor->ucMotorChan]/DEG)/(pstMotor->pstCurPath->fpAccUp + pstMotor->pstCurPath->fpAccDown))/PI_DIV_30;
			s_fpAccUpLen[pstMotor->ucMotorChan] = ((SQUARE(s_fpPathMaxV[pstMotor->ucMotorChan] *PI_DIV_30)  - SQUARE(pstMotor->pstCurPath->ssStartV*PI_DIV_30) )/(2.0*pstMotor->pstCurPath->fpAccUp))*DEG;
			s_fpAccDownLen[pstMotor->ucMotorChan] = ((SQUARE(s_fpPathMaxV[pstMotor->ucMotorChan] *PI_DIV_30)  - SQUARE(pstMotor->pstCurPath->ssEndV*PI_DIV_30) )/(2.0*pstMotor->pstCurPath->fpAccDown))*DEG;
			
		}
		else
		{
			s_fpPathMaxV[pstMotor->ucMotorChan] = pstMotor->pstCurPath->ssMaxV;
		}
	}
	//打印加速路径和减速路径长
	/*
	PutCur(3,0);
	PrintFloat(s_fpAccUpLen[0],2,2);
	PrintChar("  ");
	PrintFloat(s_fpAccDownLen[0],2,2);*/
	

	siCoderNum = READ_CODER(pstMotor->ucCoderChan);
	pstMotor->fpPotFB = pstMotor->fpCoffPot *siCoderNum; //角度的量纲
//用标志量控制计算方向不太合理，应该根据结束时和起始时
	if(pstMotor->pstCurPath->scOver & (1<<0))
	{
		if((pstMotor->fpPotFB - pstMotor->pstCurPath->siStartCode) < s_fpAccUpLen[pstMotor->ucMotorChan])
		{
			if(pstMotor->fpPotFB > pstMotor->pstCurPath->siStartCode)
			{
			      pstMotor->fpVeltDes = sqrt(SQUARE(pstMotor->pstCurPath->ssStartV*PI_DIV_30) + 2.0*pstMotor->pstCurPath->fpAccUp*(pstMotor->fpPotFB - pstMotor->pstCurPath->siStartCode)/DEG)/PI_DIV_30;
			}
			else
			{
				 pstMotor->fpVeltDes = pstMotor->pstCurPath->ssStartV;
			}
		}
		else if((pstMotor->pstCurPath->siEndCode - pstMotor->fpPotFB) < s_fpAccDownLen[pstMotor->ucMotorChan])
		{
			if(pstMotor->pstCurPath->siEndCode > pstMotor->fpPotFB)
			{
				 pstMotor->fpVeltDes = sqrt(SQUARE(pstMotor->pstCurPath->ssEndV*PI_DIV_30) + 2.0*pstMotor->pstCurPath->fpAccDown*(pstMotor->pstCurPath->siEndCode - pstMotor->fpPotFB)/DEG)/PI_DIV_30;

			}
			else
			{
				pstMotor->fpVeltDes = pstMotor->pstCurPath->ssEndV;
			}
		}
		else
		{
			pstMotor->fpVeltDes = s_fpPathMaxV[pstMotor->ucMotorChan];
		}
	}
      else
      {
	  	if((pstMotor->fpPotFB - pstMotor->pstCurPath->siStartCode) > -s_fpAccUpLen[pstMotor->ucMotorChan])
		{
			if(pstMotor->fpPotFB < pstMotor->pstCurPath->siStartCode)
			{
			      pstMotor->fpVeltDes = -sqrt(SQUARE(pstMotor->pstCurPath->ssStartV*PI_DIV_30) + 2.0*pstMotor->pstCurPath->fpAccUp*(pstMotor->pstCurPath->siStartCode - pstMotor->fpPotFB)/DEG)/PI_DIV_30;
			}
			else
			{
				 pstMotor->fpVeltDes = -pstMotor->pstCurPath->ssStartV;
			}
		}
		else if((pstMotor->pstCurPath->siEndCode - pstMotor->fpPotFB) > -s_fpAccDownLen[pstMotor->ucMotorChan])
		{
			if(pstMotor->pstCurPath->siEndCode < pstMotor->fpPotFB)
			{
				 pstMotor->fpVeltDes = -sqrt(SQUARE(pstMotor->pstCurPath->ssEndV*PI_DIV_30) + 2.0*pstMotor->pstCurPath->fpAccDown*( pstMotor->fpPotFB - pstMotor->pstCurPath->siEndCode)/DEG)/PI_DIV_30;
			}
			else
			{
				pstMotor->fpVeltDes = -pstMotor->pstCurPath->ssEndV;
			}
		}
		else
		{
			pstMotor->fpVeltDes = -s_fpPathMaxV[pstMotor->ucMotorChan];
		}
      }
	
	
}


/****************************************************************************************************
函数名称: DiffAutoPotLoopCtrl(ST_MOTOR_CTRL * pstMotor)
函数功能: 电机位置自动控制，速度规划、位置闭环、结束开环、路径切换
输入参数:  pstMotor，要控制的机器人的电机
备注: 
****************************************************************************************************/

void DiffAutoPotLoopCtrl(ST_MOTOR_CTRL * pstMotor)
{
	SINT32 siCoderNum;
	if(pstMotor->emPathState == PATH_RUN)
	{
		DiffAutoVeltDis(pstMotor);
		pstMotor->fpVeltFB = CalVelt(pstMotor);
		pstMotor->fpVeltDes = ClipFloat(pstMotor->fpVeltDes,-pstMotor->fpMaxVelt,pstMotor->fpMaxVelt);
		VeltLoopCtrl(pstMotor);

		siCoderNum = READ_CODER(pstMotor->ucCoderChan);
		pstMotor->fpPotFB = pstMotor->fpCoffPot *siCoderNum; //角度的量纲
		
		//路径判断结束
		if(pstMotor->pstCurPath->scOver &(1<<4))
		{
			if((pstMotor->pstCurPath->scOver & (1<<0)) && pstMotor->fpPotFB >= (pstMotor->pstCurPath->siEndCode - pstMotor->fpPotSen))
			{
				pstMotor->emPathState = PATH_END;
			}


			if((!(pstMotor->pstCurPath->scOver & (1<<0))) && pstMotor->fpPotFB <= (pstMotor->pstCurPath->siEndCode + pstMotor->fpPotSen))
			{
				pstMotor->emPathState = PATH_END;
			}
		}
		else
		{
			if((pstMotor->pstCurPath->scOver & (1<<0)) && pstMotor->fpPotFB >=pstMotor->pstCurPath->siEndCode)
			{
				pstMotor->emPathState = PATH_END;
			}


			if((!(pstMotor->pstCurPath->scOver & (1<<0))) && pstMotor->fpPotFB <= pstMotor->pstCurPath->siEndCode)
			{
				pstMotor->emPathState = PATH_END;
			}
		}
	}
	else
	{
		if(pstMotor->pstCurPath->scOver & (1<<4))
		{
			pstMotor->fpPotDes = pstMotor->pstCurPath->siEndCode;
			PotDoubleLoopCtrl(pstMotor);
		}
		else if(pstMotor->pstCurPath->scOver & (1<<5))
		{
			pstMotor->ssPwmDuty =0;
			pstMotor->TurnMotor(pstMotor->ssPwmDuty);
		}
		else if(pstMotor->pstCurPath->scOver &(1<<6))
		{
			pstMotor->ssPwmDuty = pstMotor->pstCurPath->EndPwmDuty;
			pstMotor->TurnMotor(pstMotor->ssPwmDuty);
		}
		else if(pstMotor->pstCurPath->scOver &(1<<7))
		{
			//路径结束时调用零速PID
			pstMotor->fpVeltFB = CalVelt(pstMotor);
		       pstMotor->fpVeltDes =0;
		       VeltLoopCtrl(pstMotor);
		}
	}
}


//电机的位置值，都为角度值
//用于设置当前电机路径的结束时的位置,使用该函数时，
//请确认结束位置和起始位置不同，否则后果可能很严重
void SetPathEndPosOnly(ST_MOTOR_CTRL *pMotor, ST_ACTION_PATH * pPath,SINT32 siEndPos)
{
	pPath->siEndCode = siEndPos;
	
	//防止起始点和结束点重合
	if(abs(pPath->siEndCode - pPath->siStartCode)<1)
	{
		pPath->siEndCode = pPath->siStartCode + 1;
	}
	
	if(pPath->siEndCode >= pPath->siStartCode)//设为scOver为增量运行
	{
		pPath->scOver |= 0x01;
	}
	else
	{
		pPath->scOver &= 0xfe;
	}
	pMotor->pstPrePath = NULL;
	pMotor->pstCurPath = pPath;
	pMotor->emPathState = PATH_RUN;
}


//设置电机路径的结束值的同时，将电机当前位置值设为起始值
//请确认结束位置和起始位置不同，否则后果可能很严重
void SetPathEndPosEx(ST_MOTOR_CTRL *pMotor,ST_ACTION_PATH * pPath,FP32 siEndPos)
{
       pPath->siStartCode = pMotor->fpCoffPot * READ_CODER(pMotor->ucCoderChan);
	pPath->siEndCode = siEndPos;
	//防止起始点和结束点重合
	if(abs(pPath->siEndCode - pPath->siStartCode)<1)
	{
		pPath->siEndCode = pPath->siStartCode + 1;
	}
	
	if(pPath->siEndCode >= pPath->siStartCode)//设为scOver为增量运行
	{
		pPath->scOver |= 0x01;
	}
	else
	{
		pPath->scOver &= 0xfe;
	}
	pMotor->pstPrePath = NULL;
	pMotor->pstCurPath = pPath;
	pMotor->emPathState = PATH_RUN;
}


void SetPathEndPosWithVEx2(ST_MOTOR_CTRL *pMotor,ST_ACTION_PATH * pPath,FP32 siEndPos)
{
	switch(pMotor->emState)
	{
		case MOTOR_PATH_CLOSE_LOOP_CTRL:
			pPath->ssStartV = pMotor->fpVeltFB;
			break;
		case MOTOR_POS_CLOSE_LOOP_CTRL:
			pPath->ssStartV = pMotor->fpVeltFB;
			break;
		case MOTOR_VELT_CLOSE_LOOP_CTRL:
			pPath->ssStartV = pMotor->fpVeltFB;
			break;
	       case MOTOR_STOP_QUICKLY:
		   	pPath->ssStartV = pMotor->fpVeltFB;
		   	break;
	       case MOTOR_STOP_SLOWLY:
		   	pPath->ssStartV = pMotor->fpVeltFB;
		   	break;
		case MOTOR_BREAK:
			pPath->ssStartV = CalVelt(pMotor);
                     break;
	       case MOTOR_OPEN_LOOP_CTRL:
		   	pPath->ssStartV = CalVelt(pMotor);
		       break;
		case MOTOR_OTHER:
			pPath->ssStartV = CalVelt(pMotor);
			break;
	       default:
		   	pPath->ssStartV = CalVelt(pMotor);
		   	break;
		
	}

	pPath->siStartCode = pMotor->fpCoffPot * READ_CODER(pMotor->ucCoderChan);
	pPath->siEndCode = siEndPos;
	//防止起始点和结束点重合
	if(abs(pPath->siEndCode - pPath->siStartCode)<1)
	{
		pPath->siEndCode = pPath->siStartCode + 1;
	}
	
	if(pPath->siEndCode >= pPath->siStartCode)//设为scOver为增量运行
	{
		pPath->scOver |= 0x01;
	}
	else
	{
		pPath->scOver &= 0xfe;
	}
	pMotor->pstPrePath = NULL;
	pMotor->pstCurPath = pPath;
	pMotor->emPathState = PATH_RUN;
}




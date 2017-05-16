/**********************************************************************************************************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：RobotCtrl.c
最近修改日期：2011.07.07
版本：1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
模块描述：自己写的一些小函数
函数列表： 


----------------------------------------------------------------------------------------------------------------------------------------------------------
修订记录：
作者        时间            版本     说明

**********************************************************************************************************************************************************/
#include "HITCRT_RobotTypes.h"
#include "HITCRT_Types.h"
#include "math.h"
#include "HITCRT_Coff.h"
#include "PrivateFunction.h"
#include "stm32f4xx.h"
#include "CAN_API.h"
#include "RobotCtrl.h"
#include "stdlib.h"





UINT32 GetCurTime(void)
{
	UINT32 uiTime;
	uiTime =  ((UINT32)TIM3->CNT << 16) | TIM2->CNT;
	return uiTime;
}

//按照任意角度分配速度siDirAngle为角度的十倍,逆时针为正方向
void ManualBaseVeltAllocate(ST_OMNI_MOBILE_ROBOT *pstR,ST_ROBOT_VELT *pstV, SINT32 siDirAngle)
{
	FP32 fpCosValue ;
	FP32 fpSinValue;
	FP32 fpTmpAngle;
	ST_ROBOT_VELT  stTmpVelt = {0,0,0};
	fpTmpAngle = ((FP32)siDirAngle) /DEG_10;
	//先将局部坐标系下转换为0度正方向
	fpCosValue = cos(fpTmpAngle);
	fpSinValue = sin(fpTmpAngle);
	stTmpVelt.fpW = pstV->fpW;
	stTmpVelt.fpVy = pstV->fpVy  * fpCosValue +  pstV->fpVx * fpSinValue;
	stTmpVelt.fpVx = pstV->fpVx * fpCosValue -  pstV->fpVy * fpSinValue;
	
	
	pstR->stBaseWcMotor.fpVeltDes = stTmpVelt.fpVx + stTmpVelt.fpW;
	pstR->stBaseWbMotor.fpVeltDes = -stTmpVelt.fpVx/2 - stTmpVelt.fpVy * 0.866 + stTmpVelt.fpW;
	pstR->stBaseWaMotor.fpVeltDes = -stTmpVelt.fpVx/2 + stTmpVelt.fpVy * 0.866 + stTmpVelt.fpW;
}


/****************************************************************************************************
函数名称:DesRobotV
函数功能: 由手柄模拟输出键值确定个方向速度
备注: 
****************************************************************************************************/
void DesRobotV(ST_JS_FPGA *pstJoyStickValue, ST_ROBOT_VELT *pstV)
{
	
	if((pstJoyStickValue->usJsLeft >> 8) > 0xae)
	{
		pstV->fpVx = ((pstJoyStickValue->usJsLeft >> 8) - 0xae) / 2;
	}
	else if((pstJoyStickValue->usJsLeft >> 8) < 0x5e)
	{
		pstV->fpVx = -(0x5e - (pstJoyStickValue->usJsLeft >> 8)) / 2;
	}
	else
	{
		pstV->fpVx = 0;
	}
					
	if((pstJoyStickValue->usJsLeft & 0xff) > 0x9e)
	{
		pstV->fpVy = -((pstJoyStickValue->usJsLeft & 0xff) - 0x9e) / 2;
	}
	else if((pstJoyStickValue->usJsLeft & 0xff) < 0x5e)
	{
		pstV->fpVy = (0x5e - (pstJoyStickValue->usJsLeft & 0xff)) / 2;
	}
	else
	{
		pstV->fpVy = 0;
	}
						
	if((pstJoyStickValue->usJsRight & 0xff) > 0xae)
	{
		pstV->fpW = ((pstJoyStickValue->usJsRight & 0xff) - 0xae) / 2;
	}
	else if(((pstJoyStickValue->usJsRight & 0xff) < 0x5e))
	{
		pstV->fpW = -(0x5e - (pstJoyStickValue->usJsRight & 0xff)) / 2;
	}
	else
	{
		pstV->fpW = 0;
	}
}


//按照任意角度分配速度siDirAngle为角度的十倍,逆时针为正方向
void AutoBaseVeltAllocate(ST_OMNI_MOBILE_ROBOT *pstR,ST_ROBOT_VELT *pstV, SINT32 siDirAngle)
{
	FP32 fpCosValue ;
	FP32 fpSinValue;
	FP32 fpTmpAngle;
	ST_ROBOT_VELT  stTmpVelt = {0,0,0};
	fpTmpAngle = ((FP32)siDirAngle) /DEG_10;
	//先将局部坐标系下转换为0度正方向
	fpCosValue = cos(fpTmpAngle);
	fpSinValue = sin(fpTmpAngle);
	stTmpVelt.fpW = pstV->fpW;
	stTmpVelt.fpVy = pstV->fpVy  * fpCosValue +  pstV->fpVx * fpSinValue;
	stTmpVelt.fpVx = pstV->fpVx * fpCosValue -  pstV->fpVy * fpSinValue;

	stTmpVelt.fpVx = L_X * stTmpVelt.fpVx;
	stTmpVelt.fpVy = L_Y *  stTmpVelt.fpVy * 0.866;
	stTmpVelt.fpW  = L_W *  stTmpVelt.fpW;
	
	pstR->stBaseWcMotor.fpVeltDes =    stTmpVelt.fpVx +  stTmpVelt.fpW;
	pstR->stBaseWbMotor.fpVeltDes = - stTmpVelt.fpVx/2 -  stTmpVelt.fpVy + stTmpVelt.fpW;
	pstR->stBaseWaMotor.fpVeltDes = -  stTmpVelt.fpVx/2 +  stTmpVelt.fpVy+ stTmpVelt.fpW;
}

void ZerosPID(ST_PID * pPid)
{
	pPid->fpE = 0;
	pPid->fpU = 0;
	pPid->fpPreE = 0;
	pPid->fpPrePreE = 0;
}


void StartMEMSGryo(void)
{
	USART_SendData(USART6, 0x11);
       while(USART_GetFlagStatus(USART6 ,USART_FLAG_TC ) != SET);
	USART_ClearFlag(USART6 ,USART_FLAG_TC);
}



void StopMEMSGryo(void)
{
	USART_SendData(USART6, 0x10);
       while(USART_GetFlagStatus(USART6 ,USART_FLAG_TC ) != SET);
	USART_ClearFlag(USART6 ,USART_FLAG_TC);
}



void RcvMEMSGryo(UCHAR8 RcvData,SSHORT16 * pssAngle)
{

	static UCHAR8 ucIndex = 0;
	static UCHAR8 ucData[8] = {0};



      ucData[ucIndex] = RcvData;
      
      if(ucIndex>=3)
      {
	  	if(ucData[ucIndex -1]== 0xaa && ucData[ucIndex -3]== 0x55)
	  	{
			*pssAngle = ucData[ucIndex]  | ((SSHORT16) (ucData[ucIndex -2]) << 8);
			ucIndex = 0;
			return;
	  	}
      }

	ucIndex++;
	if(ucIndex >= 8)
	{
		ucIndex = 0;
	}



	
	
}


void SetValvePwm(UINT32 uiOpenTime,UINT32 uiTotalTime,UCHAR8 ucValveChan  )
{
    static UCHAR8 ucFirstRun = 0;
	static UINT32 uiTime1=0;
	if(ucFirstRun == 0)
	{
	    uiTime1 = GetCurTime();
		ucFirstRun = 1;
		return ; 
	}
	

	
	if(GetCurTime()-uiTime1<uiOpenTime)
		{
			OPEN_VALVE(ucValveChan);
		}
	else
		{
			CLOSE_VALVE(ucValveChan);
		}
	if((GetCurTime()-uiTime1)>=uiTotalTime)
		{	
			uiTime1=GetCurTime();
		}
		
}

UCHAR8 FindUCHAR8Max(UCHAR8 * pucData, UCHAR8 ucLen)
{
      UCHAR8 ucMax = 0;
      UCHAR8 i = 0;
      for(i = 0; i < ucLen;i++)
      	{
      	    if(pucData[i] > ucMax)
      	    	{
      	    	   ucMax = pucData[i];
      	    	}
      	}
	 return ucMax;
}

UCHAR8 FindUCHAR8Min(UCHAR8 * pucData, UCHAR8 ucLen)
{
      UCHAR8 ucMin= 255;
      UCHAR8 i = 0;
      for(i = 0; i < ucLen;i++)
      	{
      	    if(pucData[i] < ucMin)
      	    	{
      	    	   ucMin = pucData[i];
      	    	}
      	}
	 return ucMin;
}


SINT32 RetVisionLineValue(ST_VDL * pstVDL)
{
        SINT32 siRet = -1;

		
	 if((GetCurTime() -  g_uiVisionUpdateTime)>600000)
      	{
            return -1;
      	}

      if((pstVDL->Dis1<=0) ||(pstVDL->Dis2<=0))
      	{
      	    return -2;
      	}

      if((abs(pstVDL->Dis1 - pstVDL->Dis2)>=400) || ( abs(pstVDL->Dis1 - pstVDL->Dis2)<=200 ))

      	{
      	    return -3;
      	}

	if(abs( pstVDL->Rho1 - pstVDL->Rho2) > 5)
	{
	     return -4;
	}

	if((abs( pstVDL->Rho1) > 15)  ||  (abs( pstVDL->Rho2) > 15))
	{
	     return -5;
	}

	if(pstVDL->NumLines != 2)
	{
	    return -6;
	}

	siRet = (SINT32)(pstVDL->Dis1 > pstVDL->Dis2 ? pstVDL->Dis2 : pstVDL->Dis1);
	g_ucVisionUsedCnt++;
	return siRet;

}


FP32  ReadRobotFpVy(ST_OMNI_MOBILE_ROBOT *pstR)
{
		FP32 fpVy;
		FP32 fpQ;
		fpQ=pstR->stPot.ssPosQ*RADIAN_10;
		fpVy=(1.7321/3*cos(fpQ) - 1.0/3*sin(fpQ))*pstR->stBaseWaMotor.fpVeltFB
			- (1.0/3*sin(fpQ)+ 1.732/3*cos(fpQ))*pstR->stBaseWbMotor.fpVeltFB
			+ 2.0/3*sin(fpQ)*pstR->stBaseWcMotor.fpVeltFB;
		fpVy *= 5.3198;
		return fpVy;
}
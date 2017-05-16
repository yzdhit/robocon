#include "HITCRT_Types.h"
#include "HITCRT_Coff.h"
#include "math.h"
#include "HITCRT_RobotTypes.h"
#include "HITCRT_Algorithm.h"
#include "HITCRT_API.h"
#include "Navigation.h"
#include "PrivateFunction.h"
#include "MotorCtrl.h"
//#include "path.h"




extern ST_OMNI_MOBILE_ROBOT stRobot;
//extern ST_VAR_WATCH stVarWatch;//观察变量结构体
/*
//全向轮随动轮坐标反馈
FP32	g_fpOMNI_X_Position, g_fpOMNI_Y_Position;
FP32	g_fpOMNI_Base_X,g_fpOMNI_Base_Y;
//坐标修正相关
SSHORT16 g_ssXrayPosFix = 0;					 //全场X轴坐标修正
SSHORT16 g_ssYrayPosFix = 0;					 //全场Y轴坐标修正
SSHORT16 g_ssQdegPosFix = 0;					 //全场Q航向角修正
*/
/*******************************************************************
函数名称：DoubleOMNINav()
函数功能：利用双全向轮随动轮求出机器人全场相对坐标
输入：      
输出：    无
备注：   
***************************** ***************************************/
/*
void DoubleOMNINav(ST_FBOMNILENTH_STRUCT *stOMNICoderFB)
{
	FP32 fpDeltaDegA, fpDeltaDegB, fpDeltaDegC;
	FP32 fpDeltaX, fpDeltaY;
	FP32 fpDeltaX0,fpDeltaY0;
	FP32 fpCalQ1,fpCalQ2,fpCalQ3,fpK;

	stRobot.stPot.ssPosQ = stGryo.ssQ;
	
	stOMNICoderFB->siPosQ = stRobot.stPot.ssPosQ;			//读取航向角
	stOMNICoderFB->siCoderA = -READ_CODER(OMNI_CODER_A);	//读取随动轮A脉冲数
	stOMNICoderFB->siCoderB = -READ_CODER(OMNI_CODER_B);	//读取随动轮B脉冲数
	
	fpDeltaDegA = (stOMNICoderFB->siCoderA - stOMNICoderFB->siLastCoderA) / stOMNICoderFB->fpKCodeToDegA;	//由码盘脉冲数计算随动轮A单次计算周期滚动角度
	fpDeltaDegB = (stOMNICoderFB->siCoderB - stOMNICoderFB->siLastCoderB) / stOMNICoderFB->fpKCodeToDegB;	//由码盘脉冲数计算随动轮B单次计算周期滚动角度
	fpDeltaDegC = (3 * stOMNICoderFB->fpOMNI_Position_L * (stRobot.stPot.ssPosQ - stOMNICoderFB->siLastPosQ) * RADIAN_10) / 
					stOMNICoderFB->fpOMNI_WHEEL_R - fpDeltaDegA - fpDeltaDegB;	//由双随动码盘推算出第三个码盘C的角度增量
	
	fpCalQ3 = stRobot.stPot.ssPosQ * RADIAN_10 + PI_6 + stOMNICoderFB->fpPosQFix * RADIAN_10;	//机械航向角矫正
	fpCalQ1 = PI_3 + fpCalQ3;
	fpCalQ2 = PI_3 - fpCalQ3;
	
	fpK = stOMNICoderFB->fpOMNI_WHEEL_R / (3 * sin(PI_3));
	
	fpDeltaX = fpK * ((cos(fpCalQ1) - cos(fpCalQ2)) * fpDeltaDegA + (-cos(fpCalQ3) - cos(fpCalQ1)) * fpDeltaDegB + 
			   (cos(fpCalQ3) + cos(fpCalQ2)) * fpDeltaDegC);	//局部坐标系X方向增量 
	fpDeltaY = fpK * ((sin(fpCalQ2) + sin(fpCalQ1)) * fpDeltaDegA + (-sin(fpCalQ3) - sin(fpCalQ1)) * fpDeltaDegB +
			   (sin(fpCalQ3) - sin(fpCalQ2)) * fpDeltaDegC);	//局部坐标系Y方向增量
	
	//旋转坐标系进行修正，否则造成航向角变化以后的累积误差
	fpDeltaX0 = fpDeltaY * sin(stRobot.stPot.ssPosQ * RADIAN_10) + fpDeltaX * cos(stRobot.stPot.ssPosQ * RADIAN_10);
	fpDeltaY0 = fpDeltaY * cos(stRobot.stPot.ssPosQ * RADIAN_10) - fpDeltaX * sin(stRobot.stPot.ssPosQ * RADIAN_10);
	
	if(fpDeltaX0 > 0)	//X轴正负方向矫正
	{
		fpDeltaX0 = 0.961484347 * fpDeltaX0;
	}
	else
	{
		fpDeltaX0 = 0.958521193 * fpDeltaX0;
	}
	
	if(fpDeltaY0 > 0)	//Y轴正负方向矫正
	{
		fpDeltaY0 = 1.01840809 * fpDeltaY0;
	}
	else
	{
		fpDeltaY0 = 1.022529162 * fpDeltaY0;
	}
	
	fpDeltaX = fpDeltaX0 * cos(stRobot.stPot.ssPosQ * RADIAN_10) - fpDeltaY0 * sin(stRobot.stPot.ssPosQ * RADIAN_10);
	fpDeltaY = fpDeltaX0 * sin(stRobot.stPot.ssPosQ * RADIAN_10) + fpDeltaY0 * cos(stRobot.stPot.ssPosQ * RADIAN_10);	
	
	g_fpOMNI_X_Position = g_fpOMNI_X_Position + fpDeltaX;
	g_fpOMNI_Y_Position = g_fpOMNI_Y_Position + fpDeltaY;
	
	//随动中心与底盘中心坐标转换
	g_fpOMNI_Base_X = g_fpOMNI_X_Position - OMNI_CENTER * sin(stRobot.stPot.ssPosQ * RADIAN_10);
	g_fpOMNI_Base_Y = g_fpOMNI_Y_Position + OMNI_CENTER * cos(stRobot.stPot.ssPosQ * RADIAN_10) - OMNI_CENTER;
		
	//加完巡线以后的代码
	stRobot.stPot.ssPosX = g_fpOMNI_Base_X + g_ssXrayPosFix;
	stRobot.stPot.ssPosY = g_fpOMNI_Base_Y + g_ssYrayPosFix;
	
	stOMNICoderFB->siLastPosQ = stOMNICoderFB->siPosQ;	
	stOMNICoderFB->siLastCoderA = stOMNICoderFB->siCoderA;
	stOMNICoderFB->siLastCoderB = stOMNICoderFB->siCoderB;
}
*/
/*******************************************************************
函数名称：CheckSeries()
函数功能：检查路段以开始下一段
输入：    pstNavPath导航路径，pstPos全局坐标 
输出：    无
备注：
********************************************************************/
static void CheckSeries(ST_OMNI_MOBILE_ROBOT *pstR)
{	
	/*例：如果OverMode=0x02是第二象限推出，如果OverMode=0x03是到达一二象限退出
	**存在的问题是如果启动点在退出象限内的话这条路径就被跳过了*/
	//实际使用的时候，一般是多个象限同时检测，例如可以同时检测1 2象限，这样只要实际点到达这两个象限任意一个，都认为到达
	FP32 fpE;// = (pstR->pstCurPath->ucType == LINE ? pstR->stNavPidLine.stPidVtc.fpE : pstR->stNavPidCir.stPidVtc.fpE);
	if(pstR->pstCurPath->ucType == LINE)
	{
		fpE = pstR->stNavPidLine.stPidVtc.fpE;
	}
	else
	{
		fpE = pstR->stNavPidCir.stPidVtc.fpE;
	}
//	FP32 fpK = (fpPathDis > 0 ? 1.0 : -1.0);

//****************watch*******************////////////////
//	stVarWatch.fpE = fpE;
//	stVarWatch.fpK = fpK;
//	stVarWatch.ssPotX = pstR->stPot.ssPosX;
//	stVarWatch.ssPotY = pstR->stPot.ssPosY;
//	stVarWatch.ssPotQ = pstR->stPot.ssPosQ;
//	stVarWatch.fpDis  = fpPathDis;
//***********************************////////////////	
	if(fpE < pstR->stPotSen.ssPosY && pstR->emPathState == PATH_RUN)
//	if(pstR->emPathState == PATH_RUN && (pstR->stPot.ssPosX - pstR->pstCurPath->ssEndX) * (pstR->stPot.ssPosX - pstR->pstCurPath->ssEndX) + (pstR->stPot.ssPosY - pstR->pstCurPath->ssEndY) * (pstR->stPot.ssPosY - pstR->pstCurPath->ssEndY) < pstR->stPotSen.ssPosY * pstR->stPotSen.ssPosY)
	{
		if(pstR->pstCurPath->scOver & 0x10)//结束时要位置闭环控制
		{
			pstR->emPathState = PATH_END;
		}
		else
		{
			if ((pstR->pstCurPath->scOver & (1 << 0)) && (pstR->stPot.ssPosY >= pstR->pstCurPath->ssEndY) && (pstR->stPot.ssPosX >= pstR->pstCurPath->ssEndX))     //以目标点画坐标系的第一象限
			{	
				pstR->emPathState = PATH_END;		
			}
			else if ((pstR->pstCurPath->scOver & (1 << 1)) && (pstR->stPot.ssPosY >= pstR->pstCurPath->ssEndY) && (pstR->stPot.ssPosX <= pstR->pstCurPath->ssEndX)) //以目标点画坐标系的第二象限	
			{	
				pstR->emPathState = PATH_END;
			}
			else if ((pstR->pstCurPath->scOver & (1 << 2)) && (pstR->stPot.ssPosY <= pstR->pstCurPath->ssEndY) && (pstR->stPot.ssPosX <= pstR->pstCurPath->ssEndX)) //以目标点画坐标系的第三象限
			{	
				pstR->emPathState = PATH_END;
			}
			else if ((pstR->pstCurPath->scOver & (1 << 3)) && (pstR->stPot.ssPosY <= pstR->pstCurPath->ssEndY) && (pstR->stPot.ssPosX >= pstR->pstCurPath->ssEndX)) //以目标点画坐标系的第四象限
			{	
				pstR->emPathState = PATH_END;
			} 
		}
	}
}

/***********************************************************************************************************************
函数名: void DistributeSpeed(ST_POSITION *pstDPos,ST_SPEED *pstTargetSpeed,FP32 fpAngle)
函数功能: 根据车子目标速度和目标转速，为每个轮子分配速度
输入: *pstDPos 指向车子目标速度结构体的指针 (这里使用的是位置的结构体，位置的微分就是速度）;
      *pstTargetSpeed 指向车轮目标速度的指针;
      fpAngle 车子航向角;
输出: 无
备注: 
************************************************************************************************************************/

static void DistributeSpeed(ST_OMNI_MOBILE_ROBOT *pstR, FP32 fpAngle)
{   	
	pstR->stBaseWcMotor.fpVeltDes = pstR->stVeltDes.fpVx * cos(fpAngle)
	                       				  + pstR->stVeltDes.fpVy * sin(fpAngle) 
	                      	 			  + pstR->stVeltDes.fpW;	//C轮速度
	                       
	pstR->stBaseWbMotor.fpVeltDes  = -pstR->stVeltDes.fpVx * cos(PI_3 + fpAngle)
	                       				    - pstR->stVeltDes.fpVy* sin(PI_3 + fpAngle) 
	                       				    + pstR->stVeltDes.fpW;	//B轮速度
	                       
	pstR->stBaseWaMotor.fpVeltDes = -pstR->stVeltDes.fpVx * cos(PI_3 - fpAngle)
	                      				  + pstR->stVeltDes.fpVy * sin(PI_3 - fpAngle) 
	                      				  + pstR->stVeltDes.fpW;	//A轮速度
}/*
static void DistributeSpeed(ST_OMNI_MOBILE_ROBOT *pstR, FP32 fpAngle)
{ 
	FP32 fpSinAngle,fpCosAngle;
	fpSinAngle = sin(fpAngle);
	fpCosAngle = cos(fpAngle);
	
	pstR->stBaseWcMotor.fpVeltDes = pstR->stVeltDes.fpVx * fpCosAngle
	                       				  + pstR->stVeltDes.fpVy * fpSinAngle 
	                      	 			  + L1 * pstR->stVeltDes.fpW * RADIAN_10;	//C轮速度
	                       
	pstR->stBaseWaMotor.fpVeltDes  = -pstR->stVeltDes.fpVx * fpSinAngle
	                       				    + pstR->stVeltDes.fpVy * fpCosAngle 
	                       				    + L2 * pstR->stVeltDes.fpW * RADIAN_10;	//B轮速度
	                       
	pstR->stBaseWbMotor.fpVeltDes = pstR->stVeltDes.fpVx * fpSinAngle
	                      				  - pstR->stVeltDes.fpVy * fpCosAngle
	                      				  + L2 * pstR->stVeltDes.fpW * fpCosAngle * RADIAN_10;	//A轮速度
}*/
/***********************************************************************************************************************
函数名: void ManualCtrl(ST_OMNI_MOBILE_ROBOT *pstR, ST_ROBOT_VELT *pstV)
函数功能: 手动控制机器人根据车子目标速度和目标转速，为每个轮子分配速度
输入: ST_OMNI_MOBILE_ROBOT *pstR, 机器人个方向速度ST_ROBOT_VELT *pstV
输出: 无
备注: 
************************************************************************************************************************/

void ManualCtrl(ST_OMNI_MOBILE_ROBOT *pstR, ST_ROBOT_VELT *pstV)
{
	
	pstR->stBaseWcMotor.fpVeltDes = pstV->fpVx + pstV->fpW;	//C轮速度
	                       
	pstR->stBaseWbMotor.fpVeltDes = -pstV->fpVx / 2 - pstV->fpVy * 0.866 + pstV->fpW;	//B轮速度
	                       
	pstR->stBaseWaMotor.fpVeltDes = -pstV->fpVx / 2 + pstV->fpVy * 0.866 + pstV->fpW;	//A轮速度
}


/*******************************************************************
函数名称：AutoPathDecisionOMNI()
函数功能：自动规划加速距离和减速距离，并对最高速度进行限制
输入：    pstNavPath导航路径, ssVecPos向量位置，ssVecLenth向量总长
                    pssAim机器人要求速度，非真正轮速
输出：    无
备注：    加减函数是根据 ssV2*ssV2-ssV1*ssV1=2as 计算（匀加速运动），
          先根据路段距离和起始终止速度计算出加速度，再按所走的坐标反求当前目标速度	
********************************************************************/
static void AutoPathDecisionOMNI(ST_OMNI_MOBILE_ROBOT* pstR, USHORT16 fpPath_Dis ,USHORT16 fpPath_Dis_E)
{	   
	static FP32 s_fpPathMaxV;		//路径最大速度
	static USHORT16 s_usAcUpLenth;	//加速向量里程
	static USHORT16 s_usAcDownLenth;	//减速向量里程
	USHORT16 ssAcLenthTmp;
	static FP32 s_fpAccUpTmp;
	static FP32 s_fpAccDownTmp;
	/*更新路径*/
	if (pstR->pstCurPath != pstR->pstPrePath)                                       
	{
		if(pstR->pstCurPath->ucMode != 0)	//当处于非匀速运行模式时
		{	
			if(pstR->pstCurPath->usStartV < pstR->pstCurPath->usMaxV)	//判定是否需要有加速路段
			{	
				s_fpAccUpTmp = pstR->pstCurPath->fpAccUp * 1.0;//单位转换fpAccUp单位为mm/s*s
				s_usAcUpLenth = ((UINT32)SQUARE(pstR->pstCurPath->usMaxV) - (UINT32)SQUARE(pstR->pstCurPath->usStartV)) / s_fpAccUpTmp / 2.0;	//理论加速距离
			}
			else
			{
				s_usAcUpLenth = 0;
			}
			
			if(pstR->pstCurPath->usEndV < pstR->pstCurPath->usMaxV)	//判定是否需要有减速路段
			{
				s_fpAccDownTmp = pstR->pstCurPath->fpAccDown * 1.0;
				s_usAcDownLenth = ((UINT32)SQUARE(pstR->pstCurPath->usMaxV) - (UINT32)SQUARE(pstR->pstCurPath->usEndV)) / s_fpAccDownTmp/2.0;	//理论减速距离
			}
			else
			{
				s_usAcDownLenth = 0;
			}
			
			ssAcLenthTmp = s_usAcUpLenth + s_usAcDownLenth;	//计算总加减速距离
			
			if(ssAcLenthTmp > fpPath_Dis)	//加减速距离超过总行程，重新计算最高速度
			{
				s_fpPathMaxV = sqrt( (2 * fpPath_Dis * s_fpAccUpTmp * s_fpAccDownTmp  
									+ s_fpAccDownTmp * SQUARE(pstR->pstCurPath->usStartV)
									+ s_fpAccUpTmp * SQUARE(pstR->pstCurPath->usEndV))
									/ (s_fpAccUpTmp + s_fpAccDownTmp) );
				
				s_usAcUpLenth = (SQUARE(s_fpPathMaxV) - (UINT32)SQUARE(pstR->pstCurPath->usStartV)) / s_fpAccUpTmp / 2.0;
				s_usAcDownLenth = fpPath_Dis - s_usAcUpLenth;
			}
			else
			{
				s_fpPathMaxV = pstR->pstCurPath->usMaxV;
			}
		}
	}
		
	if (pstR->pstCurPath->ucMode == 0)	//如果恒速运行，路径全程执行初始速度
	{
		pstR->stVeltDes.fpVy = pstR->pstCurPath->usStartV;
	}
	else if(pstR->pstCurPath->ucMode == 1)	//如果加减速运行
	{
		if(fpPath_Dis - fpPath_Dis_E < s_usAcUpLenth)	//向量里程处于加速里程内
		{
			if(fpPath_Dis - fpPath_Dis_E > 0)
			{
				pstR->stVeltDes.fpVy = sqrt(2 * s_fpAccUpTmp * (fpPath_Dis - fpPath_Dis_E) + (FP32)SQUARE(pstR->pstCurPath->usStartV));
			}
			else
			{
				pstR->stVeltDes.fpVy = pstR->pstCurPath->usStartV;
			}
		}
		else if(fpPath_Dis - fpPath_Dis_E >= s_usAcUpLenth && fpPath_Dis_E >= s_usAcDownLenth)	//向量里程处于匀速里程内
		{
			pstR->stVeltDes.fpVy = s_fpPathMaxV;
		}
		else if(fpPath_Dis_E < s_usAcDownLenth)
		{
			pstR->stVeltDes.fpVy = sqrt(2.0 * s_fpAccDownTmp * fpPath_Dis_E + (FP32)SQUARE(pstR->pstCurPath->usEndV));
		}
	}
}
/***********************************************************************************************************************
函数名: void DistributeSpeed(ST_POSITION *pstDPos,ST_SPEED *pstTargetSpeed,FP32 fpAngle)
函数功能: 根据车子目标速度和目标转速，为每个轮子分配速度
输入:  *pstR
输出: 无
备注: 
************************************************************************************************************************/
void NavLine(ST_OMNI_MOBILE_ROBOT *pstR)
{   
	ST_VECTOR stVecPE,stVecSE;
	FP32 fpAngLocal;
	FP32 fpAngCourse;		//航向角，单位：RAD弧度

	FP32 fpPathDis;//路段总长

	fpAngCourse = ((FP32)(pstR->stPot.ssPosQ)) * RADIAN_10;
    
	/*计算路段起点指向终点的向量SE*/
	stVecSE.ssVx = pstR->pstCurPath->ssEndX - pstR->pstCurPath->ssStartX;
	stVecSE.ssVy = pstR->pstCurPath->ssEndY - pstR->pstCurPath->ssStartY;
	/*计算机器人当前位置指向路段终点的向量PE*/
	stVecPE.ssVx = pstR->pstCurPath->ssEndX - pstR->stPot.ssPosX;
	stVecPE.ssVy = pstR->pstCurPath->ssEndY - pstR->stPot.ssPosY;

	/*计算路段直线与机器人夹角*/
	fpAngLocal = ConvertAngle(fpAngCourse - CalAngle(stVecSE));
	
	/*计算旋转PID*/
	pstR->stNavPidLine.stPidRot.fpE = ConvertDeg(pstR->pstCurPath->ssEndQ - pstR->stPot.ssPosQ);	//将偏差角度转到[-1800,1800)(0.1°)范围内
	CalPIDWTCOL(&pstR->stNavPidLine.stPidRot);
	pstR->stVeltDes.fpW = Round(pstR->stNavPidLine.stPidRot.fpU);
	pstR->stVeltDes.fpW = Clip(pstR->stVeltDes.fpW, -pstR->stVeltLimit.fpW, pstR->stVeltLimit.fpW);	//限制旋转的角速度
	
	/*计算横向PID*/
	pstR->stNavPidLine.stPidTrvs.fpE = CalNormalProjection(stVecPE,stVecSE);
	CalPIDWTCOL(&pstR->stNavPidLine.stPidTrvs);
	pstR->stVeltDes.fpVx = pstR->stNavPidLine.stPidTrvs.fpU;
	pstR->stVeltDes.fpVx = Clip(pstR->stVeltDes.fpVx, -pstR->stVeltLimit.fpVx, pstR->stVeltLimit.fpVx);	//限制横向移动速度
	
	/*计算纵向PID*/
	pstR->stNavPidLine.stPidVtc.fpE = CalRadialProjection(stVecPE,stVecSE);
	fpPathDis = CalRadialProjection(stVecSE, stVecSE);

	if(pstR->emPathState == PATH_END && (pstR->pstCurPath->scOver & 0x10))//结束时位置闭环
	{
		CalPIDWTCOL(&pstR->stNavPidLine.stPidVtc);
		pstR->stVeltDes.fpVy = pstR->stNavPidLine.stPidVtc.fpU;
	}
	else
	{
	//	GoSeries(pstR, fpPathDis, pstR->stNavPidLine.stPidVtc.fpE);
		AutoPathDecisionOMNI(pstR, fpPathDis, pstR->stNavPidLine.stPidVtc.fpE);
	}
	
	pstR->stVeltDes.fpVy = Clip(pstR->stVeltDes.fpVy, -pstR->stVeltLimit.fpVy, pstR->stVeltLimit.fpVy);
	
	/*分配各轮速度*/
	DistributeSpeed(pstR,fpAngLocal);

	CheckSeries(pstR);
	pstR->pstPrePath = pstR->pstCurPath;//将本次路径的指针赋给上一次
}


void NavLineEx1(ST_OMNI_MOBILE_ROBOT *pstR)
{
	ST_VECTOR stVecPE,stVecSE;
	FP32 fpAngLocal;
	FP32 fpAngCourse;		//航向角，单位：RAD弧度
       FP32 fpDis_E;
	FP32 fpPathDis;//路段总长
	FP32 fpTmpQDes;
	FP32 fpCurRatio;

	fpAngCourse = ((FP32)(pstR->stPot.ssPosQ)) * RADIAN_10;
    
	/*计算路段起点指向终点的向量SE*/
	stVecSE.ssVx = pstR->pstCurPath->ssEndX - pstR->pstCurPath->ssStartX;
	stVecSE.ssVy = pstR->pstCurPath->ssEndY - pstR->pstCurPath->ssStartY;
	/*计算机器人当前位置指向路段终点的向量PE*/
	stVecPE.ssVx = pstR->pstCurPath->ssEndX - pstR->stPot.ssPosX;
	stVecPE.ssVy = pstR->pstCurPath->ssEndY - pstR->stPot.ssPosY;

	/*计算路段直线与机器人夹角*/
	fpAngLocal = ConvertAngle(fpAngCourse - CalAngle(stVecSE));
	
	
	
	/*计算横向PID*/
	/*x方向上的误差应该是mm单位*/
	pstR->stNavPidLine.stPidTrvs.fpE = CalNormalProjection(stVecPE,stVecSE);
	CalPosPID(&pstR->stNavPidLine.stPidTrvs);
	pstR->stVeltDes.fpVx = pstR->stNavPidLine.stPidTrvs.fpU;
	pstR->stVeltDes.fpVx = Clip(pstR->stVeltDes.fpVx, -pstR->stVeltLimit.fpVx, pstR->stVeltLimit.fpVx);	//限制横向移动速度
	
	/*计算纵向PID*/
	/*Y方向上的误差应该是mm单位*/
	pstR->stNavPidLine.stPidVtc.fpE = CalRadialProjection(stVecPE,stVecSE);
       fpDis_E = pstR->stNavPidLine.stPidVtc.fpE;
	fpPathDis = CalRadialProjection(stVecSE, stVecSE);

	if(pstR->emPathState == PATH_END && (pstR->pstCurPath->scOver & 0x10))//结束时位置闭环
	{
		CalPosPID(&pstR->stNavPidLine.stPidVtc);
		pstR->stVeltDes.fpVy = pstR->stNavPidLine.stPidVtc.fpU;
	}
	else
	{
	//	GoSeries(pstR, fpPathDis, pstR->stNavPidLine.stPidVtc.fpE);
	       if(pstR->stNavPidLine.stPidVtc.fpE<0)
	       {
		   	pstR->stNavPidLine.stPidVtc.fpE = 0;
	       }
		AutoPathDecisionOMNI(pstR, fpPathDis, pstR->stNavPidLine.stPidVtc.fpE);
	}
	
	pstR->stVeltDes.fpVy = Clip(pstR->stVeltDes.fpVy, -pstR->stVeltLimit.fpVy, pstR->stVeltLimit.fpVy);

      
	/*计算旋转PID*/
	 if(fpDis_E >= ((0.5 + pstR->pstCurPath->fpDisRatio/2) *fpPathDis))
	 {
	 	// 跟踪起始值
	 	pstR->stNavPidLine.stPidRot.fpE  = ConvertDeg(pstR->pstCurPath->ssStartQ - pstR->stPot.ssPosQ);
	 }
	 else if(fpDis_E <= ((0.5 - pstR->pstCurPath->fpDisRatio/2) *fpPathDis))
	 {
	 	// 跟踪终止值
	 	pstR->stNavPidLine.stPidRot.fpE = ConvertDeg(pstR->pstCurPath->ssEndQ - pstR->stPot.ssPosQ);	//将偏差角度转到[-1800,1800)(0.1°)范围内
	 }
	 else
	 {
	       //跟踪Y 分配的余弦
	       fpCurRatio = (fpPathDis - fpDis_E  -  (0.5 - pstR->pstCurPath->fpDisRatio/2) *fpPathDis)/( fpPathDis* pstR->pstCurPath->fpDisRatio);
	       fpTmpQDes =( pstR->pstCurPath->ssEndQ - pstR->pstCurPath->ssStartQ )*(fpCurRatio ) +  pstR->pstCurPath->ssStartQ;
	       pstR->stPotDes.ssPosQ =(SSHORT16) fpTmpQDes;
	       pstR->stNavPidLine.stPidRot.fpE = ConvertDeg(pstR->stPotDes.ssPosQ- pstR->stPot.ssPosQ);
	 }
	//CalPIDIS(&pstR->stNavPidLine.stPidRot);
	CalPosPID(&pstR->stNavPidLine.stPidRot);
	pstR->stVeltDes.fpW = Round(pstR->stNavPidLine.stPidRot.fpU);
	pstR->stVeltDes.fpW = Clip(pstR->stVeltDes.fpW, -pstR->stVeltLimit.fpW, pstR->stVeltLimit.fpW);	//限制旋转的角速度
	
	/*分配各轮速度*/
	//DistributeSpeed(pstR,fpAngLocal);
	AutoBaseVeltAllocate(pstR, &(pstR->stVeltDes),-fpAngLocal/RADIAN_10);

	CheckSeries(pstR);
	pstR->pstPrePath = pstR->pstCurPath;//将本次路径的指针赋给上一次
}


/****************************************************************************************************
函数功能：圆弧PID导航，使机器人一定的航向角或与圆弧轨迹切向保持一定夹角沿圆弧轨迹运动
入口参数：pos：机器人当前位置
		  pid_cir：圆弧导航PID参数数据结构体
		  tpath：当前路径
		  tspeed：要分配的暂时目标速度
出口参数：1：表示已经行走至此路段终点
		  0：表示未行走至此路段终点
调试说明：采用向量计算距离，直接考虑正负号问题，减少三角函数的使用，从而提高了执行效率
		  385us左右，一条反三角函数或开方函数占用时间为30-40us,故一定要慎重使用
****************************************************************************************************/
void NavCircle(ST_OMNI_MOBILE_ROBOT* pstR)
{   
//	static UCHAR8 ucPathUpdateFlag = TRUE;//路径更新标志
	ST_VECTOR	stVecCP, stVecCE, stVecCS;
	FP32 fpAngCourse, fpAngCP, fpAngCE, fpAngCS,fpAngPN, fpDisCP, fpAngLocal;
	FP32 fpPathDis;//路段总长
	
	fpAngCourse =(FP32)pstR->stPot.ssPosQ * RADIAN_10;		//将航向角转换为弧度
	//----------计算机器人当前位置指向圆弧圆心的向量----------
	stVecCP.ssVx = pstR->stPot.ssPosX - pstR->pstCurPath->ssCenterX;
	stVecCP.ssVy = pstR->stPot.ssPosY - pstR->pstCurPath->ssCenterY;
	//----------计算机器人最终位置指向圆弧圆心的向量----------
	stVecCE.ssVx = pstR->pstCurPath->ssEndX - pstR->pstCurPath->ssCenterX;
	stVecCE.ssVy = pstR->pstCurPath->ssEndY - pstR->pstCurPath->ssCenterY;
	//----------计算机器人起始位置指向圆弧圆心的向量----------
	stVecCS.ssVx = pstR->pstCurPath->ssStartX - pstR->pstCurPath->ssCenterX;
	stVecCS.ssVy = pstR->pstCurPath->ssStartY - pstR->pstCurPath->ssCenterY;
	
	fpAngCP = CalAngle(stVecCP);		//计算CP向量夹角
	fpAngCE = CalAngle(stVecCE);        //计算CE向量夹角
	fpAngCS = CalAngle(stVecCS);        //计算CS向量夹角
	//计算PN向量夹角（即圆弧切向向量）
//	fpAngPN = ConvertAngle(pstR->pstCurPath->ssRadius > 0 ? fpAngCP - PI_2 : fpAngCP + PI_2);//ssRadius > 0顺圆
	if(pstR->pstCurPath->ssRadius > 0)		//顺圆
	{
		fpAngPN = ConvertAngle(fpAngCP - PI_2);
	}
	else 				//逆圆
	{
		fpAngPN = ConvertAngle(fpAngCP + PI_2);
	}
	
	//----------计算角度PID----------
	if(pstR->pstCurPath->ucType == 2)
	{
		pstR->stNavPidCir.stPidRot.fpE = ConvertDeg(pstR->pstCurPath->ssEndQ + Round(fpAngPN * RAD10) - pstR->stPot.ssPosQ);
	}
	else
	{   
		pstR->stNavPidCir.stPidRot.fpE = ConvertDeg(pstR->pstCurPath->ssEndQ - pstR->stPot.ssPosQ);
	}
	
	CalPIDWTCOL(&pstR->stNavPidCir.stPidRot);
	pstR->stVeltDes.fpW = pstR->stNavPidCir.stPidRot.fpU;
	//限制旋转的角速度
	pstR->stVeltDes.fpW = Clip(pstR->stVeltDes.fpW, -pstR->stVeltLimit.fpW, pstR->stVeltLimit.fpW);
	//----------计算径向PID----------
	fpDisCP = sqrt((SINT32)SQUARE(stVecCP.ssVx) + (SINT32)SQUARE(stVecCP.ssVy));/////此处有问题，向量过大会发生溢出
	//计算偏差
	
	if(pstR->pstCurPath->ssRadius > 0)	//顺圆
	{
		pstR->stNavPidCir.stPidTrvs.fpE = fpDisCP - pstR->pstCurPath->ssRadius;
	}
	else 			//逆圆
	{
		pstR->stNavPidCir.stPidTrvs.fpE = -pstR->pstCurPath->ssRadius - fpDisCP;//ssRadius带符号
	}

	CalPIDWTCOL(&pstR->stNavPidCir.stPidTrvs);
		
	pstR->stVeltDes.fpVx = Clip(pstR->stNavPidCir.stPidTrvs.fpU, -pstR->stVeltLimit.fpVx, pstR->stVeltLimit.fpVx);
	
	//----------计算切向PID----------
	
	//----------由于在pi和-pi处角度出现突变，故加入如下段程序----//
	if(pstR->pstCurPath->ssRadius > 0)//顺圆
	{
		if(fpAngCS < fpAngCE)//路径跨过pi
		{
			fpAngCS += PI2;
			if(fpAngCP < fpAngCE && fpAngCP + PI2 >= fpAngCE && fpAngCP + PI2 <= fpAngCS)//在路段中
			{
				fpAngCP += PI2;
			}
			else if(fpAngCP < fpAngCE)
			{
			
			}
		}
		else if(fpAngCS > fpAngCE)//未跨越pi
		{
			if(fpAngCP > fpAngCS)
			{
				fpAngCP -= PI2;
			}
		}
	}
	else if(pstR->pstCurPath->ssRadius < 0)//逆圆
	{
		if(fpAngCS > fpAngCE)//路径跨过pi
		{
			fpAngCS -= PI2;
			if(fpAngCP > fpAngCE && fpAngCP - PI2 <= fpAngCE && fpAngCP - PI2 >= fpAngCS)
			{
				fpAngCP -= PI2;
			}
		}
		else if(fpAngCS < fpAngCE)//未跨过pi
		{
			if(fpAngCP < fpAngCS)
			{
				fpAngCP += PI2;
			}
		}
	}
	
	//---------------------------------------------------------//
	pstR->stNavPidCir.stPidVtc.fpE = (fpAngCP - fpAngCE) * pstR->pstCurPath->ssRadius;		//计算距离终点的弧长，注意此处的R是带符号的
	fpPathDis = (fpAngCS - fpAngCE) * pstR->pstCurPath->ssRadius;//路径的总长度
	if(pstR->emPathState == PATH_END && (pstR->pstCurPath->scOver & 0x10))//结束时位置闭环
	{
		CalPIDWTCOL(&pstR->stNavPidCir.stPidVtc);
		pstR->stVeltDes.fpVy = Clip(pstR->stNavPidCir.stPidVtc.fpU, -pstR->stVeltLimit.fpVy, pstR->stVeltLimit.fpVy);
	}
	else
	{
//		GoSeries(pstR, fpPathDis, pstR->stNavPidCir.stPidVtc.fpE);
		AutoPathDecisionOMNI(pstR, fpPathDis, pstR->stNavPidCir.stPidVtc.fpE);
		pstR->stVeltDes.fpVy = Clip(pstR->stVeltDes.fpVy, -pstR->stVeltLimit.fpVy, pstR->stVeltLimit.fpVy);
	}
	
	//----------分配各轮速度----------//
	fpAngLocal = ConvertAngle(fpAngCourse - fpAngPN);		//圆弧切向与机器人夹角
	DistributeSpeed(pstR, fpAngLocal);

	CheckSeries(pstR);
	pstR->pstPrePath = pstR->pstCurPath;//将本次路径的指针赋给上一次
}



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
//extern ST_VAR_WATCH stVarWatch;//�۲�����ṹ��
/*
//ȫ�����涯�����귴��
FP32	g_fpOMNI_X_Position, g_fpOMNI_Y_Position;
FP32	g_fpOMNI_Base_X,g_fpOMNI_Base_Y;
//�����������
SSHORT16 g_ssXrayPosFix = 0;					 //ȫ��X����������
SSHORT16 g_ssYrayPosFix = 0;					 //ȫ��Y����������
SSHORT16 g_ssQdegPosFix = 0;					 //ȫ��Q���������
*/
/*******************************************************************
�������ƣ�DoubleOMNINav()
�������ܣ�����˫ȫ�����涯�����������ȫ���������
���룺      
�����    ��
��ע��   
***************************** ***************************************/
/*
void DoubleOMNINav(ST_FBOMNILENTH_STRUCT *stOMNICoderFB)
{
	FP32 fpDeltaDegA, fpDeltaDegB, fpDeltaDegC;
	FP32 fpDeltaX, fpDeltaY;
	FP32 fpDeltaX0,fpDeltaY0;
	FP32 fpCalQ1,fpCalQ2,fpCalQ3,fpK;

	stRobot.stPot.ssPosQ = stGryo.ssQ;
	
	stOMNICoderFB->siPosQ = stRobot.stPot.ssPosQ;			//��ȡ�����
	stOMNICoderFB->siCoderA = -READ_CODER(OMNI_CODER_A);	//��ȡ�涯��A������
	stOMNICoderFB->siCoderB = -READ_CODER(OMNI_CODER_B);	//��ȡ�涯��B������
	
	fpDeltaDegA = (stOMNICoderFB->siCoderA - stOMNICoderFB->siLastCoderA) / stOMNICoderFB->fpKCodeToDegA;	//�����������������涯��A���μ������ڹ����Ƕ�
	fpDeltaDegB = (stOMNICoderFB->siCoderB - stOMNICoderFB->siLastCoderB) / stOMNICoderFB->fpKCodeToDegB;	//�����������������涯��B���μ������ڹ����Ƕ�
	fpDeltaDegC = (3 * stOMNICoderFB->fpOMNI_Position_L * (stRobot.stPot.ssPosQ - stOMNICoderFB->siLastPosQ) * RADIAN_10) / 
					stOMNICoderFB->fpOMNI_WHEEL_R - fpDeltaDegA - fpDeltaDegB;	//��˫�涯�������������������C�ĽǶ�����
	
	fpCalQ3 = stRobot.stPot.ssPosQ * RADIAN_10 + PI_6 + stOMNICoderFB->fpPosQFix * RADIAN_10;	//��е����ǽ���
	fpCalQ1 = PI_3 + fpCalQ3;
	fpCalQ2 = PI_3 - fpCalQ3;
	
	fpK = stOMNICoderFB->fpOMNI_WHEEL_R / (3 * sin(PI_3));
	
	fpDeltaX = fpK * ((cos(fpCalQ1) - cos(fpCalQ2)) * fpDeltaDegA + (-cos(fpCalQ3) - cos(fpCalQ1)) * fpDeltaDegB + 
			   (cos(fpCalQ3) + cos(fpCalQ2)) * fpDeltaDegC);	//�ֲ�����ϵX�������� 
	fpDeltaY = fpK * ((sin(fpCalQ2) + sin(fpCalQ1)) * fpDeltaDegA + (-sin(fpCalQ3) - sin(fpCalQ1)) * fpDeltaDegB +
			   (sin(fpCalQ3) - sin(fpCalQ2)) * fpDeltaDegC);	//�ֲ�����ϵY��������
	
	//��ת����ϵ����������������ɺ���Ǳ仯�Ժ���ۻ����
	fpDeltaX0 = fpDeltaY * sin(stRobot.stPot.ssPosQ * RADIAN_10) + fpDeltaX * cos(stRobot.stPot.ssPosQ * RADIAN_10);
	fpDeltaY0 = fpDeltaY * cos(stRobot.stPot.ssPosQ * RADIAN_10) - fpDeltaX * sin(stRobot.stPot.ssPosQ * RADIAN_10);
	
	if(fpDeltaX0 > 0)	//X�������������
	{
		fpDeltaX0 = 0.961484347 * fpDeltaX0;
	}
	else
	{
		fpDeltaX0 = 0.958521193 * fpDeltaX0;
	}
	
	if(fpDeltaY0 > 0)	//Y�������������
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
	
	//�涯�����������������ת��
	g_fpOMNI_Base_X = g_fpOMNI_X_Position - OMNI_CENTER * sin(stRobot.stPot.ssPosQ * RADIAN_10);
	g_fpOMNI_Base_Y = g_fpOMNI_Y_Position + OMNI_CENTER * cos(stRobot.stPot.ssPosQ * RADIAN_10) - OMNI_CENTER;
		
	//����Ѳ���Ժ�Ĵ���
	stRobot.stPot.ssPosX = g_fpOMNI_Base_X + g_ssXrayPosFix;
	stRobot.stPot.ssPosY = g_fpOMNI_Base_Y + g_ssYrayPosFix;
	
	stOMNICoderFB->siLastPosQ = stOMNICoderFB->siPosQ;	
	stOMNICoderFB->siLastCoderA = stOMNICoderFB->siCoderA;
	stOMNICoderFB->siLastCoderB = stOMNICoderFB->siCoderB;
}
*/
/*******************************************************************
�������ƣ�CheckSeries()
�������ܣ����·���Կ�ʼ��һ��
���룺    pstNavPath����·����pstPosȫ������ 
�����    ��
��ע��
********************************************************************/
static void CheckSeries(ST_OMNI_MOBILE_ROBOT *pstR)
{	
	/*�������OverMode=0x02�ǵڶ������Ƴ������OverMode=0x03�ǵ���һ�������˳�
	**���ڵ�������������������˳������ڵĻ�����·���ͱ�������*/
	//ʵ��ʹ�õ�ʱ��һ���Ƕ������ͬʱ��⣬�������ͬʱ���1 2���ޣ�����ֻҪʵ�ʵ㵽����������������һ��������Ϊ����
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
		if(pstR->pstCurPath->scOver & 0x10)//����ʱҪλ�ñջ�����
		{
			pstR->emPathState = PATH_END;
		}
		else
		{
			if ((pstR->pstCurPath->scOver & (1 << 0)) && (pstR->stPot.ssPosY >= pstR->pstCurPath->ssEndY) && (pstR->stPot.ssPosX >= pstR->pstCurPath->ssEndX))     //��Ŀ��㻭����ϵ�ĵ�һ����
			{	
				pstR->emPathState = PATH_END;		
			}
			else if ((pstR->pstCurPath->scOver & (1 << 1)) && (pstR->stPot.ssPosY >= pstR->pstCurPath->ssEndY) && (pstR->stPot.ssPosX <= pstR->pstCurPath->ssEndX)) //��Ŀ��㻭����ϵ�ĵڶ�����	
			{	
				pstR->emPathState = PATH_END;
			}
			else if ((pstR->pstCurPath->scOver & (1 << 2)) && (pstR->stPot.ssPosY <= pstR->pstCurPath->ssEndY) && (pstR->stPot.ssPosX <= pstR->pstCurPath->ssEndX)) //��Ŀ��㻭����ϵ�ĵ�������
			{	
				pstR->emPathState = PATH_END;
			}
			else if ((pstR->pstCurPath->scOver & (1 << 3)) && (pstR->stPot.ssPosY <= pstR->pstCurPath->ssEndY) && (pstR->stPot.ssPosX >= pstR->pstCurPath->ssEndX)) //��Ŀ��㻭����ϵ�ĵ�������
			{	
				pstR->emPathState = PATH_END;
			} 
		}
	}
}

/***********************************************************************************************************************
������: void DistributeSpeed(ST_POSITION *pstDPos,ST_SPEED *pstTargetSpeed,FP32 fpAngle)
��������: ���ݳ���Ŀ���ٶȺ�Ŀ��ת�٣�Ϊÿ�����ӷ����ٶ�
����: *pstDPos ָ����Ŀ���ٶȽṹ���ָ�� (����ʹ�õ���λ�õĽṹ�壬λ�õ�΢�־����ٶȣ�;
      *pstTargetSpeed ָ����Ŀ���ٶȵ�ָ��;
      fpAngle ���Ӻ����;
���: ��
��ע: 
************************************************************************************************************************/

static void DistributeSpeed(ST_OMNI_MOBILE_ROBOT *pstR, FP32 fpAngle)
{   	
	pstR->stBaseWcMotor.fpVeltDes = pstR->stVeltDes.fpVx * cos(fpAngle)
	                       				  + pstR->stVeltDes.fpVy * sin(fpAngle) 
	                      	 			  + pstR->stVeltDes.fpW;	//C���ٶ�
	                       
	pstR->stBaseWbMotor.fpVeltDes  = -pstR->stVeltDes.fpVx * cos(PI_3 + fpAngle)
	                       				    - pstR->stVeltDes.fpVy* sin(PI_3 + fpAngle) 
	                       				    + pstR->stVeltDes.fpW;	//B���ٶ�
	                       
	pstR->stBaseWaMotor.fpVeltDes = -pstR->stVeltDes.fpVx * cos(PI_3 - fpAngle)
	                      				  + pstR->stVeltDes.fpVy * sin(PI_3 - fpAngle) 
	                      				  + pstR->stVeltDes.fpW;	//A���ٶ�
}/*
static void DistributeSpeed(ST_OMNI_MOBILE_ROBOT *pstR, FP32 fpAngle)
{ 
	FP32 fpSinAngle,fpCosAngle;
	fpSinAngle = sin(fpAngle);
	fpCosAngle = cos(fpAngle);
	
	pstR->stBaseWcMotor.fpVeltDes = pstR->stVeltDes.fpVx * fpCosAngle
	                       				  + pstR->stVeltDes.fpVy * fpSinAngle 
	                      	 			  + L1 * pstR->stVeltDes.fpW * RADIAN_10;	//C���ٶ�
	                       
	pstR->stBaseWaMotor.fpVeltDes  = -pstR->stVeltDes.fpVx * fpSinAngle
	                       				    + pstR->stVeltDes.fpVy * fpCosAngle 
	                       				    + L2 * pstR->stVeltDes.fpW * RADIAN_10;	//B���ٶ�
	                       
	pstR->stBaseWbMotor.fpVeltDes = pstR->stVeltDes.fpVx * fpSinAngle
	                      				  - pstR->stVeltDes.fpVy * fpCosAngle
	                      				  + L2 * pstR->stVeltDes.fpW * fpCosAngle * RADIAN_10;	//A���ٶ�
}*/
/***********************************************************************************************************************
������: void ManualCtrl(ST_OMNI_MOBILE_ROBOT *pstR, ST_ROBOT_VELT *pstV)
��������: �ֶ����ƻ����˸��ݳ���Ŀ���ٶȺ�Ŀ��ת�٣�Ϊÿ�����ӷ����ٶ�
����: ST_OMNI_MOBILE_ROBOT *pstR, �����˸������ٶ�ST_ROBOT_VELT *pstV
���: ��
��ע: 
************************************************************************************************************************/

void ManualCtrl(ST_OMNI_MOBILE_ROBOT *pstR, ST_ROBOT_VELT *pstV)
{
	
	pstR->stBaseWcMotor.fpVeltDes = pstV->fpVx + pstV->fpW;	//C���ٶ�
	                       
	pstR->stBaseWbMotor.fpVeltDes = -pstV->fpVx / 2 - pstV->fpVy * 0.866 + pstV->fpW;	//B���ٶ�
	                       
	pstR->stBaseWaMotor.fpVeltDes = -pstV->fpVx / 2 + pstV->fpVy * 0.866 + pstV->fpW;	//A���ٶ�
}


/*******************************************************************
�������ƣ�AutoPathDecisionOMNI()
�������ܣ��Զ��滮���پ���ͼ��پ��룬��������ٶȽ�������
���룺    pstNavPath����·��, ssVecPos����λ�ã�ssVecLenth�����ܳ�
                    pssAim������Ҫ���ٶȣ�����������
�����    ��
��ע��    �Ӽ������Ǹ��� ssV2*ssV2-ssV1*ssV1=2as ���㣨�ȼ����˶�����
          �ȸ���·�ξ������ʼ��ֹ�ٶȼ�������ٶȣ��ٰ����ߵ����귴��ǰĿ���ٶ�	
********************************************************************/
static void AutoPathDecisionOMNI(ST_OMNI_MOBILE_ROBOT* pstR, USHORT16 fpPath_Dis ,USHORT16 fpPath_Dis_E)
{	   
	static FP32 s_fpPathMaxV;		//·������ٶ�
	static USHORT16 s_usAcUpLenth;	//�����������
	static USHORT16 s_usAcDownLenth;	//�����������
	USHORT16 ssAcLenthTmp;
	static FP32 s_fpAccUpTmp;
	static FP32 s_fpAccDownTmp;
	/*����·��*/
	if (pstR->pstCurPath != pstR->pstPrePath)                                       
	{
		if(pstR->pstCurPath->ucMode != 0)	//�����ڷ���������ģʽʱ
		{	
			if(pstR->pstCurPath->usStartV < pstR->pstCurPath->usMaxV)	//�ж��Ƿ���Ҫ�м���·��
			{	
				s_fpAccUpTmp = pstR->pstCurPath->fpAccUp * 1.0;//��λת��fpAccUp��λΪmm/s*s
				s_usAcUpLenth = ((UINT32)SQUARE(pstR->pstCurPath->usMaxV) - (UINT32)SQUARE(pstR->pstCurPath->usStartV)) / s_fpAccUpTmp / 2.0;	//���ۼ��پ���
			}
			else
			{
				s_usAcUpLenth = 0;
			}
			
			if(pstR->pstCurPath->usEndV < pstR->pstCurPath->usMaxV)	//�ж��Ƿ���Ҫ�м���·��
			{
				s_fpAccDownTmp = pstR->pstCurPath->fpAccDown * 1.0;
				s_usAcDownLenth = ((UINT32)SQUARE(pstR->pstCurPath->usMaxV) - (UINT32)SQUARE(pstR->pstCurPath->usEndV)) / s_fpAccDownTmp/2.0;	//���ۼ��پ���
			}
			else
			{
				s_usAcDownLenth = 0;
			}
			
			ssAcLenthTmp = s_usAcUpLenth + s_usAcDownLenth;	//�����ܼӼ��پ���
			
			if(ssAcLenthTmp > fpPath_Dis)	//�Ӽ��پ��볬�����г̣����¼�������ٶ�
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
		
	if (pstR->pstCurPath->ucMode == 0)	//����������У�·��ȫ��ִ�г�ʼ�ٶ�
	{
		pstR->stVeltDes.fpVy = pstR->pstCurPath->usStartV;
	}
	else if(pstR->pstCurPath->ucMode == 1)	//����Ӽ�������
	{
		if(fpPath_Dis - fpPath_Dis_E < s_usAcUpLenth)	//������̴��ڼ��������
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
		else if(fpPath_Dis - fpPath_Dis_E >= s_usAcUpLenth && fpPath_Dis_E >= s_usAcDownLenth)	//������̴������������
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
������: void DistributeSpeed(ST_POSITION *pstDPos,ST_SPEED *pstTargetSpeed,FP32 fpAngle)
��������: ���ݳ���Ŀ���ٶȺ�Ŀ��ת�٣�Ϊÿ�����ӷ����ٶ�
����:  *pstR
���: ��
��ע: 
************************************************************************************************************************/
void NavLine(ST_OMNI_MOBILE_ROBOT *pstR)
{   
	ST_VECTOR stVecPE,stVecSE;
	FP32 fpAngLocal;
	FP32 fpAngCourse;		//����ǣ���λ��RAD����

	FP32 fpPathDis;//·���ܳ�

	fpAngCourse = ((FP32)(pstR->stPot.ssPosQ)) * RADIAN_10;
    
	/*����·�����ָ���յ������SE*/
	stVecSE.ssVx = pstR->pstCurPath->ssEndX - pstR->pstCurPath->ssStartX;
	stVecSE.ssVy = pstR->pstCurPath->ssEndY - pstR->pstCurPath->ssStartY;
	/*��������˵�ǰλ��ָ��·���յ������PE*/
	stVecPE.ssVx = pstR->pstCurPath->ssEndX - pstR->stPot.ssPosX;
	stVecPE.ssVy = pstR->pstCurPath->ssEndY - pstR->stPot.ssPosY;

	/*����·��ֱ��������˼н�*/
	fpAngLocal = ConvertAngle(fpAngCourse - CalAngle(stVecSE));
	
	/*������תPID*/
	pstR->stNavPidLine.stPidRot.fpE = ConvertDeg(pstR->pstCurPath->ssEndQ - pstR->stPot.ssPosQ);	//��ƫ��Ƕ�ת��[-1800,1800)(0.1��)��Χ��
	CalPIDWTCOL(&pstR->stNavPidLine.stPidRot);
	pstR->stVeltDes.fpW = Round(pstR->stNavPidLine.stPidRot.fpU);
	pstR->stVeltDes.fpW = Clip(pstR->stVeltDes.fpW, -pstR->stVeltLimit.fpW, pstR->stVeltLimit.fpW);	//������ת�Ľ��ٶ�
	
	/*�������PID*/
	pstR->stNavPidLine.stPidTrvs.fpE = CalNormalProjection(stVecPE,stVecSE);
	CalPIDWTCOL(&pstR->stNavPidLine.stPidTrvs);
	pstR->stVeltDes.fpVx = pstR->stNavPidLine.stPidTrvs.fpU;
	pstR->stVeltDes.fpVx = Clip(pstR->stVeltDes.fpVx, -pstR->stVeltLimit.fpVx, pstR->stVeltLimit.fpVx);	//���ƺ����ƶ��ٶ�
	
	/*��������PID*/
	pstR->stNavPidLine.stPidVtc.fpE = CalRadialProjection(stVecPE,stVecSE);
	fpPathDis = CalRadialProjection(stVecSE, stVecSE);

	if(pstR->emPathState == PATH_END && (pstR->pstCurPath->scOver & 0x10))//����ʱλ�ñջ�
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
	
	/*��������ٶ�*/
	DistributeSpeed(pstR,fpAngLocal);

	CheckSeries(pstR);
	pstR->pstPrePath = pstR->pstCurPath;//������·����ָ�븳����һ��
}


void NavLineEx1(ST_OMNI_MOBILE_ROBOT *pstR)
{
	ST_VECTOR stVecPE,stVecSE;
	FP32 fpAngLocal;
	FP32 fpAngCourse;		//����ǣ���λ��RAD����
       FP32 fpDis_E;
	FP32 fpPathDis;//·���ܳ�
	FP32 fpTmpQDes;
	FP32 fpCurRatio;

	fpAngCourse = ((FP32)(pstR->stPot.ssPosQ)) * RADIAN_10;
    
	/*����·�����ָ���յ������SE*/
	stVecSE.ssVx = pstR->pstCurPath->ssEndX - pstR->pstCurPath->ssStartX;
	stVecSE.ssVy = pstR->pstCurPath->ssEndY - pstR->pstCurPath->ssStartY;
	/*��������˵�ǰλ��ָ��·���յ������PE*/
	stVecPE.ssVx = pstR->pstCurPath->ssEndX - pstR->stPot.ssPosX;
	stVecPE.ssVy = pstR->pstCurPath->ssEndY - pstR->stPot.ssPosY;

	/*����·��ֱ��������˼н�*/
	fpAngLocal = ConvertAngle(fpAngCourse - CalAngle(stVecSE));
	
	
	
	/*�������PID*/
	/*x�����ϵ����Ӧ����mm��λ*/
	pstR->stNavPidLine.stPidTrvs.fpE = CalNormalProjection(stVecPE,stVecSE);
	CalPosPID(&pstR->stNavPidLine.stPidTrvs);
	pstR->stVeltDes.fpVx = pstR->stNavPidLine.stPidTrvs.fpU;
	pstR->stVeltDes.fpVx = Clip(pstR->stVeltDes.fpVx, -pstR->stVeltLimit.fpVx, pstR->stVeltLimit.fpVx);	//���ƺ����ƶ��ٶ�
	
	/*��������PID*/
	/*Y�����ϵ����Ӧ����mm��λ*/
	pstR->stNavPidLine.stPidVtc.fpE = CalRadialProjection(stVecPE,stVecSE);
       fpDis_E = pstR->stNavPidLine.stPidVtc.fpE;
	fpPathDis = CalRadialProjection(stVecSE, stVecSE);

	if(pstR->emPathState == PATH_END && (pstR->pstCurPath->scOver & 0x10))//����ʱλ�ñջ�
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

      
	/*������תPID*/
	 if(fpDis_E >= ((0.5 + pstR->pstCurPath->fpDisRatio/2) *fpPathDis))
	 {
	 	// ������ʼֵ
	 	pstR->stNavPidLine.stPidRot.fpE  = ConvertDeg(pstR->pstCurPath->ssStartQ - pstR->stPot.ssPosQ);
	 }
	 else if(fpDis_E <= ((0.5 - pstR->pstCurPath->fpDisRatio/2) *fpPathDis))
	 {
	 	// ������ֵֹ
	 	pstR->stNavPidLine.stPidRot.fpE = ConvertDeg(pstR->pstCurPath->ssEndQ - pstR->stPot.ssPosQ);	//��ƫ��Ƕ�ת��[-1800,1800)(0.1��)��Χ��
	 }
	 else
	 {
	       //����Y ���������
	       fpCurRatio = (fpPathDis - fpDis_E  -  (0.5 - pstR->pstCurPath->fpDisRatio/2) *fpPathDis)/( fpPathDis* pstR->pstCurPath->fpDisRatio);
	       fpTmpQDes =( pstR->pstCurPath->ssEndQ - pstR->pstCurPath->ssStartQ )*(fpCurRatio ) +  pstR->pstCurPath->ssStartQ;
	       pstR->stPotDes.ssPosQ =(SSHORT16) fpTmpQDes;
	       pstR->stNavPidLine.stPidRot.fpE = ConvertDeg(pstR->stPotDes.ssPosQ- pstR->stPot.ssPosQ);
	 }
	//CalPIDIS(&pstR->stNavPidLine.stPidRot);
	CalPosPID(&pstR->stNavPidLine.stPidRot);
	pstR->stVeltDes.fpW = Round(pstR->stNavPidLine.stPidRot.fpU);
	pstR->stVeltDes.fpW = Clip(pstR->stVeltDes.fpW, -pstR->stVeltLimit.fpW, pstR->stVeltLimit.fpW);	//������ת�Ľ��ٶ�
	
	/*��������ٶ�*/
	//DistributeSpeed(pstR,fpAngLocal);
	AutoBaseVeltAllocate(pstR, &(pstR->stVeltDes),-fpAngLocal/RADIAN_10);

	CheckSeries(pstR);
	pstR->pstPrePath = pstR->pstCurPath;//������·����ָ�븳����һ��
}


/****************************************************************************************************
�������ܣ�Բ��PID������ʹ������һ���ĺ���ǻ���Բ���켣���򱣳�һ���н���Բ���켣�˶�
��ڲ�����pos�������˵�ǰλ��
		  pid_cir��Բ������PID�������ݽṹ��
		  tpath����ǰ·��
		  tspeed��Ҫ�������ʱĿ���ٶ�
���ڲ�����1����ʾ�Ѿ���������·���յ�
		  0����ʾδ��������·���յ�
����˵������������������룬ֱ�ӿ������������⣬�������Ǻ�����ʹ�ã��Ӷ������ִ��Ч��
		  385us���ң�һ�������Ǻ����򿪷�����ռ��ʱ��Ϊ30-40us,��һ��Ҫ����ʹ��
****************************************************************************************************/
void NavCircle(ST_OMNI_MOBILE_ROBOT* pstR)
{   
//	static UCHAR8 ucPathUpdateFlag = TRUE;//·�����±�־
	ST_VECTOR	stVecCP, stVecCE, stVecCS;
	FP32 fpAngCourse, fpAngCP, fpAngCE, fpAngCS,fpAngPN, fpDisCP, fpAngLocal;
	FP32 fpPathDis;//·���ܳ�
	
	fpAngCourse =(FP32)pstR->stPot.ssPosQ * RADIAN_10;		//�������ת��Ϊ����
	//----------��������˵�ǰλ��ָ��Բ��Բ�ĵ�����----------
	stVecCP.ssVx = pstR->stPot.ssPosX - pstR->pstCurPath->ssCenterX;
	stVecCP.ssVy = pstR->stPot.ssPosY - pstR->pstCurPath->ssCenterY;
	//----------�������������λ��ָ��Բ��Բ�ĵ�����----------
	stVecCE.ssVx = pstR->pstCurPath->ssEndX - pstR->pstCurPath->ssCenterX;
	stVecCE.ssVy = pstR->pstCurPath->ssEndY - pstR->pstCurPath->ssCenterY;
	//----------�����������ʼλ��ָ��Բ��Բ�ĵ�����----------
	stVecCS.ssVx = pstR->pstCurPath->ssStartX - pstR->pstCurPath->ssCenterX;
	stVecCS.ssVy = pstR->pstCurPath->ssStartY - pstR->pstCurPath->ssCenterY;
	
	fpAngCP = CalAngle(stVecCP);		//����CP�����н�
	fpAngCE = CalAngle(stVecCE);        //����CE�����н�
	fpAngCS = CalAngle(stVecCS);        //����CS�����н�
	//����PN�����нǣ���Բ������������
//	fpAngPN = ConvertAngle(pstR->pstCurPath->ssRadius > 0 ? fpAngCP - PI_2 : fpAngCP + PI_2);//ssRadius > 0˳Բ
	if(pstR->pstCurPath->ssRadius > 0)		//˳Բ
	{
		fpAngPN = ConvertAngle(fpAngCP - PI_2);
	}
	else 				//��Բ
	{
		fpAngPN = ConvertAngle(fpAngCP + PI_2);
	}
	
	//----------����Ƕ�PID----------
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
	//������ת�Ľ��ٶ�
	pstR->stVeltDes.fpW = Clip(pstR->stVeltDes.fpW, -pstR->stVeltLimit.fpW, pstR->stVeltLimit.fpW);
	//----------���㾶��PID----------
	fpDisCP = sqrt((SINT32)SQUARE(stVecCP.ssVx) + (SINT32)SQUARE(stVecCP.ssVy));/////�˴������⣬��������ᷢ�����
	//����ƫ��
	
	if(pstR->pstCurPath->ssRadius > 0)	//˳Բ
	{
		pstR->stNavPidCir.stPidTrvs.fpE = fpDisCP - pstR->pstCurPath->ssRadius;
	}
	else 			//��Բ
	{
		pstR->stNavPidCir.stPidTrvs.fpE = -pstR->pstCurPath->ssRadius - fpDisCP;//ssRadius������
	}

	CalPIDWTCOL(&pstR->stNavPidCir.stPidTrvs);
		
	pstR->stVeltDes.fpVx = Clip(pstR->stNavPidCir.stPidTrvs.fpU, -pstR->stVeltLimit.fpVx, pstR->stVeltLimit.fpVx);
	
	//----------��������PID----------
	
	//----------������pi��-pi���Ƕȳ���ͻ�䣬�ʼ������¶γ���----//
	if(pstR->pstCurPath->ssRadius > 0)//˳Բ
	{
		if(fpAngCS < fpAngCE)//·�����pi
		{
			fpAngCS += PI2;
			if(fpAngCP < fpAngCE && fpAngCP + PI2 >= fpAngCE && fpAngCP + PI2 <= fpAngCS)//��·����
			{
				fpAngCP += PI2;
			}
			else if(fpAngCP < fpAngCE)
			{
			
			}
		}
		else if(fpAngCS > fpAngCE)//δ��Խpi
		{
			if(fpAngCP > fpAngCS)
			{
				fpAngCP -= PI2;
			}
		}
	}
	else if(pstR->pstCurPath->ssRadius < 0)//��Բ
	{
		if(fpAngCS > fpAngCE)//·�����pi
		{
			fpAngCS -= PI2;
			if(fpAngCP > fpAngCE && fpAngCP - PI2 <= fpAngCE && fpAngCP - PI2 >= fpAngCS)
			{
				fpAngCP -= PI2;
			}
		}
		else if(fpAngCS < fpAngCE)//δ���pi
		{
			if(fpAngCP < fpAngCS)
			{
				fpAngCP += PI2;
			}
		}
	}
	
	//---------------------------------------------------------//
	pstR->stNavPidCir.stPidVtc.fpE = (fpAngCP - fpAngCE) * pstR->pstCurPath->ssRadius;		//��������յ�Ļ�����ע��˴���R�Ǵ����ŵ�
	fpPathDis = (fpAngCS - fpAngCE) * pstR->pstCurPath->ssRadius;//·�����ܳ���
	if(pstR->emPathState == PATH_END && (pstR->pstCurPath->scOver & 0x10))//����ʱλ�ñջ�
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
	
	//----------��������ٶ�----------//
	fpAngLocal = ConvertAngle(fpAngCourse - fpAngPN);		//Բ������������˼н�
	DistributeSpeed(pstR, fpAngLocal);

	CheckSeries(pstR);
	pstR->pstPrePath = pstR->pstCurPath;//������·����ָ�븳����һ��
}



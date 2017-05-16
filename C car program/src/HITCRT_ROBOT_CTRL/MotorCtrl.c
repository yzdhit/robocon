/*****************************************************************************************************************************
��Ȩ������HITCRT(�����󾺼�������Э��)
�ļ�����MotorCtrl.c
����޸����ڣ�2010.07.10
�汾��1.0
-----------------------------------------------------------------------------------------------------------------------------
ģ���������������ģ��
�����б� 
1. FP32 CalVelt(ST_VELT_CODER * pstVeltCoder, UCHAR8 ucCoderChan, FP32 fpCoffC2V)		����ٶȼ��㺯��
2. void VeltLoopCtrl (ST_MOTOR_CTRL * pstRobotM)									����ٶȱջ����ƺ���
3. void PotLoopCtrl (ST_MOTOR_CTRL *pstRobotM)										���λ�ñջ����ƺ���
------------------------------------------------------------------------------------------------------------------------------
�޸ļ�¼��
����        ʱ��            �汾     ˵��
��ΰ        2010.07.10      1.0      	 �������ļ������Ʒ���ԭ������ǰ�ķ���һ������ǿ�˺����ķ�װ��
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
��������:CopyFloatSlowly(FP32 * pfpDest, FP32 *pfpSrc, FP32 fpCopyStep)
��������: ������������ֹ�ٶȵȱ�����ͻ��
�������: pfpDest Ŀ�����ݵ�ָ��
			   pfpSrcԴ���ݵ�ָ��
			   ÿ�ο����Ĳ���
�������:��
��ע: ��Ҫ�������ò�����ȫcopy��ɡ�����õ������ݲ���
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
��������:CalVelt()
��������: �������̹����������Ӧ��ʱ������ٶ�
�������: pstVeltCoder�������ٶȵ����̽ṹ��
			   cuCoderchan��ʹ�õ�����ͨ��
			   fpCoffC2V��ʵ���ٶ������̹����ٶȵ�ת��ϵ��
�������: �����ٶ�
��ע: �ú����Ǹ���T���������̷����ٶ�
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
		pstRobotM->stVeltCoder.siCurCodeNum = READ_CODER(pstRobotM->ucCoderChan);		//��ȡ��ǰ���̶���
		pstRobotM->stVeltCoder.siDetCodeNum = pstRobotM->stVeltCoder.siCurCodeNum - pstRobotM->stVeltCoder.siPreCodeNum;	//��������������Ŀ
		pstRobotM->stVeltCoder.siPreCodeNum = pstRobotM->stVeltCoder.siCurCodeNum;		//�����������̶���
		
		pstRobotM->stVeltCoder.stIntlTime.uiCurTime = ((UINT32)TIM3->CNT << 16) | TIM2->CNT;		//��ȡ��ǰʱ��
		pstRobotM->stVeltCoder.stIntlTime.uiIntlTime = pstRobotM->stVeltCoder.stIntlTime.uiCurTime - pstRobotM->stVeltCoder.stIntlTime.uiPreTime;	//����ʱ����
		pstRobotM->stVeltCoder.stIntlTime.uiPreTime = pstRobotM->stVeltCoder.stIntlTime.uiCurTime;	//��������ʱ��

		return (pstRobotM->fpCoffVelt * pstRobotM->stVeltCoder.siDetCodeNum / pstRobotM->stVeltCoder.stIntlTime.uiIntlTime);
	}
}

/****************************************************************************************************
��������:VeltLoopCtrl()
��������: �Ե�������ٶȱջ�����
�������:  pstRobotM��Ҫ���ƵĻ������ϵĵ��
��ע: 
****************************************************************************************************/
void VeltLoopCtrl (ST_MOTOR_CTRL * pstRobotM)
{
	pstRobotM->stVeltPid.fpE = pstRobotM->fpVeltDes - pstRobotM->fpVeltFB;		//����ƫ��
	pstRobotM->stVeltPid.fpE = ClipFloat(pstRobotM->stVeltPid.fpE,-pstRobotM->stVeltPid.fpEthreshold, pstRobotM->stVeltPid.fpEthreshold);
	CalPIDIS (&pstRobotM->stVeltPid);		//���л��ַ���ʽPID����

	pstRobotM->ssPwmDuty = Clip (Round (pstRobotM->stVeltPid.fpU), -pstRobotM->usMaxPwmDuty, pstRobotM->usMaxPwmDuty);	//����PWM

	pstRobotM->TurnMotor (pstRobotM->ssPwmDuty);		//����������źţ�����ռ�ձȺͷ���
}

/****************************************************************************************************
��������:PotLoopCtrl()  
��������: �Ե������λ�ñջ����ƣ�λ�÷���������Ϊ����
�������:  pstRobotM��Ҫ���ƵĻ����˵ĵ��
��ע: 
****************************************************************************************************/
void PotLoopCtrl (ST_MOTOR_CTRL *pstRobotM)
{
	pstRobotM->fpPotFB = pstRobotM->fpCoffPot * READ_CODER(pstRobotM->ucCoderChan);		//���㷴��λ��
	
	pstRobotM->stPotPid.fpE = pstRobotM->fpPotDes - pstRobotM->fpPotFB;		//����ƫ��

	CalPIDWTCOL(&pstRobotM->stPotPid);			//����PID

	pstRobotM->ssPwmDuty = Clip (Round (pstRobotM->stPotPid.fpU), -pstRobotM->usMaxPwmDuty, pstRobotM->usMaxPwmDuty);	//����PWM

	pstRobotM->TurnMotor (pstRobotM->ssPwmDuty);		//����������źţ�����ռ�ձȺͷ���
}

/****************************************************************************************************
��������:OpenLoopCtrl()
��������: �����������
�������:  pstRobotM��Ҫ���Ƶĵ��
��ע: 
****************************************************************************************************/
void OpenLoopCtrl(ST_MOTOR_CTRL *pstRobotM)
{
	pstRobotM->TurnMotor(pstRobotM->ssPwmDuty);
}




void AutoVeltDis(ST_MOTOR_CTRL *pstRobotM)
{	   
	static SSHORT16 s_ssPathMaxV[AMQ];		//·������ٶ�
	static SINT32 s_siAcUpLenth[AMQ];	//�����������
	static SINT32 s_siAcDownLenth[AMQ];	//�����������
	static SINT32 s_siLenth[AMQ];//�ܵ�·��
	SINT32 siAcLenthTmp, siCurCode;
	static FP32 s_fpAccUpTmp[AMQ], s_fpAccDownTmp[AMQ];
	/*����·��*/
	if (pstRobotM->pstCurPath != pstRobotM->pstPrePath)                                       
	{	
		if(pstRobotM->pstCurPath->ssStartV < pstRobotM->pstCurPath->ssMaxV)	//�ж��Ƿ���Ҫ�м���·��
		{	
			s_fpAccUpTmp[pstRobotM->ucMotorChan] = pstRobotM->pstCurPath->fpAccUp * 1.0;//��λת��fpAccUp 
			s_siAcUpLenth[pstRobotM->ucMotorChan] = (SQUARE(pstRobotM->pstCurPath->ssMaxV) - SQUARE(pstRobotM->pstCurPath->ssStartV)) / s_fpAccUpTmp[pstRobotM->ucMotorChan] / 2.0;	//���ۼ��پ���
		}
		else
		{
			s_siAcUpLenth[pstRobotM->ucMotorChan] = 0;
		}
			
		if(pstRobotM->pstCurPath->ssEndV < pstRobotM->pstCurPath->ssMaxV)	//�ж��Ƿ���Ҫ�м���·��
		{
			s_fpAccDownTmp[pstRobotM->ucMotorChan] = pstRobotM->pstCurPath->fpAccDown * 1.0;//a
			s_siAcDownLenth[pstRobotM->ucMotorChan] = (SQUARE(pstRobotM->pstCurPath->ssMaxV) - SQUARE(pstRobotM->pstCurPath->ssEndV)) / s_fpAccDownTmp[pstRobotM->ucMotorChan] / 2.0;	//���ۼ��پ���
		}
		else
		{
			s_siAcDownLenth[pstRobotM->ucMotorChan] = 0;
		}
		
		siAcLenthTmp = s_siAcUpLenth[pstRobotM->ucMotorChan] + s_siAcDownLenth[pstRobotM->ucMotorChan];	//�����ܼӼ��پ���
		s_siLenth[pstRobotM->ucMotorChan] = abs(pstRobotM->pstCurPath->siEndCode - pstRobotM->pstCurPath->siStartCode);
			
		if(siAcLenthTmp > s_siLenth[pstRobotM->ucMotorChan])	//�Ӽ��پ��볬�����г̣����¼�������ٶ�
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
	
	siCurCode = READ_CODER(pstRobotM->ucCoderChan);//��ȡ��ǰλ������ֵ
	
	if(pstRobotM->pstCurPath->scOver & (1 << 0))	//������̼�����������
	{
		if(siCurCode - pstRobotM->pstCurPath->siStartCode < s_siAcUpLenth[pstRobotM->ucMotorChan])	//������̴��ڼ��������
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
		else if(siCurCode - pstRobotM->pstCurPath->siStartCode >= s_siAcUpLenth[pstRobotM->ucMotorChan] && pstRobotM->pstCurPath->siEndCode - siCurCode >= s_siAcDownLenth[pstRobotM->ucMotorChan])	//������̴������������
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
	else//������̼�����������
	{
		if(siCurCode - pstRobotM->pstCurPath->siStartCode > -s_siAcUpLenth[pstRobotM->ucMotorChan])	//������̴��ڼ��������
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
		else if(siCurCode - pstRobotM->pstCurPath->siStartCode <= -s_siAcUpLenth[pstRobotM->ucMotorChan] && pstRobotM->pstCurPath->siEndCode - siCurCode <= -s_siAcDownLenth[pstRobotM->ucMotorChan])	//������̴������������
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
��������:AutoPotLoopCtrl(ST_MOTOR_CTRL *pstRobotM) 
��������: ���λ���Զ����ƣ������ٶȹ滮��λ�ñջ��������������ƣ�·���л��ȹ���
�������:  pstRobotM��Ҫ���ƵĻ����˵ĵ��
��ע: 
****************************************************************************************************/
void AutoPotLoopCtrl(ST_MOTOR_CTRL *pstRobotM)
{
	if(pstRobotM->emPathState == PATH_RUN)//·������
	{
		AutoVeltDis(pstRobotM);//�Զ������ٶ�
		pstRobotM->fpVeltDes = Clip(pstRobotM->fpVeltDes, -pstRobotM->fpMaxVelt, pstRobotM->fpMaxVelt);//���ٶȽ�������
		VeltLoopCtrl(pstRobotM);//�ٶȱջ�����  
		//·�������ж�
		if(pstRobotM->pstCurPath->scOver & (1 << 4))//�ж��Ƿ����ʱλ�ñջ�,��ǰ�ж�
		{
			if ((pstRobotM->pstCurPath->scOver & (1 << 0)) && pstRobotM->stVeltCoder.siCurCodeNum >= pstRobotM->pstCurPath->siEndCode - pstRobotM->fpPotSen)   //��ǰ����ֵ�����յ�����ֵʱ����·��
			{
				pstRobotM->emPathState = PATH_END;
			}
			else if((!(pstRobotM->pstCurPath->scOver & (1 << 0))) && pstRobotM->stVeltCoder.siCurCodeNum <= pstRobotM->pstCurPath->siEndCode + pstRobotM->fpPotSen  )//��ǰ����ֵС���յ�����ֵʱ����·��
			{
				pstRobotM->emPathState = PATH_END;
			}
		}
		else//����ʱ����Ҫλ�ñջ�
		{
			if ((pstRobotM->pstCurPath->scOver & (1 << 0)) && pstRobotM->stVeltCoder.siCurCodeNum >= pstRobotM->pstCurPath->siEndCode)   //��ǰ����ֵ�����յ�����ֵʱ����·��
			{
				pstRobotM->emPathState = PATH_END;
			}
			else if((!(pstRobotM->pstCurPath->scOver & (1 << 0))) && pstRobotM->stVeltCoder.siCurCodeNum <= pstRobotM->pstCurPath->siEndCode)//��ǰ����ֵС���յ�����ֵʱ����·��
			{
				pstRobotM->emPathState = PATH_END;
			}
		}
	}
	else
	{
		if(pstRobotM->pstCurPath->scOver & (1 << 4))//�ж��Ƿ����ʱλ�ñջ�
		{
			pstRobotM->fpPotDes = pstRobotM->fpCoffPot * pstRobotM->pstCurPath->siEndCode;
			PotLoopCtrl(pstRobotM);
		}
		else if(pstRobotM->pstCurPath->scOver & (1 << 5))//�ж��Ƿ����ʱPWMDuty=0
		{
			pstRobotM->ssPwmDuty = 0;
			pstRobotM->TurnMotor(pstRobotM->ssPwmDuty);
		}
		else if(pstRobotM->pstCurPath->scOver & (1 << 6))//�ж��Ƿ����ʱPWMDuty=EndPWMDuty
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
	//λ�û�
	pstMotor->fpPotFB = pstMotor->fpCoffPot * READ_CODER(pstMotor->ucCoderChan);
	pstMotor->stPotPid.fpE = pstMotor->fpPotDes - pstMotor->fpPotFB;		//����ƫ��
	pstMotor->stPotPid.fpE = ClipFloat(pstMotor->stPotPid.fpE, -pstMotor->stPotPid.fpEthreshold,pstMotor->stPotPid.fpEthreshold);
	CalPosPID(&pstMotor->stPotPid);

	//�ٶȻ�
	pstMotor->fpVeltDes = pstMotor->stPotPid.fpU;
	pstMotor->fpVeltFB = CalVelt(pstMotor);
	VeltLoopCtrl(pstMotor);	
}


/****************************************************************************************************
��������:DiffAutoVeltDis(ST_MOTOR_CTRL *pstMotor)
��������: ��Ӧ���ٵ��̵ĵ���ٶȷ��䣬��Ҫ�ǰ��սǶȷ��䣬�����ǰ����������
�������:  pstMotor��ִ�е����ָ��
��ע: ע���Ǵ�0-  AMQ-1
****************************************************************************************************/
void DiffAutoVeltDis(ST_MOTOR_CTRL *pstMotor)
{
	static FP32 s_fpPathMaxV[AMQ];  //ʵ��Ϊ���ٶȣ���λΪr/min
	static FP32 s_fpAccUpLen[AMQ];  //ʵ��Ϊ�Ƕȣ���λΪ��,���ٶ�Ϊrad/s^2
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
	//��ӡ����·���ͼ���·����
	/*
	PutCur(3,0);
	PrintFloat(s_fpAccUpLen[0],2,2);
	PrintChar("  ");
	PrintFloat(s_fpAccDownLen[0],2,2);*/
	

	siCoderNum = READ_CODER(pstMotor->ucCoderChan);
	pstMotor->fpPotFB = pstMotor->fpCoffPot *siCoderNum; //�Ƕȵ�����
//�ñ�־�����Ƽ��㷽��̫����Ӧ�ø��ݽ���ʱ����ʼʱ
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
��������: DiffAutoPotLoopCtrl(ST_MOTOR_CTRL * pstMotor)
��������: ���λ���Զ����ƣ��ٶȹ滮��λ�ñջ�������������·���л�
�������:  pstMotor��Ҫ���ƵĻ����˵ĵ��
��ע: 
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
		pstMotor->fpPotFB = pstMotor->fpCoffPot *siCoderNum; //�Ƕȵ�����
		
		//·���жϽ���
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
			//·������ʱ��������PID
			pstMotor->fpVeltFB = CalVelt(pstMotor);
		       pstMotor->fpVeltDes =0;
		       VeltLoopCtrl(pstMotor);
		}
	}
}


//�����λ��ֵ����Ϊ�Ƕ�ֵ
//�������õ�ǰ���·���Ľ���ʱ��λ��,ʹ�øú���ʱ��
//��ȷ�Ͻ���λ�ú���ʼλ�ò�ͬ�����������ܺ�����
void SetPathEndPosOnly(ST_MOTOR_CTRL *pMotor, ST_ACTION_PATH * pPath,SINT32 siEndPos)
{
	pPath->siEndCode = siEndPos;
	
	//��ֹ��ʼ��ͽ������غ�
	if(abs(pPath->siEndCode - pPath->siStartCode)<1)
	{
		pPath->siEndCode = pPath->siStartCode + 1;
	}
	
	if(pPath->siEndCode >= pPath->siStartCode)//��ΪscOverΪ��������
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


//���õ��·���Ľ���ֵ��ͬʱ���������ǰλ��ֵ��Ϊ��ʼֵ
//��ȷ�Ͻ���λ�ú���ʼλ�ò�ͬ�����������ܺ�����
void SetPathEndPosEx(ST_MOTOR_CTRL *pMotor,ST_ACTION_PATH * pPath,FP32 siEndPos)
{
       pPath->siStartCode = pMotor->fpCoffPot * READ_CODER(pMotor->ucCoderChan);
	pPath->siEndCode = siEndPos;
	//��ֹ��ʼ��ͽ������غ�
	if(abs(pPath->siEndCode - pPath->siStartCode)<1)
	{
		pPath->siEndCode = pPath->siStartCode + 1;
	}
	
	if(pPath->siEndCode >= pPath->siStartCode)//��ΪscOverΪ��������
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
	//��ֹ��ʼ��ͽ������غ�
	if(abs(pPath->siEndCode - pPath->siStartCode)<1)
	{
		pPath->siEndCode = pPath->siStartCode + 1;
	}
	
	if(pPath->siEndCode >= pPath->siStartCode)//��ΪscOverΪ��������
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




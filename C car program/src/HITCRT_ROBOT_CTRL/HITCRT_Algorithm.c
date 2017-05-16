/*******************************************************************
��Ȩ������HITCRT(�����󾺼�������Э��)
�ļ�����HITCRT_Algorithm.c
����޸����ڣ�2010.11.29
�汾��
--------------------------------------------------------------------
ģ���������������㺯��
�����б� 
1.Clip()                  ����������ȥ���������ֵ����Сֵ֮���ֵ����֮��������Сֵ  
2.Round()                 ���������������룬����32λ������
3.CalPIDIS()              ����PID��  ���ַ���ʽPID���ɷ�ֹ���󳬵���
4.CalPIDWTCOL()           ����PID��  ��������ʽPID���ɷ�ֹPID���������ڱ���
5.NoCalPIDWTCOL()         ����PID��  ����������ʽPID 
6.CalNormalProjection()   ����һ�����ڻ�׼����������ͶӰ
7.CalRadialProjection()   ����һ�����ڻ�׼��������ͶӰ
8.CalAngle()              ����һ������ssPosX��������н�
9.ConvertAngle()          ���Ƕ�ת��Ϊȫ������ϵ�ĺ���Ƿ�Χ[-PI,PI)
10.ConvertDeg()           ���Ƕ�ת��Ϊȫ������ϵ�ĺ���Ƿ�Χ[-1800,1800)(��λ��0.1��)
11.CalAverage()           ��һ�����ƽ��ֵ
12.AvoidDoublePress()     ��ֹ��������
13.CalPIDUni()			  ��һ����������
--------------------------------------------------------------------
�޸ļ�¼��
����        ʱ��            �汾     ˵��
��ΰ        2010.3.5        1.0      �������ļ�
ղ����      2010.11.29      2.0      �淶�����ļ�
********************************************************************/

#include "HITCRT_RobotTypes.h"
#include "HITCRT_Coff.h"
#include "HITCRT_API.h"
#include "math.h"
/*******************************************************************
�������ƣ�Clip()
�������ܣ�����������ȥ���������ֵ����Сֵ֮���ֵ����֮��������Сֵ
���룺    siValue:ʵ��ֵ
			siMin:����ֵ
			siMax:����ֵ
�����   �������ֵ
��ע��
********************************************************************/
SINT32 Clip(SINT32 siValue, SINT32 siMin, SINT32 siMax)
{
	if(siValue < siMin)
	{
		return siMin;
	}
	else if(siValue > siMax)
	{
		return siMax;
	}
	else 
	{
		return siValue;
	}
}

FP32	 ClipFloat(FP32 fpValue, FP32 fpMin, FP32 fpMax)
{
	if(fpValue < fpMin)
	{
		return fpMin;
	}
	else if(fpValue > fpMax)
	{
		return fpMax;
	}
	else 
	{
		return fpValue;
	}
	return fpValue;
}

/*******************************************************************
�������ƣ�Round()
�������ܣ����������������룬����32λ������
���룺    fpValue
�����    ��������󷵻ص�������
��ע��
********************************************************************/
SINT32 Round(FP32 fpValue)
{   
    if (fpValue >= 0)
    {
    	return (SINT32)(fpValue + 0.5);
    }
    else 
    {
    	return (SINT32)(fpValue - 0.5);
    }
}

/*******************************************************************
�������ƣ�CalPIDIS()
�������ܣ�����PID��
          ������PID�㷨�ļ��㹫ʽ��detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+D*det(E(k))+det(E(k)))
          ���ַ���ʽPID���ɷ�ֹ���󳬵���
���룺    pstPid:��Ҫ��PID������ٶ�PID�ṹ��ָ��
�����    ��
��ע��
********************************************************************/
void CalPIDIS(volatile ST_PID *pstPid)	//���ַ���ʽPID�㷨(IntegralSeparated)
{   
    UCHAR8  ucK;
    
	if (fabs(pstPid->fpE) > pstPid->fpELimit)
	{
		ucK=0; 
	}
	else 
	{
		ucK=1;
	}
	pstPid->fpU += pstPid->fpP * (pstPid->fpE - pstPid->fpPreE) + ucK * pstPid->fpI * pstPid->fpE + pstPid->fpD * (pstPid->fpE - 2 * pstPid->fpPreE + pstPid->fpPrePreE);
	pstPid->fpPrePreE = pstPid->fpPreE;
	pstPid->fpPreE=pstPid->fpE;
}

/*******************************************************************
�������ƣ�CalPIDWTCOL() 
�������ܣ�����PID��
          ������PID�㷨�ļ��㹫ʽ��detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+fpD*det(E(k))+det(E(k)))
          ��������ʽPID���ɷ�ֹPID���������ڱ���
���룺    pstPid:��Ҫ��PID������ٶ�PID�ṹ��ָ��
�����    ��
��ע��
********************************************************************/
void CalPIDWTCOL(volatile ST_PID *pstPid)    //��������ʽPID�㷨��Weaken The Case Of Limited��
{   
    UCHAR8 ucK;
    
	 
	if (((pstPid->fpU > pstPid->fpULimit) && (pstPid->fpE > 0)) || ((pstPid->fpU < -pstPid->fpULimit) && (pstPid->fpE < 0)))//�����ͣ����ܼ�С�������ֵ��ǰ���²Ž��л���
	{
	    ucK = 0;
	}
	else
	{ 
		ucK = 1;
	}
	
	pstPid->fpU += pstPid->fpP * (pstPid->fpE - pstPid->fpPreE) + ucK * pstPid->fpI * pstPid->fpE + pstPid->fpD * (pstPid->fpE - 2 * pstPid->fpPreE + pstPid->fpPrePreE);
	
	pstPid->fpPrePreE = pstPid->fpPreE;
	
	pstPid->fpPreE = pstPid->fpE;
}

/*******************************************************************
�������ƣ�CalPIDWTCOLIS() 
�������ܣ�����PID��
          ������PID�㷨�ļ��㹫ʽ��detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+fpD*det(E(k))+det(E(k)))
          ���ַ���+��������ʽPID��϶����ŵ�
���룺    pstPid:��Ҫ��PID������ٶ�PID�ṹ��ָ��
�����    ��
��ע��
********************************************************************/
void CalPIDWTCOLIS(volatile ST_PID *pstPid)  
{   
    UCHAR8 ucK1, ucK2;
    
    
    //��������
	if (((pstPid->fpU > pstPid->fpULimit) && (pstPid->fpE > 0)) 
		|| ((pstPid->fpU < -pstPid->fpULimit) && (pstPid->fpE < 0)))//������һ�ε�pstPid���������ж��Ƿ�Ҫ�л��ֻ���
	{
	    ucK1=0;
	}
	else
	{ 
		ucK1=1;
	}
	
	//���ַ���
	if (fabs(pstPid->fpE) > pstPid->fpELimit)
	{
		ucK2=0;
	}
	else
	{ 
		ucK2=1;
	}
	
	pstPid->fpU += pstPid->fpP * (pstPid->fpE - pstPid->fpPreE) + ucK1 * ucK2 * pstPid->fpI * pstPid->fpE + pstPid->fpD * (pstPid->fpE - 2 * pstPid->fpPreE + pstPid->fpPrePreE);
	
	pstPid->fpPrePreE = pstPid->fpPreE;
	
	pstPid->fpPreE = pstPid->fpE;
	
}

/*******************************************************************
�������ƣ�NoCalPIDWTCOL()  
�������ܣ�����PID��
          ������PID�㷨�ļ��㹫ʽ��detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+fpD*det(E(k))+det(E(k)))
          ����������ʽPID
���룺    pstPid:��Ҫ��PID������ٶ�PID�ṹ��ָ��
�����    ��
��ע��
********************************************************************/
void NoCalPIDWTCOL(volatile ST_PID *pstPid)    
{   
    UCHAR8 ucK;
   
	ucK=1;
	pstPid->fpU += pstPid->fpP * (pstPid->fpE - pstPid->fpPreE) + ucK * pstPid->fpI * pstPid->fpE + pstPid->fpD * (pstPid->fpE - 2 * pstPid->fpPreE + pstPid->fpPrePreE);
	pstPid->fpPrePreE = pstPid->fpPreE;
	pstPid->fpPreE = pstPid->fpE;
}

/*******************************************************************
�������ƣ�CalPIDUni()  
�������ܣ�����PID��
          ������PID�㷨�ļ��㹫ʽ��detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+fpD*det(E(k))+det(E(k)))
          PID��һ������������ʽ
���룺    pstPid:��Ҫ��PID������ٶ�PID�ṹ��ָ��
�����    ��
��ע��	  �����ã�ֻ����������Kp��ϵ������ȷ���ο�����鼮
********************************************************************/
void CalPIDUni(volatile ST_PID *pstPid)    
{   
	pstPid->fpU += pstPid->fpP * (11.0286 * pstPid->fpE - 21 * pstPid->fpPreE + 10 * pstPid->fpPrePreE);
	pstPid->fpPrePreE = pstPid->fpPreE;
	pstPid->fpPreE = pstPid->fpE;
}

/*******************************************************************
�������ƣ�CalNormalProjection()  
�������ܣ�����һ�����ڻ�׼����������ͶӰ
���룺    stAim:Ŀ������������ҪͶӰ������
		  stBase:��׼����
�����    Ŀ�������ڻ�׼�����������ϵ�ͶӰ
��ע��    ��׼�����ķ�������Ϊ��׼����˳ʱ����ת90��õ�
		  ��Ŀ����ڻ�׼ֱ�����ʱ���Ի�׼��������Ϊ�����򣩣�ͶӰֵΪ������֮Ϊ��
********************************************************************/
SSHORT16 CalNormalProjection(ST_VECTOR stAim,ST_VECTOR stBase)
{   
	return Round((FP32)(stBase.ssVy * stAim.ssVx + -stBase.ssVx * stAim.ssVy)/sqrt((FP32)(stBase.ssVy * stBase.ssVy + stBase.ssVx * stBase.ssVx)));		
}

/*******************************************************************
�������ƣ�CalRadialProjection()  
�������ܣ�����һ�����ڻ�׼��������ͶӰ
���룺    stAim:Ŀ������������ҪͶӰ������
		  stBase:��׼����
�����    Ŀ�������ڻ�׼�����ϵ�ͶӰ
��ע��    ��Ŀ����ڻ�׼ֱ��֮��ʱ��ͶӰֵΪ������֮Ϊ��
********************************************************************/
SSHORT16 CalRadialProjection(ST_VECTOR stAim, ST_VECTOR stBase)
{   
	return Round((FP32)(stBase.ssVx * stAim.ssVx + stBase.ssVy * stAim.ssVy)/sqrt((FP32)(stBase.ssVy * stBase.ssVy + stBase.ssVx * stBase.ssVx))) ;		
}

/*******************************************************************
�������ƣ�CalAngle()  
�������ܣ�����һ������ssPosY��������н�
���룺    stAim:Ŀ������
�����    Ŀ��������ssPosY��������н�(RADIAN)
��ע��    ��ʱ��Ϊ����˳ʱ��Ϊ��������нǷ�ΧΪ��[-PI,PI)
********************************************************************/
FP32 CalAngle(ST_VECTOR stAim)
{   
    FP32 fpAngAim;
	
    if(stAim.ssVy == 0)//��ĸΪ0
    {
    	if(stAim.ssVx > 0)
    	{
    		return -PI_2;
    	}
		else if(stAim.ssVx < 0)
		{
			return PI_2;
		}
		else
		{
			return 0;
		}
    }
	else
	{
		if(stAim.ssVx == 0)
		{
			if(stAim.ssVy > 0)
			{
				return 0;
			}
			else if(stAim.ssVy < 0)
			{
				return -PI;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			fpAngAim = -atan((FP32)stAim.ssVx / (FP32)stAim.ssVy);
		    if (stAim.ssVy < 0)//3��4����
		    {
		    	if (fpAngAim >= 0)//4����
		    	{
					fpAngAim -= PI;
				}
				else if (fpAngAim < 0)//3
				{
					fpAngAim += PI;
				}
			}
			return fpAngAim;
		}
	}
}

/*******************************************************************
�������ƣ�ConvertAngle()  
�������ܣ����Ƕ�ת��Ϊȫ������ϵ�ĺ���Ƿ�Χ[-PI,PI)
���룺    ang��Ŀ��Ƕ�(RADIAN)
�����    ת����ĽǶ�(RADIAN)
��ע��    ��ʱ��Ϊ����˳ʱ��Ϊ�������ʺ϶ԽǶ�ֵ�ϴ��ֵ��ת��
********************************************************************/
FP32 ConvertAngle(FP32 fpAngA)
{   
    do
    {
		if (fpAngA >= PI)
		{
			fpAngA -= PI2;
		}
		else if (fpAngA < -PI)
		{
			fpAngA += PI2;
		}
	}while (fpAngA >= PI || fpAngA < -PI);
	return fpAngA;
}
/*******************************************************************
�������ƣ�ConvertAngle_2()  
�������ܣ���[-PI,PI]�Ƕ�ת��Ϊȫ������ϵ��X������ĺ���Ƿ�Χ[0,2PI)
���룺    ang��Ŀ��Ƕ�(RADIAN)
�����    ת����ĽǶ�(RADIAN)
��ע��    ��ʱ��Ϊ��
********************************************************************/
FP32 ConverAngle_2(FP32 fpAngA)
{
	if(fpAngA >= -PI_2)
	{
		fpAngA = fpAngA + PI_2;
	}
	else
	{
		fpAngA = fpAngA + 5 * PI_2;
	}
	
	return fpAngA;
}

/*******************************************************************
�������ƣ�ConvertAngle_3()  
�������ܣ����Ƕ�ת��Ϊȫ������ϵ��X������ĺ���Ƿ�Χ[0,2PI)
���룺    ang��Ŀ��Ƕ�(RADIAN)
�����    ת����ĽǶ�(RADIAN)
��ע��    ��ʱ��Ϊ��
********************************************************************/
FP32 ConverAngle_3(FP32 fpAngA)
{
	do
    {
		if (fpAngA >= PI2)
		{
			fpAngA -= PI2;
		}
		else if (fpAngA < 0)
		{
			fpAngA += PI2;
		}
	}while (fpAngA < 0 || fpAngA > PI2);
	return fpAngA;
}
/*******************************************************************
�������ƣ�ConvertDeg()  
�������ܣ����Ƕ�ת��Ϊȫ������ϵ�ĺ���Ƿ�Χ[-1800,1800)(��λ��0.1��)
���룺    ang��Ŀ��Ƕ�(��λ��0.1��)
�����    ת����ĽǶ�(��λ��0.1��)
��ע��    ��ʱ��Ϊ����˳ʱ��Ϊ�������ʺ϶ԽǶ�ֵ�ϴ��ֵ��ת��
********************************************************************/
SSHORT16 ConvertDeg(SSHORT16 fpDegA)
{   
    do
    {
		if (fpDegA >= 1800)
		{
			fpDegA -= 3600;
		}
		else if (fpDegA < -1800)
		{
			fpDegA += 3600;
		}
	}while (fpDegA >= 1800 || fpDegA < -1800);
	return fpDegA;
}

/*******************************************************************
�������ƣ�CalAverage()  
�������ܣ���һ�����ƽ��ֵ
���룺    pssNum�������׵�ַ
		  ssLenNum������ĳ���
�����    �����ƽ��ֵ
��ע��    
********************************************************************/
FP32 CalAverage(SSHORT16 *pssNum, SSHORT16 ssLenNum)
{
	SINT32 sumTemp = 0;
	USHORT16 i;
	for (i = 0; i < ssLenNum; i++)
	{
		sumTemp += *(pssNum + i);
	}
	
	return (FP32)sumTemp / ssLenNum;
}



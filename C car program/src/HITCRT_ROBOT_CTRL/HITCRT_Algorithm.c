/*******************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：HITCRT_Algorithm.c
最近修改日期：2010.11.29
版本：
--------------------------------------------------------------------
模块描述：各种运算函数
函数列表： 
1.Clip()                  削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值  
2.Round()                 将浮点数四舍五入，返回32位整型数
3.CalPIDIS()              计算PID量  积分分离式PID，可防止过大超调量
4.CalPIDWTCOL()           计算PID量  遇限削弱式PID，可防止PID运算结果长期饱和
5.NoCalPIDWTCOL()         计算PID量  无遇限削弱式PID 
6.CalNormalProjection()   计算一向量在基准向量法向方向投影
7.CalRadialProjection()   计算一向量在基准向量方向投影
8.CalAngle()              计算一向量与ssPosX轴正半轴夹角
9.ConvertAngle()          将角度转换为全局坐标系的航向角范围[-PI,PI)
10.ConvertDeg()           将角度转换为全局坐标系的航向角范围[-1800,1800)(单位：0.1°)
11.CalAverage()           求一数组的平均值
12.AvoidDoublePress()     防止按键连击
13.CalPIDUni()			  归一化参数整定
--------------------------------------------------------------------
修改记录：
作者        时间            版本     说明
任伟        2010.3.5        1.0      建立此文件
詹军成      2010.11.29      2.0      规范化此文件
********************************************************************/

#include "HITCRT_RobotTypes.h"
#include "HITCRT_Coff.h"
#include "HITCRT_API.h"
#include "math.h"
/*******************************************************************
函数名称：Clip()
函数功能：削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值
输入：    siValue:实际值
			siMin:下限值
			siMax:上限值
输出：   削波后的值
备注：
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
函数名称：Round()
函数功能：将浮点数四舍五入，返回32位整型数
输入：    fpValue
输出：    四舍五入后返回的整型数
备注：
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
函数名称：CalPIDIS()
函数功能：计算PID量
          增量型PID算法的计算公式：detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+D*det(E(k))+det(E(k)))
          积分分离式PID，可防止过大超调量
输入：    pstPid:需要做PID运算的速度PID结构体指针
输出：    无
备注：
********************************************************************/
void CalPIDIS(volatile ST_PID *pstPid)	//积分分离式PID算法(IntegralSeparated)
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
函数名称：CalPIDWTCOL() 
函数功能：计算PID量
          增量型PID算法的计算公式：detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+fpD*det(E(k))+det(E(k)))
          遇限削弱式PID，可防止PID运算结果长期饱和
输入：    pstPid:需要做PID运算的速度PID结构体指针
输出：    无
备注：
********************************************************************/
void CalPIDWTCOL(volatile ST_PID *pstPid)    //遇限削弱式PID算法（Weaken The Case Of Limited）
{   
    UCHAR8 ucK;
    
	 
	if (((pstPid->fpU > pstPid->fpULimit) && (pstPid->fpE > 0)) || ((pstPid->fpU < -pstPid->fpULimit) && (pstPid->fpE < 0)))//若饱和，在能减小结果绝对值的前提下才进行积分
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
函数名称：CalPIDWTCOLIS() 
函数功能：计算PID量
          增量型PID算法的计算公式：detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+fpD*det(E(k))+det(E(k)))
          积分分离+遇限削弱式PID结合二者优点
输入：    pstPid:需要做PID运算的速度PID结构体指针
输出：    无
备注：
********************************************************************/
void CalPIDWTCOLIS(volatile ST_PID *pstPid)  
{   
    UCHAR8 ucK1, ucK2;
    
    
    //遇限削弱
	if (((pstPid->fpU > pstPid->fpULimit) && (pstPid->fpE > 0)) 
		|| ((pstPid->fpU < -pstPid->fpULimit) && (pstPid->fpE < 0)))//根据上一次的pstPid运算结果来判断是否要有积分环节
	{
	    ucK1=0;
	}
	else
	{ 
		ucK1=1;
	}
	
	//积分分离
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
函数名称：NoCalPIDWTCOL()  
函数功能：计算PID量
          增量型PID算法的计算公式：detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+fpD*det(E(k))+det(E(k)))
          无遇限削弱式PID
输入：    pstPid:需要做PID运算的速度PID结构体指针
输出：    无
备注：
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
函数名称：CalPIDUni()  
函数功能：计算PID量
          增量型PID算法的计算公式：detU(k)=U(k)-U(k-1)=Kp(detE(k)+IE(k)+fpD*det(E(k))+det(E(k)))
          PID归一化参数整定公式
输入：    pstPid:需要做PID运算的速度PID结构体指针
输出：    无
备注：	  导航用，只需整定参数Kp，系数数的确定参看相关书籍
********************************************************************/
void CalPIDUni(volatile ST_PID *pstPid)    
{   
	pstPid->fpU += pstPid->fpP * (11.0286 * pstPid->fpE - 21 * pstPid->fpPreE + 10 * pstPid->fpPrePreE);
	pstPid->fpPrePreE = pstPid->fpPreE;
	pstPid->fpPreE = pstPid->fpE;
}

/*******************************************************************
函数名称：CalNormalProjection()  
函数功能：计算一向量在基准向量法向方向投影
输入：    stAim:目标向量，即需要投影的向量
		  stBase:基准向量
输出：    目标向量在基准向量法向方向上的投影
备注：    基准向量的法向向量为基准向量顺时针旋转90°得到
		  当目标点在基准直线左侧时（以基准向量方向为正方向），投影值为正，反之为负
********************************************************************/
SSHORT16 CalNormalProjection(ST_VECTOR stAim,ST_VECTOR stBase)
{   
	return Round((FP32)(stBase.ssVy * stAim.ssVx + -stBase.ssVx * stAim.ssVy)/sqrt((FP32)(stBase.ssVy * stBase.ssVy + stBase.ssVx * stBase.ssVx)));		
}

/*******************************************************************
函数名称：CalRadialProjection()  
函数功能：计算一向量在基准向量方向投影
输入：    stAim:目标向量，即需要投影的向量
		  stBase:基准向量
输出：    目标向量在基准向量上的投影
备注：    当目标点在基准直线之间时，投影值为正，反之为负
********************************************************************/
SSHORT16 CalRadialProjection(ST_VECTOR stAim, ST_VECTOR stBase)
{   
	return Round((FP32)(stBase.ssVx * stAim.ssVx + stBase.ssVy * stAim.ssVy)/sqrt((FP32)(stBase.ssVy * stBase.ssVy + stBase.ssVx * stBase.ssVx))) ;		
}

/*******************************************************************
函数名称：CalAngle()  
函数功能：计算一向量与ssPosY轴正半轴夹角
输入：    stAim:目标向量
输出：    目标向量与ssPosY轴正半轴夹角(RADIAN)
备注：    逆时针为正，顺时针为负，计算夹角范围为：[-PI,PI)
********************************************************************/
FP32 CalAngle(ST_VECTOR stAim)
{   
    FP32 fpAngAim;
	
    if(stAim.ssVy == 0)//分母为0
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
		    if (stAim.ssVy < 0)//3、4象限
		    {
		    	if (fpAngAim >= 0)//4象限
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
函数名称：ConvertAngle()  
函数功能：将角度转换为全局坐标系的航向角范围[-PI,PI)
输入：    ang：目标角度(RADIAN)
输出：    转换后的角度(RADIAN)
备注：    逆时针为正，顺时针为负，不适合对角度值较大的值做转换
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
函数名称：ConvertAngle_2()  
函数功能：将[-PI,PI]角度转换为全局坐标系与X轴正向的航向角范围[0,2PI)
输入：    ang：目标角度(RADIAN)
输出：    转换后的角度(RADIAN)
备注：    逆时针为正
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
函数名称：ConvertAngle_3()  
函数功能：将角度转换为全局坐标系与X轴正向的航向角范围[0,2PI)
输入：    ang：目标角度(RADIAN)
输出：    转换后的角度(RADIAN)
备注：    逆时针为正
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
函数名称：ConvertDeg()  
函数功能：将角度转换为全局坐标系的航向角范围[-1800,1800)(单位：0.1°)
输入：    ang：目标角度(单位：0.1°)
输出：    转换后的角度(单位：0.1°)
备注：    逆时针为正，顺时针为负，不适合对角度值较大的值做转换
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
函数名称：CalAverage()  
函数功能：求一数组的平均值
输入：    pssNum：数组首地址
		  ssLenNum：数组的长度
输出：    数组的平均值
备注：    
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



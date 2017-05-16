#include "HITCRT_Types.h"
#include "HITCRT_Coff.h"
#include "math.h"
#include "HITCRT_RobotTypes.h"
#include "HITCRT_Algorithm.h"
#include "Location.h"
#include "FPGA_REGS.h"
#include "RobotCtrl.h"

extern UCHAR8 g_ucUseMemsGryo;
/*******************************************************************
函数名称：DoubleVerticalOMNILocate()
函数功能：利用垂直布置的双全向随动轮求出机器人的全局坐标
输入：     
输出：    无
备注：   
***************************** ***************************************/
void DoubleVerticalOMNILocate(ST_DOUBLE_OMNI_LOCATION *pstDoubleOMNI , ST_OMNI_MOBILE_ROBOT *pstRobot)
{
       FP32 fpDeltaLengthA, fpDeltaLengthB;    //A为x向随动轮位移，B为y向随动轮位移
	FP32 fpDeltaLengthX, fpDeltaLengthY;    //机器人局部坐标
	FP32 fpHalfDeltaQ;                      //机器人航向角 1/2
	FP32 fpQ,fpQTemp;                               //计算时用到的航向角，坐标系旋转，坐标系角度偏差均在此角内
		 
	/*读取码盘脉冲数及航向角*/
	pstDoubleOMNI->siCoderA = READ_CODER(OMNI_CODER_A);	
	pstDoubleOMNI->siCoderB =  READ_CODER(OMNI_CODER_B);	
   	pstDoubleOMNI->fpPosQ = COFF_OMNI_Q * GRYO_Q;	  


	/*计算随动轮位移差，码盘B的输出对应的y正向与定义的y正向相反，故加负号*/
	fpDeltaLengthA = COFF_OMNI_A * (pstDoubleOMNI->siCoderA - pstDoubleOMNI->siLastCoderA) 
	                 *COFF_CODERA_TO_LENGTH ;
	fpDeltaLengthB =  COFF_OMNI_B * (pstDoubleOMNI->siCoderB - pstDoubleOMNI->siLastCoderB) 
	                 * COFF_CODERA_TO_LENGTH ;

    /*角度处理原因:1.简化计算 2.(需要时)完成坐标系旋转 3.完成坐标系角度偏差修正*/
	fpQ = pstDoubleOMNI->fpPosQ * RADIAN_10 + DELTA_ANGLE_VERTICAL;
	fpHalfDeltaQ = (pstDoubleOMNI->fpPosQ - pstDoubleOMNI->fpLastPosQ) * RADIAN_10 / 2;    //简化计算
//	fpQTemp=1.2235* RADIAN_10; 
	
	/*将随动轮位移 转化为随动轮局部坐标位移*/ 	
	if(fabs(fpHalfDeltaQ) < 1e-7)
	{
     	fpDeltaLengthX = fpDeltaLengthA;
     	fpDeltaLengthY = fpDeltaLengthB;
	}
	else
	{
		fpDeltaLengthX = fpHalfDeltaQ / tan(fpHalfDeltaQ) * fpDeltaLengthA 
     		             - fpHalfDeltaQ * fpDeltaLengthB;
	     	fpDeltaLengthY = fpHalfDeltaQ / tan(fpHalfDeltaQ) * fpDeltaLengthB 
	     		             - fpHalfDeltaQ * fpDeltaLengthA;
	}
	
#if	1
	/*由随动轮局部坐标位移计算出随动轮全局坐标*/
	pstDoubleOMNI->fpPosX = fpDeltaLengthX * cos(fpQ) - fpDeltaLengthY * sin(fpQ)
	                        + pstDoubleOMNI->fpLastPosX;
	pstDoubleOMNI->fpPosY = fpDeltaLengthX * sin(fpQ) + fpDeltaLengthY * cos(fpQ)
		                    + pstDoubleOMNI->fpLastPosY;

#endif	

	
	/*随动轮坐标转化为机器人坐标*/
/*	stPosTemp1.ssPosX = (SSHORT16)pstDoubleOMNI->fpPosX;
    stPosTemp1.ssPosY = (SSHORT16)pstDoubleOMNI->fpPosY;
	stPosTemp1.ssPosQ = (SSHORT16)pstDoubleOMNI->fpPosQ;*/
	pstRobot->stPot.ssPosX = (SSHORT16)pstDoubleOMNI->fpPosX;//+ sin(-fpQ + COFF_2CENTER_ANGLE) * COFF_2CENTER_DIS;
	pstRobot->stPot.ssPosY = (SSHORT16)pstDoubleOMNI->fpPosY ;//+ cos(-fpQ + COFF_2CENTER_ANGLE) * COFF_2CENTER_DIS;
	pstRobot->stPot.ssPosQ = (SSHORT16)pstDoubleOMNI->fpPosQ;

	
	/*更新上一次的值*/
	pstDoubleOMNI->fpLastPosQ = pstDoubleOMNI->fpPosQ;		
	pstDoubleOMNI->fpLastPosX = pstDoubleOMNI->fpPosX;
	pstDoubleOMNI->fpLastPosY = pstDoubleOMNI->fpPosY;
    pstDoubleOMNI->siLastCoderA = pstDoubleOMNI->siCoderA;
	pstDoubleOMNI->siLastCoderB = pstDoubleOMNI->siCoderB;

}


#if 0
/*******************************************************************
函数名称：DoubleTriangularOMNILocate()
函数功能：利用90度布置的双随动轮求出机器人全局坐标
输入：      
输出：    无
备注：    每次计算时，三轮速度和角速度与上一时刻相比（理解成)不变，再
          利用当前航向角计算出绝度速度，此速度也（理解成）不变，（实际
          上，此速度用上一次的航向角计算，肯定不同，但此算法忽略其影响）。
          最后，对所有的速度表达式两端积分，便可得到以下公式。
***************************** ***************************************/
void DoubleVerticalOMNILocatenew(ST_DOUBLE_OMNI_LOCATION *pstDoubleOMNI , ST_OMNI_MOBILE_ROBOT *pstRobot)
{
	FP32 fpDeltaLengthA, fpDeltaLengthB;    //A为左随动轮，B为右随动轮，C为假想随动轮
	FP32 fpQ,fpQTemp;
                   //局部坐标x向，y向位移差
                      //全局坐标x向，y向位移差

	
	/*读取码盘脉冲数及航向角*/
	//pstDoubleOMNI->siCoderA = -READ_CODER(OMNI_CODER_A);	
	//pstDoubleOMNI->siCoderB = -READ_CODER(OMNI_CODER_B);	

	pstDoubleOMNI->siCoderB = -READ_CODER(OMNI_CODER_A);	
	pstDoubleOMNI->siCoderA = -READ_CODER(OMNI_CODER_B);	
    	pstDoubleOMNI->fpPosQ = COFF_OMNI_NEWQ* GRYO_Q;

	
	/*计算随动轮位移差,由双随动码盘A,B推算出第三个假想码盘C的位移增量*/
	fpDeltaLengthA =  (pstDoubleOMNI->siCoderA - pstDoubleOMNI->siLastCoderA) * COFF_PULSE_TO_LENGTHA;    
	fpDeltaLengthB =  (pstDoubleOMNI->siCoderB - pstDoubleOMNI->siLastCoderB) * COFF_PULSE_TO_LENGTHB;
	
    /*角度处理:1.简化计算 2.(需要时)完成坐标系旋转 3.完成坐标系角度偏差修正*/
   	fpQ = pstDoubleOMNI->fpPosQ * RADIAN_10;// - DELTA_ANGLE_TRIANGULAR;
	
	fpQTemp=1.2235* RADIAN_10;
	

	pstDoubleOMNI->fpPosX =-( fpDeltaLengthA * sin(fpQ) + fpDeltaLengthB * sin(fpABQ-fpQ))/sin(fpABQ)
		                    + pstDoubleOMNI->fpLastPosX; 


	pstDoubleOMNI->fpPosY= (fpDeltaLengthA* cos(fpQ) - fpDeltaLengthB* cos(fpABQ-fpQ))/sin(fpABQ)
	                        + pstDoubleOMNI->fpLastPosY;
	
	
	/*随动中心坐标转化为机器人坐标*/  
	pstRobot->stPot.ssPosX = (SSHORT16)(pstDoubleOMNI->fpPosX); //- sin(fpQ) * COFF_2CENTER_DIS);
	pstRobot->stPot.ssPosY = (SSHORT16)(pstDoubleOMNI->fpPosY); //+ cos(fpQ) * COFF_2CENTER_DIS);
	pstRobot->stPot.ssPosQ = (SSHORT16)pstDoubleOMNI->fpPosQ;

	
	/*更新上一次的值*/
	pstDoubleOMNI->fpLastPosQ = pstDoubleOMNI->fpPosQ;		
	pstDoubleOMNI->fpLastPosX = pstDoubleOMNI->fpPosX;
	pstDoubleOMNI->fpLastPosY = pstDoubleOMNI->fpPosY;
   	 pstDoubleOMNI->siLastCoderA = pstDoubleOMNI->siCoderA;
	pstDoubleOMNI->siLastCoderB = pstDoubleOMNI->siCoderB;
}

#endif

#if 1
/*******************************************************************
函数名称：DoubleTriangularOMNILocate()
函数功能：利用120度布置的双随动轮求出机器人全局坐标
输入：      
输出：    无
备注：    每次计算时，三轮速度和角速度与上一时刻相比（理解成)不变，再
          利用当前航向角计算出绝度速度，此速度也（理解成）不变，（实际
          上，此速度用上一次的航向角计算，肯定不同，但此算法忽略其影响）。
          最后，对所有的速度表达式两端积分，便可得到以下公式。
***************************** ***************************************/
void DoubleTriangularOMNILocate(ST_DOUBLE_OMNI_LOCATION *pstDoubleOMNI , ST_OMNI_MOBILE_ROBOT *pstRobot)
{
	FP32 fpDeltaLengthA, fpDeltaLengthB, fpDeltaLengthC;    //A为左随动轮，B为右随动轮，C为假想随动轮
	FP32 fpQA,fpQB,fpQ;
	FP32 fpDeltaLengthX,fpDeltaLengthY;                     //局部坐标x向，y向位移差
    FP32 fpDeltaLengthX0,fpDeltaLengthY0;                   //全局坐标x向，y向位移差

	
	/*读取码盘脉冲数及航向角*/
	pstDoubleOMNI->siCoderA = -READ_CODER(OMNI_CODER_A);	
	pstDoubleOMNI->siCoderB = -READ_CODER(OMNI_CODER_B);	
    pstDoubleOMNI->fpPosQ = COFF_OMNI_Q * GRYO_Q;

	
	/*计算随动轮位移差,由双随动码盘A,B推算出第三个假想码盘C的位移增量*/
	fpDeltaLengthA =  (pstDoubleOMNI->siCoderA - pstDoubleOMNI->siLastCoderA) * COFF_PULSE_TO_LENGTHA;    
	fpDeltaLengthB =  (pstDoubleOMNI->siCoderB - pstDoubleOMNI->siLastCoderB) * COFF_PULSE_TO_LENGTHB;
	fpDeltaLengthC = COFF_ROBOT_3R * (pstDoubleOMNI->fpPosQ - pstDoubleOMNI->fpLastPosQ) * RADIAN_10
					 - fpDeltaLengthA - fpDeltaLengthB;	


    /*角度处理:1.简化计算 2.(需要时)完成坐标系旋转 3.完成坐标系角度偏差修正*/
    fpQ = pstDoubleOMNI->fpPosQ * RADIAN_10;// - DELTA_ANGLE_TRIANGULAR;
	fpQA = PI_3 - fpQ;
	fpQB = PI_3 + fpQ;

	
	/*由随动轮位移计算出全局坐标位移*/
	fpDeltaLengthX0 = COFF_2_3 * (-fpDeltaLengthA * cos(fpQA) - fpDeltaLengthB * cos(fpQB) 
		              + fpDeltaLengthC * cos(fpQ));
	fpDeltaLengthY0 = COFF_2_3 * (fpDeltaLengthA * sin(fpQA) - fpDeltaLengthB * sin(fpQB) 
		              + fpDeltaLengthC * sin(fpQ));
	
	
	/*因修正系数在航向角为零的情况下测得，故将全局坐标位移转换至局部坐标位移进行修正*/
	fpDeltaLengthX = fpDeltaLengthY0 * sin(fpQ) + fpDeltaLengthX0 * cos(fpQ);
	fpDeltaLengthY = fpDeltaLengthY0 * cos(fpQ) - fpDeltaLengthX0 * sin(fpQ);          
	
	if(fpDeltaLengthX > 0)	//X轴正负方向矫正
	{
		fpDeltaLengthX *= 0.98428;//0.932794872;
	}
	else
	{
		fpDeltaLengthX *= 0.98428;//0.932794872;
	}
	
	if(fpDeltaLengthY > 0)	//Y轴正负方向矫正
	{
		fpDeltaLengthY *= 1.01045;//0.997486334f;
	}
	else
	{
		fpDeltaLengthY *= 1.01045;//0.996770463f;
	}


    /*将修正后的位移再转换至全局坐标，并累加，得到随动中心的坐标*/
	pstDoubleOMNI->fpPosX = fpDeltaLengthX * cos(fpQ) - fpDeltaLengthY * sin(fpQ)
	                        + pstDoubleOMNI->fpLastPosX;
	pstDoubleOMNI->fpPosY = fpDeltaLengthX * sin(fpQ) + fpDeltaLengthY * cos(fpQ)
		                    + pstDoubleOMNI->fpLastPosY; 

	
	/*随动中心坐标转化为机器人坐标*/  
	pstRobot->stPot.ssPosX = (SSHORT16)(pstDoubleOMNI->fpPosX - sin(fpQ) * COFF_2CENTER_DIS);
	pstRobot->stPot.ssPosY = (SSHORT16)(pstDoubleOMNI->fpPosY + cos(fpQ) * COFF_2CENTER_DIS);
	pstRobot->stPot.ssPosQ = (SSHORT16)pstDoubleOMNI->fpPosQ;
//	pstRobot->stPot.ssPosX = (SSHORT16)pstDoubleOMNI->fpPosX;
//	pstRobot->stPot.ssPosY = (SSHORT16)pstDoubleOMNI->fpPosY;
//	pstRobot->stPot.ssPosQ = (SSHORT16)pstDoubleOMNI->fpPosQ;
	/*stPosTemp1.ssPosX = (SSHORT16)pstDoubleOMNI->fpPosX - stPosTemp3.ssPosX;
    stPosTemp1.ssPosY = (SSHORT16)pstDoubleOMNI->fpPosY - stPosTemp3.ssPosY;
	stPosTemp1.ssPosQ = (SSHORT16)pstDoubleOMNI->fpPosQ;*/
	
	/*更新上一次的值*/
	pstDoubleOMNI->fpLastPosQ = pstDoubleOMNI->fpPosQ;		
	pstDoubleOMNI->fpLastPosX = pstDoubleOMNI->fpPosX;
	pstDoubleOMNI->fpLastPosY = pstDoubleOMNI->fpPosY;
    pstDoubleOMNI->siLastCoderA = pstDoubleOMNI->siCoderA;
	pstDoubleOMNI->siLastCoderB = pstDoubleOMNI->siCoderB;
}

#endif
void DoubleVerticalOMNILocateEx1(ST_DOUBLE_OMNI_LOCATION * pstDoubleOMNI, ST_OMNI_MOBILE_ROBOT * pstRobot)
{
	FP32 fpCoderALength, fpCoderBLength;
	FP32 fpCosValue,fpSinValue;
	FP32 fpCosValue1,fpSinValue1;
	FP32 fpQ,fpQ1,fpSinALPHA_A_B,fpQ_ALPHA_A_B,fpQReal;
	FP32 fpDeltaX, fpDeltaY;

	if(g_ucUseMemsGryo==1)
		{
       		g_fpGyroAngle = -g_ssMemsGryo;
		}
	else
		{
			g_fpGyroAngle = -COFF_OMNI_Q * GRYO_Q;
		}

	pstDoubleOMNI->siCoderA =  READ_CODER(OMNI_CODER_A);	
	pstDoubleOMNI->siCoderB = -READ_CODER(OMNI_CODER_B);	
       pstDoubleOMNI->fpPosQ = g_fpGyroAngle - g_fpModifyAngle;

	 fpQReal=pstDoubleOMNI->fpPosQ+8;//12.5

	   fpQ_ALPHA_A_B=ALPHA_A_B+10.5*RADIAN_10;
	  fpSinALPHA_A_B = sin(fpQ_ALPHA_A_B);
	   fpCoderALength = (pstDoubleOMNI->siCoderA - pstDoubleOMNI->siLastCoderA)*COFF_CODERA_TO_LENGTH;
	   fpCoderBLength = (pstDoubleOMNI->siCoderB-  pstDoubleOMNI->siLastCoderB)*COFF_CODERB_TO_LENGTH;

	  // fpQ = pstDoubleOMNI->fpPosQ * RADIAN_10 + PI_ALPHA_A;
	  // fpQ1 =  pstDoubleOMNI->fpPosQ * RADIAN_10 +PI_ALPHA_A_ALPHA_A_B;
	   fpQ =  fpQReal* RADIAN_10 + fpQ_ALPHA_A_B/2;
	   fpQ1 =  -  fpQReal* RADIAN_10+fpQ_ALPHA_A_B/2 ;
	   fpCosValue = cos(fpQ);
	   fpSinValue  = sin(fpQ);
	   fpCosValue1 = cos(fpQ1);
	   fpSinValue1  = sin(fpQ1);

	   fpDeltaX =- fpCoderALength*fpCosValue+ fpCosValue1*  fpCoderBLength;
	   fpDeltaY = -fpCoderALength*fpSinValue -fpSinValue1 *  fpCoderBLength;
	   
	   fpDeltaX = fpDeltaX / fpSinALPHA_A_B;
	   fpDeltaY = fpDeltaY  /fpSinALPHA_A_B;
	   
       pstDoubleOMNI->fpPosX =  pstDoubleOMNI->fpLastPosX + fpDeltaX;
	pstDoubleOMNI->fpPosY =   pstDoubleOMNI->fpLastPosY   +  fpDeltaY;   
	   
       pstRobot->stPot.ssPosX = (SSHORT16)(pstDoubleOMNI->fpPosX )+ CTR_TO_CTR *  sin(pstDoubleOMNI->fpPosQ * RADIAN_10);
	pstRobot->stPot.ssPosY = (SSHORT16)(pstDoubleOMNI->fpPosY)+ CTR_TO_CTR *(1 -   cos(pstDoubleOMNI->fpPosQ * RADIAN_10)) ;
	pstRobot->stPot.ssPosQ = (SSHORT16)pstDoubleOMNI->fpPosQ;

	pstDoubleOMNI->fpLastPosQ = pstDoubleOMNI->fpPosQ;		
	pstDoubleOMNI->fpLastPosX = pstDoubleOMNI->fpPosX;
	pstDoubleOMNI->fpLastPosY = pstDoubleOMNI->fpPosY;
       pstDoubleOMNI->siLastCoderA = pstDoubleOMNI->siCoderA;
	pstDoubleOMNI->siLastCoderB = pstDoubleOMNI->siCoderB;

	   
	   

	
}


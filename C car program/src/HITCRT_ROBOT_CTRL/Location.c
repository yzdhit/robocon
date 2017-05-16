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
�������ƣ�DoubleVerticalOMNILocate()
�������ܣ����ô�ֱ���õ�˫ȫ���涯����������˵�ȫ������
���룺     
�����    ��
��ע��   
***************************** ***************************************/
void DoubleVerticalOMNILocate(ST_DOUBLE_OMNI_LOCATION *pstDoubleOMNI , ST_OMNI_MOBILE_ROBOT *pstRobot)
{
       FP32 fpDeltaLengthA, fpDeltaLengthB;    //AΪx���涯��λ�ƣ�BΪy���涯��λ��
	FP32 fpDeltaLengthX, fpDeltaLengthY;    //�����˾ֲ�����
	FP32 fpHalfDeltaQ;                      //�����˺���� 1/2
	FP32 fpQ,fpQTemp;                               //����ʱ�õ��ĺ���ǣ�����ϵ��ת������ϵ�Ƕ�ƫ����ڴ˽���
		 
	/*��ȡ�����������������*/
	pstDoubleOMNI->siCoderA = READ_CODER(OMNI_CODER_A);	
	pstDoubleOMNI->siCoderB =  READ_CODER(OMNI_CODER_B);	
   	pstDoubleOMNI->fpPosQ = COFF_OMNI_Q * GRYO_Q;	  


	/*�����涯��λ�Ʋ����B�������Ӧ��y�����붨���y�����෴���ʼӸ���*/
	fpDeltaLengthA = COFF_OMNI_A * (pstDoubleOMNI->siCoderA - pstDoubleOMNI->siLastCoderA) 
	                 *COFF_CODERA_TO_LENGTH ;
	fpDeltaLengthB =  COFF_OMNI_B * (pstDoubleOMNI->siCoderB - pstDoubleOMNI->siLastCoderB) 
	                 * COFF_CODERA_TO_LENGTH ;

    /*�Ƕȴ���ԭ��:1.�򻯼��� 2.(��Ҫʱ)�������ϵ��ת 3.�������ϵ�Ƕ�ƫ������*/
	fpQ = pstDoubleOMNI->fpPosQ * RADIAN_10 + DELTA_ANGLE_VERTICAL;
	fpHalfDeltaQ = (pstDoubleOMNI->fpPosQ - pstDoubleOMNI->fpLastPosQ) * RADIAN_10 / 2;    //�򻯼���
//	fpQTemp=1.2235* RADIAN_10; 
	
	/*���涯��λ�� ת��Ϊ�涯�־ֲ�����λ��*/ 	
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
	/*���涯�־ֲ�����λ�Ƽ�����涯��ȫ������*/
	pstDoubleOMNI->fpPosX = fpDeltaLengthX * cos(fpQ) - fpDeltaLengthY * sin(fpQ)
	                        + pstDoubleOMNI->fpLastPosX;
	pstDoubleOMNI->fpPosY = fpDeltaLengthX * sin(fpQ) + fpDeltaLengthY * cos(fpQ)
		                    + pstDoubleOMNI->fpLastPosY;

#endif	

	
	/*�涯������ת��Ϊ����������*/
/*	stPosTemp1.ssPosX = (SSHORT16)pstDoubleOMNI->fpPosX;
    stPosTemp1.ssPosY = (SSHORT16)pstDoubleOMNI->fpPosY;
	stPosTemp1.ssPosQ = (SSHORT16)pstDoubleOMNI->fpPosQ;*/
	pstRobot->stPot.ssPosX = (SSHORT16)pstDoubleOMNI->fpPosX;//+ sin(-fpQ + COFF_2CENTER_ANGLE) * COFF_2CENTER_DIS;
	pstRobot->stPot.ssPosY = (SSHORT16)pstDoubleOMNI->fpPosY ;//+ cos(-fpQ + COFF_2CENTER_ANGLE) * COFF_2CENTER_DIS;
	pstRobot->stPot.ssPosQ = (SSHORT16)pstDoubleOMNI->fpPosQ;

	
	/*������һ�ε�ֵ*/
	pstDoubleOMNI->fpLastPosQ = pstDoubleOMNI->fpPosQ;		
	pstDoubleOMNI->fpLastPosX = pstDoubleOMNI->fpPosX;
	pstDoubleOMNI->fpLastPosY = pstDoubleOMNI->fpPosY;
    pstDoubleOMNI->siLastCoderA = pstDoubleOMNI->siCoderA;
	pstDoubleOMNI->siLastCoderB = pstDoubleOMNI->siCoderB;

}


#if 0
/*******************************************************************
�������ƣ�DoubleTriangularOMNILocate()
�������ܣ�����90�Ȳ��õ�˫�涯�����������ȫ������
���룺      
�����    ��
��ע��    ÿ�μ���ʱ�������ٶȺͽ��ٶ�����һʱ����ȣ�����)���䣬��
          ���õ�ǰ����Ǽ���������ٶȣ����ٶ�Ҳ�����ɣ����䣬��ʵ��
          �ϣ����ٶ�����һ�εĺ���Ǽ��㣬�϶���ͬ�������㷨������Ӱ�죩��
          ��󣬶����е��ٶȱ��ʽ���˻��֣���ɵõ����¹�ʽ��
***************************** ***************************************/
void DoubleVerticalOMNILocatenew(ST_DOUBLE_OMNI_LOCATION *pstDoubleOMNI , ST_OMNI_MOBILE_ROBOT *pstRobot)
{
	FP32 fpDeltaLengthA, fpDeltaLengthB;    //AΪ���涯�֣�BΪ���涯�֣�CΪ�����涯��
	FP32 fpQ,fpQTemp;
                   //�ֲ�����x��y��λ�Ʋ�
                      //ȫ������x��y��λ�Ʋ�

	
	/*��ȡ�����������������*/
	//pstDoubleOMNI->siCoderA = -READ_CODER(OMNI_CODER_A);	
	//pstDoubleOMNI->siCoderB = -READ_CODER(OMNI_CODER_B);	

	pstDoubleOMNI->siCoderB = -READ_CODER(OMNI_CODER_A);	
	pstDoubleOMNI->siCoderA = -READ_CODER(OMNI_CODER_B);	
    	pstDoubleOMNI->fpPosQ = COFF_OMNI_NEWQ* GRYO_Q;

	
	/*�����涯��λ�Ʋ�,��˫�涯����A,B�������������������C��λ������*/
	fpDeltaLengthA =  (pstDoubleOMNI->siCoderA - pstDoubleOMNI->siLastCoderA) * COFF_PULSE_TO_LENGTHA;    
	fpDeltaLengthB =  (pstDoubleOMNI->siCoderB - pstDoubleOMNI->siLastCoderB) * COFF_PULSE_TO_LENGTHB;
	
    /*�Ƕȴ���:1.�򻯼��� 2.(��Ҫʱ)�������ϵ��ת 3.�������ϵ�Ƕ�ƫ������*/
   	fpQ = pstDoubleOMNI->fpPosQ * RADIAN_10;// - DELTA_ANGLE_TRIANGULAR;
	
	fpQTemp=1.2235* RADIAN_10;
	

	pstDoubleOMNI->fpPosX =-( fpDeltaLengthA * sin(fpQ) + fpDeltaLengthB * sin(fpABQ-fpQ))/sin(fpABQ)
		                    + pstDoubleOMNI->fpLastPosX; 


	pstDoubleOMNI->fpPosY= (fpDeltaLengthA* cos(fpQ) - fpDeltaLengthB* cos(fpABQ-fpQ))/sin(fpABQ)
	                        + pstDoubleOMNI->fpLastPosY;
	
	
	/*�涯��������ת��Ϊ����������*/  
	pstRobot->stPot.ssPosX = (SSHORT16)(pstDoubleOMNI->fpPosX); //- sin(fpQ) * COFF_2CENTER_DIS);
	pstRobot->stPot.ssPosY = (SSHORT16)(pstDoubleOMNI->fpPosY); //+ cos(fpQ) * COFF_2CENTER_DIS);
	pstRobot->stPot.ssPosQ = (SSHORT16)pstDoubleOMNI->fpPosQ;

	
	/*������һ�ε�ֵ*/
	pstDoubleOMNI->fpLastPosQ = pstDoubleOMNI->fpPosQ;		
	pstDoubleOMNI->fpLastPosX = pstDoubleOMNI->fpPosX;
	pstDoubleOMNI->fpLastPosY = pstDoubleOMNI->fpPosY;
   	 pstDoubleOMNI->siLastCoderA = pstDoubleOMNI->siCoderA;
	pstDoubleOMNI->siLastCoderB = pstDoubleOMNI->siCoderB;
}

#endif

#if 1
/*******************************************************************
�������ƣ�DoubleTriangularOMNILocate()
�������ܣ�����120�Ȳ��õ�˫�涯�����������ȫ������
���룺      
�����    ��
��ע��    ÿ�μ���ʱ�������ٶȺͽ��ٶ�����һʱ����ȣ�����)���䣬��
          ���õ�ǰ����Ǽ���������ٶȣ����ٶ�Ҳ�����ɣ����䣬��ʵ��
          �ϣ����ٶ�����һ�εĺ���Ǽ��㣬�϶���ͬ�������㷨������Ӱ�죩��
          ��󣬶����е��ٶȱ��ʽ���˻��֣���ɵõ����¹�ʽ��
***************************** ***************************************/
void DoubleTriangularOMNILocate(ST_DOUBLE_OMNI_LOCATION *pstDoubleOMNI , ST_OMNI_MOBILE_ROBOT *pstRobot)
{
	FP32 fpDeltaLengthA, fpDeltaLengthB, fpDeltaLengthC;    //AΪ���涯�֣�BΪ���涯�֣�CΪ�����涯��
	FP32 fpQA,fpQB,fpQ;
	FP32 fpDeltaLengthX,fpDeltaLengthY;                     //�ֲ�����x��y��λ�Ʋ�
    FP32 fpDeltaLengthX0,fpDeltaLengthY0;                   //ȫ������x��y��λ�Ʋ�

	
	/*��ȡ�����������������*/
	pstDoubleOMNI->siCoderA = -READ_CODER(OMNI_CODER_A);	
	pstDoubleOMNI->siCoderB = -READ_CODER(OMNI_CODER_B);	
    pstDoubleOMNI->fpPosQ = COFF_OMNI_Q * GRYO_Q;

	
	/*�����涯��λ�Ʋ�,��˫�涯����A,B�������������������C��λ������*/
	fpDeltaLengthA =  (pstDoubleOMNI->siCoderA - pstDoubleOMNI->siLastCoderA) * COFF_PULSE_TO_LENGTHA;    
	fpDeltaLengthB =  (pstDoubleOMNI->siCoderB - pstDoubleOMNI->siLastCoderB) * COFF_PULSE_TO_LENGTHB;
	fpDeltaLengthC = COFF_ROBOT_3R * (pstDoubleOMNI->fpPosQ - pstDoubleOMNI->fpLastPosQ) * RADIAN_10
					 - fpDeltaLengthA - fpDeltaLengthB;	


    /*�Ƕȴ���:1.�򻯼��� 2.(��Ҫʱ)�������ϵ��ת 3.�������ϵ�Ƕ�ƫ������*/
    fpQ = pstDoubleOMNI->fpPosQ * RADIAN_10;// - DELTA_ANGLE_TRIANGULAR;
	fpQA = PI_3 - fpQ;
	fpQB = PI_3 + fpQ;

	
	/*���涯��λ�Ƽ����ȫ������λ��*/
	fpDeltaLengthX0 = COFF_2_3 * (-fpDeltaLengthA * cos(fpQA) - fpDeltaLengthB * cos(fpQB) 
		              + fpDeltaLengthC * cos(fpQ));
	fpDeltaLengthY0 = COFF_2_3 * (fpDeltaLengthA * sin(fpQA) - fpDeltaLengthB * sin(fpQB) 
		              + fpDeltaLengthC * sin(fpQ));
	
	
	/*������ϵ���ں����Ϊ�������²�ã��ʽ�ȫ������λ��ת�����ֲ�����λ�ƽ�������*/
	fpDeltaLengthX = fpDeltaLengthY0 * sin(fpQ) + fpDeltaLengthX0 * cos(fpQ);
	fpDeltaLengthY = fpDeltaLengthY0 * cos(fpQ) - fpDeltaLengthX0 * sin(fpQ);          
	
	if(fpDeltaLengthX > 0)	//X�������������
	{
		fpDeltaLengthX *= 0.98428;//0.932794872;
	}
	else
	{
		fpDeltaLengthX *= 0.98428;//0.932794872;
	}
	
	if(fpDeltaLengthY > 0)	//Y�������������
	{
		fpDeltaLengthY *= 1.01045;//0.997486334f;
	}
	else
	{
		fpDeltaLengthY *= 1.01045;//0.996770463f;
	}


    /*���������λ����ת����ȫ�����꣬���ۼӣ��õ��涯���ĵ�����*/
	pstDoubleOMNI->fpPosX = fpDeltaLengthX * cos(fpQ) - fpDeltaLengthY * sin(fpQ)
	                        + pstDoubleOMNI->fpLastPosX;
	pstDoubleOMNI->fpPosY = fpDeltaLengthX * sin(fpQ) + fpDeltaLengthY * cos(fpQ)
		                    + pstDoubleOMNI->fpLastPosY; 

	
	/*�涯��������ת��Ϊ����������*/  
	pstRobot->stPot.ssPosX = (SSHORT16)(pstDoubleOMNI->fpPosX - sin(fpQ) * COFF_2CENTER_DIS);
	pstRobot->stPot.ssPosY = (SSHORT16)(pstDoubleOMNI->fpPosY + cos(fpQ) * COFF_2CENTER_DIS);
	pstRobot->stPot.ssPosQ = (SSHORT16)pstDoubleOMNI->fpPosQ;
//	pstRobot->stPot.ssPosX = (SSHORT16)pstDoubleOMNI->fpPosX;
//	pstRobot->stPot.ssPosY = (SSHORT16)pstDoubleOMNI->fpPosY;
//	pstRobot->stPot.ssPosQ = (SSHORT16)pstDoubleOMNI->fpPosQ;
	/*stPosTemp1.ssPosX = (SSHORT16)pstDoubleOMNI->fpPosX - stPosTemp3.ssPosX;
    stPosTemp1.ssPosY = (SSHORT16)pstDoubleOMNI->fpPosY - stPosTemp3.ssPosY;
	stPosTemp1.ssPosQ = (SSHORT16)pstDoubleOMNI->fpPosQ;*/
	
	/*������һ�ε�ֵ*/
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


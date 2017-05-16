#ifndef	__ACTION_MOTOR_H
#define __ACTION_MOTOR_H
#include "MotorConfig.h"
#include "math.h"

#define M1COFF_POT	3.341549f						

#define M2COFF_POT	3.745501f

/*****************M1������˶��ߴ�*************************/
#define M1M_PICKUP_C		-138
#define M1M_PUTDOWN_C		24
#define M1A_DOWN_C			-150
#define M1A_UP_C			57
#define M1_L2_UP_C			24
#define M1_CLIMBSTEP_C		-221
#define M1_DOUBLEPOTLOOP_C   -188

/***************************��������ߴ�************************************/
#define M1_BLUE_PICKUPBAN_TEMP_C	7
#define M1_BLUE_PICKUPBAN_LAST_C	20
#define M1_BLUE_WAIT_M_C			141//Cץ����ͷ֮��ȴ�Mʱ�ĸ߶�
#define M1_BLUE_DROPBAN_C			135
#define M1_BLUE_LIFT_BAN			135//Cץ����ͷ֮������ĸ߶�
#define M1_BLUE_CATCHTOPBAN_C		145//ץȡ������ͷʱ������ľ���
#define M1_BLUE_PICKUPTOPBAN_C	240//274
#define M1_BLUE_DROPTOPBAN_C		-190//�Ӷ�����ͷ�ľ���
/***************************�쳡����ߴ�*******************************************************/
#define M1_RED_PICKUPBAN_TEMP_C	7
#define M1_RED_PICKUPBAN_LAST_C	20
#define M1_RED_LIFT_BAN			135//Cץ����ͷ֮������ĸ߶�
#define M1_RED_WAIT_M_C			141//Cץ����ͷ֮��ȴ�Mʱ�ĸ߶�
#define M1_RED_DROPBAN_C			135
#define M1_RED_CATCHTOPBAN_C		67//120//ץȡ������ͷʱ������ľ���
#define M1_RED_PICKUPTOPBAN_C		240//274
#define M1_RED_DROPTOPBAN_C		-190//�Ӷ�����ͷ�ľ���
/***********************************************************************/

#define M1_BLUE_CATCHMIDDLEBAN_C	20//�˴���C��ȡ��ͷʱ�����ĸ߶Ⱥܵͣ��Ա������ͷ�׼�

#define M1_RED_CATCHMIDDLEBAN_C	20//�˴���C��ȡ��ͷʱ�����ĸ߶Ⱥܵͣ��Ա������ͷ�׼�
/******************M2������˶��ߴ�***********************************************/

/***********************��������ߴ�*******************************/
#define M2_BLUE_PICKUPBANRES1_LAST_C	35//����·��1ʱ��ȡ�в���ͷM2������������λ��
#define M2_BLUE_PICKUPBANRES2_LAST_C	80  //����·��2ʱ��ȡ�в���ͷM2������������λ��
#define M2_BLUE_PICKUPBANRES3_TEMP_C	80
#define M2_BLUE_PICKUPBANRES3_LAST_C	110//����·��3ʱ��ȡ�в���ͷM2������������λ��
#define M2_BLUE_DROPBAN_C				40//���в�͵ײ���ͷʱM2�������ľ���
#define M2_BLUE_PICKUPTOPBAN_C		370//��ȡ������ͷʱM2�������ľ���
#define M2_BLUE_DROPTOPBAN_C			390//�Ӷ�����ͷʱM2���������ľ���
#define M2_BLUE_CATCHTOPBAN_COM_ON_C	10//����ͨ��ʱ��M2������ж�

/***********************�쳡����ߴ�*******************************/
#define M2_RED_PICKUPBANRES1_LAST_C	35//����·��1ʱ��ȡ�в���ͷM2������������λ��
#define M2_RED_PICKUPBANRES2_LAST_C	60  //����·��2ʱ��ȡ�в���ͷM2������������λ��
#define M2_RED_PICKUPBANRES3_TEMP_C	80
#define M2_RED_PICKUPBANRES3_LAST_C	110//����·��3ʱ��ȡ�в���ͷM2������������λ��
#define M2_RED_DROPBAN_C				40//���в�͵ײ���ͷʱM2�������ľ���
#define M2_RED_PICKUPTOPBAN_C			370//345//��ȡ������ͷʱM2�������ľ���
#define M2_RED_DROPTOPBAN_C			390//�Ӷ�����ͷʱM2���������ľ���
#define M2_RED_CATCHTOPBAN_COM_ON_C	10//����ͨ��ʱ��M2������ж�
 
 
 
 
 
 
 
 
 
 

/*--------------------����ִ�л������-----------------------*/
#define Action_M1_MOTOR_NAME    	"ACTION_M1"
#define Action_M1_MOTOR_CHANNEL		2
#define Action_M1_MOTOR_DIR_SEL		FIT
#define Action_M1_CODER_CHANNEL		10
#define Action_M1_PWM_DUTY_INIT		0
#define Action_M1_MAX_PWM_DUTY		2500
#define Action_M1_CODER_NUM_INIT	       0
#define Action_M1_POT_INIT			       0
#define Action_M1_POT_SEN                         1
#define Action_M1_COFF_POT_INIT		-0.0510594842f
#define Action_M1_VELT_INIT			       0
#define Action_M1_MAX_VELT                       400
#define Action_M1_COFF_VELT_INIT	       -8509.914049f//K_V_C_M2657//8498.58//1000.0*1000.0/7060.0  //K_V_C_M2657  r/min

#define Action_M2_MOTOR_NAME    	       "ACTION_M2"
#define Action_M2_MOTOR_CHANNEL		0
#define Action_M2_MOTOR_DIR_SEL		FIT
#define Action_M2_CODER_CHANNEL		7
#define Action_M2_PWM_DUTY_INIT		0
#define Action_M2_MAX_PWM_DUTY		2500
#define Action_M2_CODER_NUM_INIT	       0
#define Action_M2_POT_INIT			       0
#define Action_M2_POT_SEN                         1
#define Action_M2_COFF_POT_INIT		-0.042531710964f//-0.042531710964f������
#define Action_M2_VELT_INIT			       0
#define Action_M2_MAX_VELT                       110
#define Action_M2_COFF_VELT_INIT	      -7088.618494f//8498.58//1000.0*1000.0/7060.0  //K_V_C_M2657  r/min

#define Action_M3_MOTOR_NAME    	      "ACTION_M3"
#define Action_M3_MOTOR_CHANNEL		3
#define Action_M3_MOTOR_DIR_SEL		OPST//
#define Action_M3_CODER_CHANNEL		3
#define Action_M3_PWM_DUTY_INIT		0
#define Action_M3_MAX_PWM_DUTY		2500
#define Action_M3_CODER_NUM_INIT	       0
#define Action_M3_POT_INIT			       0
#define Action_M3_POT_SEN                         0
#define Action_M3_COFF_POT_INIT		1.0
#define Action_M3_VELT_INIT			       0
#define Action_M3_MAX_VELT                      150.0
#define Action_M3_COFF_VELT_INIT	      7603.59903f//8498.58//1000.0*1000.0/7060.0  //K_V_C_M2657  r/min





//ִ�е��PID����

/*����PID�������ݽṹ*/
/**************************************************************
typedef struct 
{
	FP32 fpP;		      	//����ϵ��KP
	FP32 fpI;		      	//����ϵ��KI
	FP32 fpD;		      	//΢��ϵ��KD
	FP32 fpE;			  	//ƫ��fpE(i)
	FP32 fpPreE;		  	//ƫ��fpE(i-1)
	FP32 fpPrePreE;			//ƫ����ۼ�
	FP32 fpU;	  			//����PID������
	FP32 fpELimit;  		//�����ַ�������ʱ�ļ���ƫ��
	FP32 ssULimit;			//����������ʱ������ֵ
	FP32 fpEDead;			//�������
}ST_PID;*////////////////////////////////////////////////////////////////
					     			             //P    	 I       		D       	E          PreE     	fpPrePreE    fpU         fpELimit                  ssULimit  		             fpEDead                   fpEthreshold
#define	Action_M1_VELT_PID_PARA		{11.0,      3.0,			0,		0.0,		0.0,		0.0,		   0.0,		800,	                   2500,				         0 ,                               200}
#define	Action_M1_POT_PID_PARA		{15.0, 	 0,			0,		0.0,		0.0,		0.0,		   0.0,		800,	                   Action_M1_MAX_VELT,	 0.0510594842*2 ,          20}
#define	Action_M2_VELT_PID_PARA		{24.0,	 3.0,			0.0,		0.0,		0.0,		0.0,		   0.0,		800,	                   2500,				         0,                                50}
#define	Action_M2_POT_PID_PARA		{3.0,	 0.0,			0.0,		0.0,		0.0,		0.0,		   0.0,		800,		            Action_M2_MAX_VELT,	  0.042531710964*2 ,      10}
#define	Action_M3_VELT_PID_PARA		{10.0,	 3.0,			0.0,		0.0,		0.0,		0.0,		   0.0,		800,		            2500,				         0 ,                               1000}
#define	Action_M3_POT_PID_PARA		{2.5,	0,			0.0,		0.0,		0.0,		0.0,		   0.0,		0.0,		            500,				         0 ,                               1000}


/*
typedef struct
{
	UCHAR8   ucSeries;        //·������
	SSHORT16 scOver;		//��������  1<<0--���̼������ӣ�0<<0--���̼������٣�1<<4--����ʱλ�ñջ���1<<5--����ʱPWMDuty=0,1<<6--����ʱ����PWMDuty =?,1<<7 ����ʱ����pid
	SINT32   siStartCode;     //StartCode
	SINT32   siEndCode;       //EndCode
	SSHORT16 ssStartV;      //StartVelocity (mm/s)
	SSHORT16 ssEndV;        //EndVelocity (mm/s)
}ST_ACTION_PATH;*/
/*ִ�е�����ƽṹ��  2010 12 28*/
#if 0
const ST_ACTION_PATH astActionPath[AMQ][20] =   
{
		/*SerialNumber		EndTypes	StartCode\EndCode			StartV\EndV   MaxV   AccUp/AccDown   EndPwmDuty*/ 
		//����������·��
	{
		//����
		{0,				 0x41,			0,28824,					20,20,		      70,        2,2,		1200}, //���������ȼ����˶�
		{1,				 0x10,			28824,-21000,					20,20,		      70,        2,2,		-1500}, //���������ȼ����˶�
	},
	{
		//�հ�
		{0,				 0x10,			0,-57500,					30,50,		      200,        3,6,		0}, //���������ȼ����˶�
		{1,				 0x10,			60000,0,					10,10,		      80,        0.8,0.8,		-200}, //���������ȼ����˶�
	},
	{
		//��ת
		{0,				 0x10,			0,-1300,					15,15,		      25,        0.5,0.5,		0}, //���������ȼ����˶�
//		{1,				 0x00,			-1250,-1350,					5,5,		      5,        0.5,0.5,		0}, //���������ȼ����˶�
//		{2,				 0x01,			-1350,-1150,					5,5,		      5,        0.5,0.5,		0}, //���������ȼ����˶�		
		{1,				 0x10,			-1300,-2000,					15,15,		      25,        0.8,0.8,		-500}, //���������ȼ����˶�
	},

		

};
#endif
                                                            /*SerialNumber		EndTypes	StartCode\EndCode			StartV\EndV\MaxV   AccUp/AccDown \EndPwmDuty*/
// ����
ST_ACTION_PATH g_stActionM1DymPathLow = {0,                          0x11,  0 , 10 ,                                            10, 10, 100,            100, 100 , 0};
ST_ACTION_PATH g_stActionM2DymPathLow = {0,                          0x11,  0 , 10 ,                                            10, 10, 100,            20, 20 , 0};

// ����
ST_ACTION_PATH g_stActionM1DymPathMid = {0,                          0x11,  0 , 10 ,                                            10, 10, 350,            105, 105 , 0};
ST_ACTION_PATH g_stActionM2DymPathMid = {0,                          0x11,  0 , 10 ,                                            10, 10, 100,            20, 20 , 0};

//����
ST_ACTION_PATH g_stActionM1DymPathTop = {0,                          0x11,  0 , 10 ,                                            10, 10, 350,            150, 150 , 0};
ST_ACTION_PATH g_stActionM2DymPathTop = {0,                          0x11,  0 , 10 ,                                            10, 10, 250,            35, 35 , 0};


//ʵ����ִ�л������

INSTANCE_MOTOR(Action_M1, stAction_M1, Action_M1_MOTOR_CHANNEL, Action_M1_CODER_CHANNEL, Action_M1_MOTOR_DIR_SEL);
INSTANCE_MOTOR(Action_M2, stAction_M2, Action_M2_MOTOR_CHANNEL, Action_M2_CODER_CHANNEL, Action_M2_MOTOR_DIR_SEL);
INSTANCE_MOTOR(Action_M3, stAction_M3, Action_M3_MOTOR_CHANNEL, Action_M3_CODER_CHANNEL, Action_M3_MOTOR_DIR_SEL);


#endif

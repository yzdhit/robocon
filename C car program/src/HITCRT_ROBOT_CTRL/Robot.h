#include "RobotConfig.h"
#include "math.h"


/*--------------------���õ��̵��-----------------------*/
#define BASE_WA_MOTOR_NAME		"BASE_WA"
#define BASE_WA_MOTOR_CHANNEL	5
#define BASE_WA_MOTOR_DIR_SEL	OPST
#define BASE_WA_CODER_CHANNEL	2
#define BASE_WA_PWM_DUTY_INIT	0
#define BASE_WA_MAX_PWM_DUTY	2500
#define BASE_WA_CODER_NUM_INIT	0
#define BASE_WA_POT_INIT		0
#define BASE_WA_POT_SEN         0
#define BASE_WA_COFF_POT_INIT	0
#define BASE_WA_VELT_INIT		0
#define BASE_WA_MAX_VELT        500
#define BASE_WA_COFF_VELT_INIT	8497.018963f





#define BASE_WB_MOTOR_NAME		"BASE_WB"
#define BASE_WB_MOTOR_CHANNEL	4
#define BASE_WB_MOTOR_DIR_SEL   OPST
#define BASE_WB_CODER_CHANNEL	1
#define BASE_WB_PWM_DUTY_INIT	0
#define BASE_WB_MAX_PWM_DUTY	2500
#define BASE_WB_CODER_NUM_INIT	0
#define BASE_WB_POT_INIT		0
#define BASE_WB_POT_SEN         0
#define BASE_WB_COFF_POT_INIT	0
#define BASE_WB_VELT_INIT		0
#define BASE_WB_MAX_VELT        500
#define BASE_WB_COFF_VELT_INIT	8497.620666f

#define BASE_WC_MOTOR_NAME		"BASE_WC"
#define BASE_WC_MOTOR_CHANNEL	1
#define BASE_WC_MOTOR_DIR_SEL 	OPST
#define BASE_WC_CODER_CHANNEL	5
#define BASE_WC_PWM_DUTY_INIT	0
#define BASE_WC_MAX_PWM_DUTY	2500
#define BASE_WC_CODER_NUM_INIT	0
#define BASE_WC_POT_INIT		0
#define BASE_WC_POT_SEN         0
#define BASE_WC_COFF_POT_INIT	0
#define BASE_WC_VELT_INIT		0
#define BASE_WC_MAX_VELT        500
#define BASE_WC_COFF_VELT_INIT	8498.342823f

//���̵��PID����
					     				               //P    	I       	D       	E       	PreE    	PrePreE    
#define	BASE_WA_VELT_PID_PARA			{12 ,	      4,		0.0,		0.0,		0.0,		0.0,		0.0,		500,	    0.0,				0,       1000}
#define	BASE_WB_VELT_PID_PARA			{12 ,	      4,		0.0,		0.0,		0.0,		0.0,		0.0,		500,	    0.0,				0,       1000}
#define	BASE_WC_VELT_PID_PARA			{12 ,	      4,		0.0,		0.0,		0.0,		0.0,		0.0,		500,	    0.0,				0 ,      1000}
#define	BASE_WA_POT_PID_PARA   BASE_WA_VELT_PID_PARA
#define	BASE_WB_POT_PID_PARA   BASE_WB_VELT_PID_PARA
#define	BASE_WC_POT_PID_PARA   BASE_WC_VELT_PID_PARA
//ֱ�ߵ���PID ����			 //P    	I       	D       	E       	PreE    	PrePreE    
#define NAV_LINE_ROT_PID_PARA		{7,	       0.0,		0.0,		0.0,		0.0,		0.0,		0.0,		800,	    3000,				0 ,       1000}		
#define NAV_LINE_TRVS_PID_PARA	{5,	       0.0,	       0.0,		0.0,		0.0,		0.0,		0.0,		800,	    3000,				0 ,       1000}		
#define NAV_LINE_VTC_PID_PARA		{5,	       0.0,		0.0,		0.0,		0.0,		0.0,		0.0,		800,	    3000,				0 ,       1000}	

//Բ������PID ����					//P    	I       	D       	E       	PreE    	PrePreE    
#define NAV_CIR_ROT_PID_PARA		{0.8,	0.0,		0.0,		0.0,		0.0,		0.0,		0.0,		100,	    0.0,				0 ,       1000}		
#define NAV_CIR_TRVS_PID_PARA		{0.8,	0.01,	0.0,		0.0,		0.0,		0.0,		0.0,		100,	    0.0,				0 ,       1000}		
#define NAV_CIR_VTC_PID_PARA		{0.4,	0.0,		0.0,		0.0,		0.0,		0.0,		0.0,		100,	    0.0,				0 ,       1000}	


/*--------------------------������λ��-------------------------*/
//�����˳�ʼ����	X	Y	Q
#define ROBOT_POT_INIT		{0,	0,	0}

/*---------------�����˵�����ز���----------------*/
						//		X(mm)	Y(mm)	Q(0.1��)
#define ROBOT_POT_SEN			{20,		20,		20}
							//	Vx		Vy		W
#define ROBOT_VELT_DES_INIT	{0,		0,		0}			//�����˵���ʱ��������������ٶ�
#define ROBOT_VELT_LIMIT		{400,		2300,		2000}		//�����˵������������ϵ��ٶ����� ��λ�ֱ��� �ȡ� cm/s





//ʵ��һ�������˶���
INSTANCE_ROBOT(stRobot,BASE_WA,BASE_WB,BASE_WC,BASE_WA_MOTOR_DIR_SEL,BASE_WB_MOTOR_DIR_SEL,BASE_WC_MOTOR_DIR_SEL)





 

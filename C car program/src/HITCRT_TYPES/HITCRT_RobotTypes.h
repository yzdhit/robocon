#ifndef	__HITCRT_ROBOTTYPES_H
#define	__HITCRT_ROBOTTYPES_H

#include "HITCRT_Types.h"
#include "Select.h"

/*����״̬*/
typedef enum
{
	FIRST,		//�״�����
	RUN			//�Ѿ�����
}EM_RUN_STATE;

/*�����ṹ��*/
typedef struct 
{
	SSHORT16 ssVx;
	SSHORT16 ssVy;
}ST_VECTOR;

/*����PID�������ݽṹ*/
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
	FP32 fpULimit;			//����������ʱ������ֵ
	FP32 fpEDead;			//�������
	FP32 fpEthreshold;           //���������������ֵ��������ʱ����Ѹ�ֵ��Ϊ�ϴ��һ����
}ST_PID;

/*������λ�˽ṹ��*/
typedef struct
{
	SSHORT16 ssPosX;	//������(��λ��mm)
	SSHORT16 ssPosY;	//������(��λ��mm)
	SSHORT16 ssPosQ;	//�����(��λ��0.1��)
}ST_ROBOT_POT;

/*����·���ṹ��*/
typedef struct 				
{
	UCHAR8   ucSeries;	//·�α��
	UCHAR8   ucType;	//·������
	SCHAR8   scOver;	//��������
	SSHORT16 ssStartX;	//StartX
	SSHORT16 ssStartY;	//StartY
	SSHORT16 ssEndX;	//EndX
	SSHORT16 ssEndY;	//EndY
	UCHAR8 	 ucMode;	//�ٶȼ�������
	USHORT16 usStartV;	//·����ʼ�ٶ�
	USHORT16 usEndV;	//·����ֹ�ٶ�
	SSHORT16 ssCenterX;	//CenterX
	SSHORT16 ssCenterY;	//CenterY
	SSHORT16 ssRadius;	//Radius
	SSHORT16 ssStartQ;  //���ĺ����
	SSHORT16 ssEndQ;    //�յ�ĺ����(0.1��)
	USHORT16 usMaxV;	//·�������
	FP32 fpAccUp;	//·�μ��ٶ�
	FP32 fpAccDown;	//·�μ��ٶ�
	FP32 fpDisRatio;//·�ε���ת·��ռ��·�εı������������ҷ���
}ST_PATH;				//����·���ṹ��

/*ִ�е�����ƽṹ�壨�¼ӵģ�*/
typedef struct
{
	UCHAR8   ucSeries;        //·������
	SSHORT16 scOver;		//��������
	SINT32   siStartCode;     //StartCode
	SINT32   siEndCode;       //EndCode
	SSHORT16 ssStartV;      //StartVelocity (mm/s)
	SSHORT16 ssEndV;        //EndVelocity (mm/s)
	SSHORT16 ssMaxV;	//·�������
	FP32 fpAccUp;	//·�μ��ٶ�
	FP32 fpAccDown;	//·�μ��ٶ�
	SSHORT16 EndPwmDuty;
//	SINT32   siUpCode;     //����ת�����ٵ�����ֵ
//	SINT32   siDownCode;       //����ת�����ٵ�����ֵ
}ST_ACTION_PATH;


/*ʱ�����ṹ��*/
typedef struct
{
	UINT32 uiPreTime;		//ǰһʱ��
	UINT32 uiCurTime;		//��ǰʱ��
	UINT32 uiIntlTime;		//���ʱ��
	EM_RUN_STATE emRunState;//
}ST_INTL_TIME;

/*���̹����ٶȽṹ��*/
typedef struct
{
	SINT32	siPreCodeNum;	//��һ�����̶���
	SINT32 	siCurCodeNum;	//��ǰ���̶���
	SINT32	siDetCodeNum;	//���̶�����ֵ
	ST_INTL_TIME	stIntlTime; 	//ʱ����
	EM_RUN_STATE  emRunState;
}ST_VELT_CODER;
//����˫�涯ȫ���־��뷴���ṹ��
typedef struct
{
	SINT32 siCoderA;		//�涯��A��������������
	SINT32 siLastCoderA;	//�涯��A������������һ�μ���
	SINT32 siCoderB;		//�涯��B��������������
	SINT32 siLastCoderB;	//�涯��B������������һ�μ���
	SINT32 siPosQ;			//���κ����
	SINT32 siLastPosQ;		//��һ�κ����
	FP32 fpPosQFix;			//�ֲ�����ϵ����Ǿ�ƫ
	FP32 fpKCodeToDegA;		//�涯��A������������������ת��ϵ��
	FP32 fpKCodeToDegB;		//�涯��B������������������ת��ϵ��
	FP32 fpOMNI_WHEEL_R;	//�涯��AB�İ뾶
	FP32 fpOMNI_Position_L;	//�涯�ְ�װλ�������ȫ���ֵ��̵��е�λ��
}ST_FBOMNILENTH_STRUCT;	    
typedef enum
{
	MOTOR_BREAK,
	MOTOR_OPEN_LOOP_CTRL,
	MOTOR_PATH_CLOSE_LOOP_CTRL,
	MOTOR_POS_CLOSE_LOOP_CTRL,
	MOTOR_VELT_CLOSE_LOOP_CTRL,
	MOTOR_STOP_QUICKLY,
	MOTOR_STOP_SLOWLY,
	MOTOR_OTHER
}EM_MOTOR_STATE;
typedef enum
{
	PATH_END,
	PATH_RUN
}EM_PATH_STATE;

typedef enum
{
	/***********�����˸�λʱʹ��*****************/
	REPOSIT_INIT,
	REPOSIT1,
	REPOSIT2,
	REPOSIT3,
	REPOSIT4,
	REPOSIT5
	/****************************/	
}EM_ROBOT_REPOSIT;

typedef enum
{
	/***********�����˸�λʱʹ��*****************/
	SELFCHECK_INIT,
	SELFCHECK1,
	SELFCHECK2,
	SELFCHECK3,
	SELFCHECK4,
	SELFCHECK5
	/****************************/	
}EM_ROBOT_CHECK;
typedef enum
{
	NO_ACTION,
	ALL_STOP,
	ALL_BREAK,
	INIT,
       /*Ԥ���������񣬹�����*/
	TASK_START_1,
	TASK_START_2,
	TASK_START_3,

	TASK_INSERT_1,
	TASK_INSERT_2,
	TASK_INSERT_3,

	TASK_INSERT_4,

	TASK_INSERT_5,//����������ʱ��ʹ��
	TASK_INSERT_6,//����������ʽ4��ʱ��ʹ��
	TASK_INSERT_7,
	TASK_INSERT_8,
	TASK_INSERT_9,
	TASK_INSERT_10,
	

	TASK_TOPAIR1,
	TASK_TOPAIR2,
	TASK_TOPAIR3,
	TASK_TOPAIR4,
	TASK_TOPAIR5,
	TASK_TOPAIR6,
	TASK_TOPAIR7,

	
	
	TASK1,
	TASK2,
	TASK3,
	TASK4,
	TASK5,
	TASK6,
	TASK7,
	TASK8,
	TASK9,
	TASK10,
	TASK11,
	TASK12,
	TASK13,
	TASK14,
	TASK15,
	TASK16,
	TASK17,
	TASK18,
	TASK19,
	TASK20,
	TASK21,
	TASK22,
	TASK23,
	TASK24,
	TASK25,
	TASK26,
	TASK27,
	TASK28,
	TASK29,
	TASK30,
	TASK31,
	TASK32,
	
	/*Ԥ������������β*/
	TASK_END_1,
	TASK_END_2,
	TASK_END_3,
	
	OVER
}EM_ROBOT_TASK;
typedef struct
{
	SSHORT16		ssPwmDuty;			//���PWMռ�ձ�
	USHORT16		usMaxPwmDuty;		//����������ռ�ձ�
	SINT32			siCoderNum;			//��������
	FP32			fpPotFB;			//����λ��Position��feedback
	FP32			fpPrePotFB;			//��һ�η���λ��
	FP32			fpPotDes;			//����λ�ã�desired
	FP32            fpPotSen;           //����λ��ƫ��
	FP32			fpCoffPot;			//ʵ��λ������������ת��ϵ��
	FP32			fpVeltFB;			//�����ٶ�Velocity
	FP32			fpVeltDes;			//�����ٶ�
	FP32            fpMaxVelt;			//������Ƶ�����ٶ�
	FP32			fpCoffVelt;			//ʵ���ٶ������̹����ٶȵ�ת��ϵ��					
	ST_VELT_CODER	stVeltCoder;		//���̹����ٶȽṹ��
	ST_PID			stVeltPid;				//����ٶȱջ�����PID�ṹ��
	ST_PID			stPotPid;				//���λ�ñջ�����PID�ṹ��
	void			(*TurnMotor)(SSHORT16 ssPwmDuty);		//����������ƺ���
	ST_ACTION_PATH*	pstCurPath;
	ST_ACTION_PATH*	pstPrePath;
	UCHAR8 			ucName[12];			//�������
	UCHAR8			ucMotorChan;		//���ͨ��
	UCHAR8			ucCoderChan;		//����ͨ��
	EM_MOTOR_STATE	emState;			//�������״̬
	EM_PATH_STATE	emPathState;
}ST_MOTOR_CTRL;

//�������ٶȽṹ��
typedef struct
{
	FP32 fpVx;		//X�����ٶ�
	FP32 fpVy;		//Y�����ٶ�
	FP32 fpW;		//��ת���ٶ�
}ST_ROBOT_VELT;

#if	BASE_TYPE
typedef struct
{
	ST_PID	stPidRot;				//��תPID��Rotation
	ST_PID	stPidTrvs;				//����PID��Transverse
	ST_PID	stPidVtc;				//����PID��Vertical
}ST_NAV_PID;
#else
typedef struct
{
	ST_PID	stPidNav;
}ST_NAV_PID;
#endif


typedef enum
{
	ROBOT_STOP,
	ROBOT_MOVE
}EM_ROBOT_STATE;



typedef enum
{
	NAV_LINE,
	NAV_CIR,
	NAV_OFF,
	NAV_MANUAL,
	NAV_ERROR,
	NAV_NULL,
	NAV_BASE_DIRECT,
	NAV_XYQ
}EM_NAV_STATE;
typedef enum
{
	BASE_OPEN_LOOP_CTRL,
	BASE_CLOSE_LOOP_CTRL,
	BASE_BREAK,
	BASE_STOP_QUICKLY,
	BASE_STOP_SLOWLY,
	OTHER
}EM_BASE_STATE;


typedef enum
{
	RED_FIELD,
	RED_FIELD_A_RESTART,
	RED_FIELD_S2_RESTART,
	RED_FIELD_L2_RESTART1,
	RED_FIELD_L2_RESTART2,
	RED_FIELD_L2_RESTART3,
	RED_FIELD_L2_RESTART4,
	BLUE_FIELD,
	BLUE_FIELD_A_RESTART,
	BLUE_FIELD_S2_RESTART,
	BLUE_FIELD_L2_RESTART1,
	BLUE_FIELD_L2_RESTART2,
	BLUE_FIELD_L2_RESTART3,
	BLUE_FIELD_L2_RESTART4,

	TEST_TOPAIR
	
}EM_ROBOT_TACTIC;


typedef enum
{
	TOP_SPEED,
	MID_SPEED,
	LOW_SPEED
}EM_SPEED_MODE;

typedef struct
{
	ST_MOTOR_CTRL	      stBaseWaMotor;
	ST_MOTOR_CTRL	      stBaseWbMotor;
	ST_MOTOR_CTRL	      stBaseWcMotor;
	ST_NAV_PID		      stNavPidLine;
	ST_NAV_PID		      stNavPidCir;
	ST_ROBOT_POT	      stPot;
	ST_ROBOT_POT           stPotDes;
	ST_ROBOT_POT	      stPotSen;
	ST_ROBOT_VELT	      stVeltDes;
	ST_ROBOT_VELT	      stVeltLimit;
	ST_PATH*		      pstCurPath;
	ST_PATH*		      pstPrePath;
	EM_ROBOT_STATE	      emState;
	EM_NAV_STATE	      emNavState;
	EM_PATH_STATE	      emPathState;
	EM_BASE_STATE 	      emBaseState;
	EM_ROBOT_TASK   	  emRobotTask;
	EM_ROBOT_TACTIC        emRobotTactic;
	EM_SPEED_MODE            emSpeedMode;
	EM_ROBOT_REPOSIT	emRobotReposit;
	EM_ROBOT_CHECK	emRobotCheck;
}ST_OMNI_MOBILE_ROBOT;

typedef struct
{
	SINT32    siCoderA;		//�涯��A��������������
	SINT32    siLastCoderA;	//�涯��A������������һ�μ���
	SINT32    siCoderB;		//�涯��B��������������
	SINT32    siLastCoderB;	//�涯��B������������һ�μ���
	FP32      fpPosQ;	    //���κ����(��λ: 0.1��)
	FP32      fpLastPosQ;	//��һ�κ����
	FP32      fpPosX;	    //������(��λ��mm)
	FP32      fpLastPosX;   //��һ�κ�����
	FP32      fpPosY;	    //������(��λ��mm)
	FP32      fpLastPosY;   //��һ��������
	
}ST_DOUBLE_OMNI_LOCATION;	    //����˫�涯ȫ���־��뷴���ṹ��
//�Լ�����ĸ��ٱ����õĽṹ��
typedef struct
{
	SSHORT16              ssPotX;
	SSHORT16              ssPotY;
//	SSHORT16              ssPotQ;
	FP32                  fpE;//�����ƫ��
	FP32                  fpDis;//�ܾ���
	FP32                  fpK;//ϵ��
}ST_VAR_WATCH;


/*---------------PS2�ֱ�------------------*/
typedef struct
{
	USHORT16 usReserved;
	USHORT16 usJsLeft;
	USHORT16 usJsRight;
	USHORT16 usJsState;
}ST_JS_FPGA;


typedef struct
{
	UCHAR8 ucLeftSpeedCnt;
	UCHAR8 ucLeftSpeedLastCnt;
	UCHAR8 ucRightSpeedCnt;
	UCHAR8 ucRightSpeedLastCnt;
	UCHAR8 ucLeftPwmCnt;
	UCHAR8 ucLeftPwmLastCnt;
	UCHAR8 ucRightPwmCnt;
	UCHAR8 ucRightPwmLastCnt;
	UCHAR8 ucSpeedCntLimit;
	UCHAR8 ucPwmCntLimit;
	UCHAR8 ucErrFlag;
	UCHAR8 ucErrSource;
	FP32 fpSpeedLimit;
	FP32 fpPwmLimit;
	
	
}ST_DETECT_DIFF_BASE;

typedef struct
{
	SSHORT16 ssRate;
	SSHORT16 ssGryo;
}ST_GRYO;


//�״���صĶ���
typedef struct
{
  FP32 Angle[2];
  SSHORT16 Distance[2];
} ST_POS_INFO;

typedef struct
{
	USHORT16 usLeftDis;
	USHORT16 usRightDis;
}ST_CALIBRATE_POS;

typedef struct {
	SINT32 NumLines;  //  ֱ�߶�����
	FP32 Rho1;	   //  ��һ�������
	FP32 Rho2;	   //  �ڶ���ֱ�����
	FP32 Dis1;    //  ��һ��ֱ�ߵ�ԭ����� �� ֱ�� y = a * x + b �� b �ķ�����dis�ķ���
	FP32 Dis2;    //  �ڶ���ֱ�ߵ�ԭ�����
}ST_VDL;

											
#endif


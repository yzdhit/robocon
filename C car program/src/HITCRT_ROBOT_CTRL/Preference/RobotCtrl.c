/*****************************************NEW C���������2012.6.14*****************************************************************************************************************
��Ȩ������HITCRT(�����󾺼�������Э��)
�ļ�����RobotCtrl.c
����޸����ڣ�2011.07.07
�汾��1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
ģ�����������������س���������
�����б� 

----------------------------------------------------------------------------------------------------------------------------------------------------------
�޶���¼��
����        ʱ��            �汾     ˵��
��ΰ        2011.07.07      1.0      ���ֽ�����ģ��
**********************************************************************************************************************************************************/

#include "HITCRT_Types.h"
#include "ucos_ii.h"
#include "Select.h"
#include "stdio.h"
#include "HITCRT_API.h"
#include "MotorCtrl.h"
#include "Robot.h"
#include "Action_Motor.h"
#include "OS.h"
#include "Path.h"
#include "HITCRT_Coff.h"
#include "PrivateFunction.h"
#include "cordic.h"
#include "Location.h"
#include "stm32f4xx.h"
#include "hal.h"
#include "PC_ARM.h"
#include "HITCRT_Algorithm.h"
#include "HITCRT_Coff.h"
#include "CAN_API.h"
#include "IrComm.h"
#include "RobotCtrl.h"
#include "Navigation.h"
#include "FlashSave.h"

ST_OMNI_MOBILE_ROBOT stRobot = 	SET_ONMI_ROBOT_PARA(BASE_WA,BASE_WB,BASE_WC);
ST_MOTOR_CTRL stAction_M1, stAction_M2, stAction_M3;

ST_JS_FPGA stJSValue;
volatile USHORT16 usKeyValue;

UCHAR8 uc_TestAirFlag=0;//�������ױ���
UCHAR8 uc_TestAirStartFlag=0;//�������ױ���
UINT32 g_uiTTestAirTime = 0; //�������ױ���
UINT32 g_uiConstTime = 2000000; //�������ױ���

/*---------------------------ģʽѡ����صı���------------------------------------*/
UCHAR8  g_ucCurModeSel = 0;
UCHAR8  g_ucModeCnt = 7;
const UCHAR8  g_ucTotalMode = 9;

/*---------------------------��λ��صı���------------------------------------------*/

ST_DOUBLE_OMNI_LOCATION stDoubleOMNI = {0,0,0,0,0,0,0,0, 0, 0};
//Ѳ����ص�ֵ

UCHAR8 g_ucFliterLineValue[12]={75,75,75,75,75,75,75,75,75,75,75,75};
UCHAR8 g_ucLineValue = 0;
const UCHAR8 g_ucLineMidValue = 80;
const UCHAR8 g_ucLineMinValue  = 10;
const UCHAR8 g_ucLineMaxValue = 140;
FP32 g_fpPostionXTemp=0;
//�������ݵ�����,Ҫ��������ʱ����
FP32 g_fpModifyAngle = 0;
FP32 g_fpGyroAngle = 0;
FP32 g_fpPotTemp =0;

//mems���ݵ�ֵ
SSHORT16 g_ssMemsGryo = 0;


/*------------------------����׷��ʱ����ر���------------------------------------*/

UCHAR8 g_ucOnceRunFlag = 0;  //��������ִֻ��һ�εĴ���
UCHAR8 g_ucLoopRunFlag = 1;  // ��������һֱִ�еĴ���
UCHAR8 g_ucJumpRunFlag = 0; //����������ת�Ĵ���
UCHAR8 g_ucTaskFlag=0;//�������� ����Ҫ�ı�����

UCHAR8 g_ucTaskCnt = 0;     // �����еļ�����

UINT32 g_uiTaskTime1 = 0;   //�����е�ʱ��
UINT32 g_uiTaskTime2 = 0;
SINT32 g_siTaskDeltaTime = 0;

//UCHAR8 g_ucAllContinueFlag = 1; //Ĭ��ȫ���Զ���ɣ��ǵ���ִ�У�Ϊ0ʱ������ִ��
UCHAR8 g_ucAllContinueFlag =0;

/*--------------------------�Զ�������صı���-------------------------------------*/
//NAV_XYQ��صı���
SINT32 g_siRobotXDesShadow = 0;
SINT32 g_siRobotYDesShadow = 0;
SINT32 g_siRobotQDesShadow = 0;

UCHAR8 g_ucRobotXEnable = 0;
UCHAR8 g_ucRobotYEnable = 0;
UCHAR8 g_ucRobotQEnable = 0;

SINT32 g_siRobotXDes = 0;
SINT32 g_siRobotYDes = 0;
SINT32 g_siRobotQDes = 0;

SSHORT16 g_ssPotXTemp=0;
SSHORT16 g_ssPotYTemp=0;
SSHORT16 g_ssPotQTemp=0;

/*---------------------------�ֶ����Ƶ���ر���------------------------------------*/

//�ֶ����ƻ�����ʱ��Ŀ���ٶ�
ST_ROBOT_VELT g_stRobotDesV = {0,0,0};
SINT32 g_siDirAngle = 0;

/*----------------------------ִ�е�������Ķ���----------------------------------*/
//ִ�е����Ԥ���ٶȵ�Ӱ�ӣ���ִ�е�����ٶȱջ�ģʽ��ʹ��
FP32 g_fpActionM1VeltDesShadow = 0;
FP32 g_fpActionM2VeltDesShadow = 0;
FP32 g_fpActionM3VeltDesShadow = 0;

//ִ�е����λ�ñջ�������λ��
FP32  g_fpActionM1PotFB = 0;
FP32  g_fpActionM2PotFB = 0;

//���Ƶ��·���ջ�ʱ��ÿ�ε�����,�����ֶ�����
FP32   g_fpM1DeltaPos = 40;//��ֱ�������
FP32   g_fpM1DeltaPos1 = 5;//��ֱ�������
FP32   g_fpM1DeltaPos2 = 20;//��ֱ�������
FP32   g_fpM2DeltaPos = 80;//�ϲ�ˮƽ���
FP32   g_fpM2DeltaPos2 = 40;//��ֱ�������


UCHAR8 g_ucWatchComTemp;//ͨ�Ź۲����
/*--------------------------M��ͨ�ŵı�������--------------------------------------*/
UCHAR8 g_ucMC_M1UpCnt = 0;        //���1���϶�
UCHAR8 g_ucMC_M1DownCnt = 0;   //���1���¶�
UCHAR8 g_ucMC_M2OutCnt = 0;      //���2������
UCHAR8 g_ucMC_M2InCnt =0;         //���2������
UCHAR8 g_ucMC_CatchCnt = 0;       //ץ���� 
UCHAR8 g_ucMC_ReleaseCnt = 0;   //�Ű���
UCHAR8 g_ucMC_RotFitCnt = 0;      //˳ʱ����ת
UCHAR8 g_ucMC_RotOpstCnt = 0;   //��ʱ����ת
UCHAR8 g_ucMC_AutoCnt = 0;        // �Զ���
UCHAR8 g_ucMC_Rot_ValveCnt = 0;        // �Զ���ת����
UCHAR8 g_ucMC_InitCnt=0;		//��ʼ������
UCHAR8  g_ucMCComStatusFlag=0;//MCͨ���Ƿ�ر�����

/******************************A��ͨ�ű�������***************************************************************/
UCHAR8 g_ucAC_LeaveStartCnt = 0;//A���뿪���C����ͨ��
UCHAR8 g_ucAC_L2Cnt=0;  //  A������L2֮��Ķ�C����ͨ��
UCHAR8 g_ucAC_BasketCnt=0; // C�ϵ����A����ͨ��
 UCHAR8 g_ucAC_LiftUpCnt=0;//C��A�����Ƿ������ͨ��

const UCHAR8 g_ucMC_CntTime = 3;

UCHAR8  g_ucACComStatusFlag=0;//ACͨ���Ƿ�رյ�����

//M���ֶ�����C�ı�־��

/*----------------------------Һ�������Ķ���----------------------------------------*/
// ����Һ���ķ�ҳ
UCHAR8 g_ucCurLcdPage = 0;
const UCHAR8 g_ucTotalLcdPage = 30;

/**********************************·������Ĭ�϶��Ǻ쳡����*************************************************************/
ST_PATH  *g_pstL2Path=&stNavPathRed[0];
ST_PATH  *g_pstCatchBanPath=&stNavPathRed[1];
ST_PATH  *g_pstReleaseBanPath=&stNavPathRed[2];
ST_PATH  *g_pstGoBasketPath=&stNavPathRed[24];//Ĭ���Ǻ쳡

/*************************************************************************************************/

/***********************************flash�洢��ر���************************************************************/

SINT32 g_siTestFlashRead = 0;
UCHAR8 g_ucFlashFlag=0;
UCHAR8 g_ucFlashWrongNum=0;
SINT32 g_siFlashValue[100]={-1};
/*--------------------------------�Ӿ���صĶ���----------------------------*/
UINT32 g_uiVisionUpdateTime = 0;
UCHAR8  g_ucVisionUsedCnt = 0;
FP32 g_fpVisionValue = 0;
FP32 g_fpVisionPotY = 0;

/******************************������������ر���**************************************/
UCHAR8 g_ucKeyFlag=1;//�������������ñ�������0����쳡��1��������

/******************************************************************/

/************************����������Ƿ��ǵ�һ�����еı���*****************************************/
UCHAR8 g_FirstRun=0;//0���������δ���У�1���������������
/***************************************************************/

/*********************AC���ʱ���Բ�ͨ�ŵı�־��*******************************************/
UCHAR8 g_ACFlag1=0;//AC��A���뿪ʱ����ͨ��ͨ�ŵı�־��
UCHAR8 g_ACFlag2=0;//AC��ʼͨ���Ƿ�ɹ���־��
UCHAR8 g_ACFlag3=0;//AC��L2�ǵ�����ͨ��ͨ�ŵı�־��
UCHAR8 g_ACFlag4=0;//AC��L2�ǵ�������½� �ı�־��
UCHAR8 g_ACFlag5=0;//C����L2�ǵ�ʱ�����Ƕȵı�־��
UCHAR8 g_CFlag6=0;//C����ȡ����ͷ֮�������߶ȱ�־��
UCHAR8 g_CFlag7=0;//C������ȡ������ͷ��������ת��־��
UCHAR8 g_CFlag8=0;//C������ȡ������ͷ��������ת��־��

UCHAR8 g_AC_NoCom_On_Off=0;//����1��ʾ��ͨ��ͨ�ŵ�ģʽ����������0��ʾֻͨ��ͨ��
SINT32  g_M3TempCoder=0;//C���ϵ�ʱ��Ҫ���ջ�������

UCHAR8 g_SelectState_Electric_On=0;//�Ƿ�ѡ������������״̬��

UCHAR8 g_ucFlashLoad=0;//�Ƿ����Flash��־����0��ʾ�����أ�1��ʾ����Flash

UCHAR8  g_ucUseMemsGryo=0;//�Ƿ�����Mems���ݣ�1��������Mems���ݣ�0��������ģ������

UINT32 g_uiCPUUsage = 0;
UINT32 g_uiCPUUsageMax = 0;

UINT32 g_TimeTemp=0;
UINT32 g_fpValveCoff=25;
int main(void )
{
	OSInit();				//UCOS INIT~
 	ChipHalInit();
	ChipOutHalInit();
	CREATE_OS_TASK(InitTask);		//������ʼ������
	OSStart();
	while(1)
	{	
	}
}

/****************************************************************************************************
�������ƣ�InitTask()
�����ܣ����豸�Լ�һЩ�����˵Ĳ������г�ʼ�������������˿��Ƶ������������
****************************************************************************************************/
void InitTask (void *pdata)
{
	pdata = pdata;
       OSTimeDly(500);//
	INIT_FPGA(); //��ʼ��fpga 
	CODER_INIT();  //��������ͨ����
 	PWM_MOTOR_INIT();//���е����pwm����
 	PWM_SERVO_INIT();//���ж����pwm����

	OSTimeDly(100);//
	/*������������*/
	CREATE_OS_TASK(BaseCtrlTask);
	CREATE_OS_TASK(ActionMotorCtrlTask);
	CREATE_OS_TASK(LcdDispTask);
	CREATE_OS_TASK(ReadKeyTask);
	CREATE_OS_TASK(NavTask);
	CREATE_OS_TASK(DispatchTask);
	CREATE_OS_TASK(DetectTask);
	CREATE_OS_TASK(CanTask);

	

	while (1)
	{
		OSTimeDly(500);
		ARM_LED_FLASH();
		//CONFIG_TO_LINE(0);
	}
}

/****************************************************************************************************
�������ƣ�DispatchTask ()
�����ܣ���������
****************************************************************************************************/


void DispatchTask (void *pdata)
{
       
	stRobot.emNavState = NAV_OFF;//Ĭ�ϵ����ر�
	
	while(1)
	{

		//��λ������
		DoubleVerticalOMNILocateEx1(&stDoubleOMNI, &stRobot);
		//DoubleVerticalOMNILocate(&stDoubleOMNI, &stRobot);
		//DoubleVerticalOMNILocatenew(&stDoubleOMNI, &stRobot);
		
		switch(stRobot.emRobotTask)
		{
		
			case NO_ACTION:
				/*ʲôҲ����*/
				break;
			case ALL_STOP:
				stAction_M1.emState = MOTOR_STOP_SLOWLY;
			       stAction_M2.emState = MOTOR_STOP_SLOWLY;
				stAction_M3.emState = MOTOR_BREAK;
				stRobot.emBaseState = BASE_STOP_SLOWLY;
				stRobot.emNavState = NAV_NULL;				
				break;
			case ALL_BREAK:
				stAction_M1.emState = MOTOR_BREAK;
			       stAction_M2.emState = MOTOR_BREAK;
				stAction_M3.emState = MOTOR_BREAK;
				stRobot.emBaseState = BASE_BREAK;
				stRobot.emNavState = NAV_OFF;
				
				break;
			case INIT://��ʼ����һ����λ��
				if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
					stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathLow,30);
					stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
					
					stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
					stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
					SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,10*M2COFF_POT);

					stAction_M3.fpVeltDes=100;
					stAction_M3.emState = MOTOR_VELT_CLOSE_LOOP_CTRL;
					CLOSE_VALVE(FRONT_LEG_VALVE);
					CLOSE_VALVE(MID_CATCH_VALVE);
					CLOSE_VALVE(MID_LIFTUP_VALVE);
					CLOSE_VALVE(TOP_CATCH_VALVE);
					CLOSE_VALVE(ROT_VALVE);

				}

				if(g_ucLoopRunFlag == 1)
				{
					// �ڴ����ѭ��ִ�д���
                                     

					// �ڴ��ж��Ƿ���ת
					if(stAction_M1.emPathState==PATH_END)
					{
						g_ucJumpRunFlag = 1;
						
					}

				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{
						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
						g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//������ת�Ĳ���
						stRobot.emRobotTask = TASK_START_1;	
					}
				}
				
				break;
			case TASK_START_1://M��ץס֮���ͨ�ţ��ջ�ǰ����֧�Ÿ�
				 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						g_fpModifyAngle = g_fpGyroAngle; //������ǰ���ݽǶȣ����ں���MȡC,C���������ж���
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						stAction_M1.pstCurPath = &g_stActionM1DymPathLow;
	 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathLow,M1M_PICKUP_C*M1COFF_POT);
						stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						g_ucMC_M1DownCnt = 0;

						stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
						SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,10*M2COFF_POT);

						
						stAction_M3.fpVeltDes=0;
						stAction_M3.emState = MOTOR_VELT_CLOSE_LOOP_CTRL;
						while(IrComm1IsDataInRxBuf())
							{
								IrComm1GetRxBufDat();
							}
					}

				if(g_ucLoopRunFlag == 1)
				{
                                  // if(IS_SWITCH_ON(RIGHT_LIGHT_SWITCH)||IS_SWITCH_ON(LEFT_LIGHT_SWITCH))//���ϴ˴��жϣ�ֻ�е�M������C����ʱ�򣬲Ŵ�ͨ��
                                          if(1)   //����Ӵ�ʱ�����ز������ղ���ͨ��
                                   	{
							while(IrComm1IsDataInRxBuf())
							{
								switch(IrComm1GetRxBufDat())
								{
									case MC_PICKUP_CMD:
										g_ucMC_M1DownCnt++;
									break;
									default:
									break;
								}

							}
                                   	}
					// �ڴ��ж��Ƿ���ת
					if((g_ucMC_M1DownCnt>g_ucMC_CntTime)||(abs(stRobot.stPot.ssPosQ)>200))//�˴�����ͨ�������˵ĽǶ����жϣ��Ƿ񽫳��ļ���������
						{
	                                       
							g_ucJumpRunFlag = 1;
							g_ucAllContinueFlag =1;
							
						}



					/********************************************************************************************************/	
					 //�˴�Ϊ��΢���߶�
						  if(KEY_F5(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
						        stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						        g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
							 SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathLow, g_fpActionM1PotFB + g_fpM1DeltaPos1);
						       
							}	
					        

						  if(KEY_F4(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
						        stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						        g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
							 SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathLow, g_fpActionM1PotFB - g_fpM1DeltaPos1);
						       
							}	
/*************************************************************************************************************/

							 

				if(JS_L1(JOYSTICK_RESERVED)&&(stAction_M1.emPathState==PATH_END))
					{
						g_ucJumpRunFlag = 1;
						g_ucAllContinueFlag =1;
					}

					
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					
							// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
							{

								g_ucOnceRunFlag = 0;
								g_ucLoopRunFlag = 1;
								g_ucJumpRunFlag = 0;
								g_ucTaskCnt = 0;
								g_uiTaskTime1 = 0;
								g_uiTaskTime2 = 0;
								g_siTaskDeltaTime = 0;
			/*****************************�Ĺ�***********************************************/
								//������ת�Ĳ���
								//stRobot.emRobotTask = TASK14;
								stRobot.emRobotTask =TASK_INSERT_2;
			/*******************************************************************************/
							}    
				}
			break;
			
			case TASK_INSERT_2://�ȴ�AC�뿪A�Ĳ���
				 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
									   
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						g_ACFlag1=0;
						g_ACFlag2=0;
						g_ucAC_LeaveStartCnt=0;
						stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
	 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1M_PUTDOWN_C*M1COFF_POT);//M����C֮��C���µĸ߶�
						stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

						stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
						SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,10*M2COFF_POT);
						
						while(IrComm1IsDataInRxBuf())
							{
								IrComm1GetRxBufDat();
							}
					}

				if(g_ucLoopRunFlag == 1)
				{
					// �ڴ����ѭ��ִ�д���
					if(stAction_M1.emPathState==PATH_END)
					{
							OPEN_VALVE(FRONT_LEG_VALVE);
					}

					while(IrComm1IsDataInRxBuf())
						{
						switch(IrComm1GetRxBufDat())
							 {
								case AC_AC_LEAVE_START_CMD  :
									g_ucAC_LeaveStartCnt++;
									break;
								default:
									break;

							}
						}
					

					if((g_ucAC_LeaveStartCnt>g_ucMC_CntTime)&&(g_ACFlag2==0))//�˴��ٴξ����������꣬��Ϊ��ͨ��ͨ�ž������ĽǶȿ��ܲ�׼ȷ,����ֻ����һ��
						{
							g_fpModifyAngle = g_fpGyroAngle; 
							stRobot.stPot.ssPosQ = 0;
							g_ucAC_LeaveStartCnt=0;
							g_ACFlag2=1;//AC��ʼͨ���Ƿ�ɹ�������
							g_ucJumpRunFlag = 1;
							g_ucAllContinueFlag = 1;//�Զ�
						}

					if(g_ACFlag2==1)
						{
							/*****************ACͨ���Ƿ�ɹ���־��**************************/

							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							/*************************************************************************/
						}
					
					if((g_ucAC_LeaveStartCnt>g_ucMC_CntTime)||JS_R1(JOYSTICK_RESERVED))
						{
							g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������
							stRobot.stPot.ssPosQ = 0;
							g_ACFlag2=1;//AC��ʼͨ���Ƿ�ɹ�������
							g_ucJumpRunFlag = 1;
							g_ucAllContinueFlag = 1;//�Զ�
						}
					/***********************����ͨ��Ҳ����A���½��ĳ���*********************************************/
					if((GET_LINE_VALUE(LINE_CHAN_0)<145)
						&&(GET_LINE_VALUE(LINE_CHAN_0)>0)
						&&IS_SWITCH_OFF(LEFT_LIGHT_SWITCH)&&(g_AC_NoCom_On_Off==1))
						{
							if(g_ACFlag1==0)
								{
									g_uiTaskTime1=GetCurTime();
									g_ACFlag1=1;
								}
							if((GetCurTime()-g_uiTaskTime1)>2500000)
								{
									// �ڴ��ж��Ƿ���ת
									g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������
									stRobot.stPot.ssPosQ = 0;
									g_ACFlag1=1;
									g_ucJumpRunFlag = 1;
									g_ucAllContinueFlag = 1;//�Զ�
								}
						}
					else
						{
							g_ACFlag1=0;
						}
	
				}

				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
						g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//������ת�Ĳ���
						stRobot.emRobotTask = TASK_INSERT_3;
					}
				}
				
				break;
			case TASK_INSERT_3://A����Ҫ���ﵺ��ʱ���������,�˴�Ҳ��AC��A�����������
				if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						
									   
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						g_ucAC_LeaveStartCnt=0;
						g_ucAC_LiftUpCnt=0;
						g_uiTaskTime1=GetCurTime();
						g_ACFlag3=0;
						
						OPEN_VALVE(FRONT_LEG_VALVE);
						CLOSE_VALVE(MID_CATCH_VALVE);
						CLOSE_VALVE(MID_LIFTUP_VALVE);
						CLOSE_VALVE(TOP_CATCH_VALVE);
						CLOSE_VALVE(ROT_VALVE);

						stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
	 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1A_DOWN_C*M1COFF_POT);//��A�����½��ĸ߶�
						stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

						stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
						SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,10*M2COFF_POT);

						stAction_M3.fpVeltDes=0;
						stAction_M3.emState = MOTOR_VELT_CLOSE_LOOP_CTRL;
						
						stRobot.emNavState=NAV_NULL;
					       stRobot.emBaseState=BASE_STOP_QUICKLY;
						
						while(IrComm1IsDataInRxBuf())
							{
								IrComm1GetRxBufDat();
							}
					}

				if(g_ucLoopRunFlag == 1)
				{
					// �ڴ����ѭ��ִ�д���
					while(IrComm1IsDataInRxBuf())
						{
							switch(IrComm1GetRxBufDat())
							 	{
									case AC_C_LIFTUP_CMD:
										g_ucAC_LiftUpCnt++;
									break;
									case AC_AC_LEAVE_START_CMD  :
										g_ucAC_LeaveStartCnt++;
									break;
									default:
									break;

								}
						}

					
					/*******************ACͨ�ųɹ��Լ��������ݽǶ�*********************************/
					if((g_ucAC_LeaveStartCnt>g_ucMC_CntTime)&&(g_ACFlag2==0))//�˴��ٴξ����������꣬��Ϊ��ͨ��ͨ�ž������ĽǶȿ��ܲ�׼ȷ,����ֻ����һ��
						{
							g_fpModifyAngle = g_fpGyroAngle; 
							stRobot.stPot.ssPosQ = 0;
							g_ucAC_LeaveStartCnt=0;
							g_ACFlag2=1;//AC��ʼͨ���Ƿ�ɹ�������
						}
					
					if((g_ucAC_LiftUpCnt>g_ucMC_CntTime)||JS_L1(JOYSTICK_RESERVED))
						{
							
			/*************��C����A����̧��ʱ������XY���꣬ʹ��C���ڵ��ϵĻ��о���Ҳ����������֮��******************/
							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;

							stRobot.stPot.ssPosX = 0;
							stRobot.stPot.ssPosY = 0;
						/***********************************/	

							//g_fpModifyAngle = g_fpGyroAngle; //������ǰ����������ʱ��A����Σ��ʽ��Ƕ���ǰ��C����A������ȥ�����
							
							 //stRobot.stPot.ssPosQ = 0;
							// �ڴ��ж��Ƿ���ת
							g_ACFlag2=1;//ACͨ���Ƿ�ɹ�������
							g_ucJumpRunFlag = 1;
							g_ucAllContinueFlag =1;
						}

					
							
					/******C���Զ��ǵ����򣬲�ͨ��ͨ��***********************/

					if(((GetCurTime()-g_uiTaskTime1)>3000000)&&(g_ACFlag2==0)&&(g_AC_NoCom_On_Off==1))//��ʱ4S��������ͨ��ͨ�žͿ����ϵ��ĳ���
						{
							if(IS_SWITCH_ON(FRONT_LEG_GOL2_SWITCH))
								{
									if(g_ACFlag3==0)
										{
											g_uiTaskTime2=GetCurTime();
											g_ACFlag3=1;
										}
									if((GetCurTime()-g_uiTaskTime2)>1000000)
										{
											stDoubleOMNI.fpLastPosX=0;
											stDoubleOMNI.fpLastPosY=0;

											stRobot.stPot.ssPosX = 0;
											stRobot.stPot.ssPosY = 0;
											// �ڴ��ж��Ƿ���ת
											g_ACFlag3=1;
											g_ucJumpRunFlag = 1;
											g_ucAllContinueFlag =1;
										}
								}
						}
					else
						{
							g_ACFlag3=0;
						}
					/***********************************************************************/
					
					if(g_ACFlag2==1)
						{
							/*****************ACͨ���Ƿ�ɹ���־��**************************/

							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							SendByteByUART1(AC_AC_COMSUCCESS_CMD);
							
							/*************************************************************************/
						}
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
						g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//������ת�Ĳ���
						stRobot.emRobotTask = TASK_START_2;
					}
				}
				
				break;
			case TASK_START_2://��A����,��Ҫ�ǵ������,��ʱ���������Ҫ��L2�ĸ߶ȣ���S2����֮��ĸ߶�
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
					stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1A_UP_C*M1COFF_POT);//AC��Ҫ�ϵ�֮ǰC�������ĸ߶�
					stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

					stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
					stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
					SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,10*M2COFF_POT);

					stAction_M3.fpVeltDes=0;
					stAction_M3.emState = MOTOR_VELT_CLOSE_LOOP_CTRL;

					OPEN_VALVE(FRONT_LEG_VALVE);
					CLOSE_VALVE(MID_CATCH_VALVE);
					CLOSE_VALVE(MID_LIFTUP_VALVE);
					CLOSE_VALVE(TOP_CATCH_VALVE);
					CLOSE_VALVE(ROT_VALVE);
					//g_uiTaskTime1=GetCurTime();
					/***********AC��Ҫ�ϵ�ʱ�����̿���************************/
					stRobot.emNavState =  NAV_NULL;
					stRobot.emBaseState=BASE_BREAK;
					while(IrComm1IsDataInRxBuf())
						{
							IrComm1GetRxBufDat();
						}
				}

				if(g_ucLoopRunFlag == 1)
				{
					// �ڴ����ѭ��ִ�д���

					//if((GetCurTime()-g_uiTaskTime1)>8000000)	
							while(IrComm1IsDataInRxBuf())
								{
									switch(IrComm1GetRxBufDat())
									{
										case AC_C_L2_CMD:
											g_ucAC_L2Cnt++;
										break;
										default:
										break;
									}

								}

									// �ڴ��ж��Ƿ���ת
									if(g_ucAC_L2Cnt>g_ucMC_CntTime)
									{			
										g_ucAC_L2Cnt=0;
										
										g_ucJumpRunFlag = 1;

										g_ucAllContinueFlag = 1;//�Զ�
									}
							/**********************��ͨ��ͨ�ŵ�����£�C���Զ��ǵ�*******************************/

								if((g_ACFlag3==1)&&(g_AC_NoCom_On_Off==1))
										{
										if(stAction_M1.emPathState==PATH_END)
											{
											g_ucJumpRunFlag = 1;

											g_ucAllContinueFlag = 1;//�Զ�
											}
										}

								/**********************************************************************/
								if(JS_R1(JOYSTICK_RESERVED) )
									{
										g_ucJumpRunFlag = 1;

										g_ucAllContinueFlag = 1;//�Զ�
									}
								/************************************************************************/
					
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
						g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//������ת�Ĳ���
						stRobot.emRobotTask = TASK2;
					}
				}
				break;
/*****************************����ʱ�����·��************************************************/		
				
			case TASK1://L2������ʱ�����·��
				
				//��΢�������������ǰ��
				 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==BLUE_FIELD)
							||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART)
							||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART))
							{
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
			 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1A_UP_C*M1COFF_POT);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						else
							{
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
			 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathLow,M1_L2_UP_C*M1COFF_POT);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}

						stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
						SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,10*M2COFF_POT);

						stAction_M3.fpVeltDes=0;
						stAction_M3.emState = MOTOR_VELT_CLOSE_LOOP_CTRL;

						OPEN_VALVE(FRONT_LEG_VALVE);
						CLOSE_VALVE(MID_CATCH_VALVE);
						CLOSE_VALVE(MID_LIFTUP_VALVE);
						CLOSE_VALVE(TOP_CATCH_VALVE);
						CLOSE_VALVE(ROT_VALVE);

					}

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���
	                                  
						if(stAction_M1.emPathState==PATH_END)
						{
	                                          OPEN_VALVE(FRONT_LEG_VALVE);
							// �ڴ��ж��Ƿ���ת
							g_ucJumpRunFlag = 1;
						
						}
						
					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
							g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//������ת�Ĳ���
							stRobot.emRobotTask = TASK2;
						}
					}
					
				break;
			case TASK2://L2�ϵ���ض�����·��
				//�������ݽǶȺ���һ��·��
				if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						 g_ACFlag4=0 ;
						USE_BLACK_FIELD_CFG(LINE_CHAN_0);
						//g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������

					#if 0//��C����A����̧��ʱ������XY���꣬ʹ��C���ڵ��ϵĻ��о���Ҳ����������֮�ڣ��ʴ˶δ������
						stDoubleOMNI.fpLastPosX=0;
						stDoubleOMNI.fpLastPosY=0;

					     /* g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������
						stRobot.stPot.ssPosQ = 0;�������Ƕ���ǰ��C����A����������ʱ��*/
						stRobot.stPot.ssPosX = 0;
						stRobot.stPot.ssPosY = 0;
					#endif
						
						stRobot.pstPrePath=NULL;
						stRobot.pstCurPath = g_pstL2Path;
						stRobot.pstCurPath->ssStartX=0;
						stRobot.pstCurPath->ssStartY=stRobot.stPot.ssPosY;

						stRobot.pstCurPath->ssStartQ=stRobot.stPot.ssPosQ;
						stRobot.pstCurPath->ssEndQ=stRobot.stPot.ssPosQ;
						if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==BLUE_FIELD)
							||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART)
							||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART))
							{
								stRobot.pstCurPath->usStartV=(ReadRobotFpVy(&stRobot)+150);//��ʱδд��������Y���ٶ�
							}

						stRobot.emPathState = PATH_RUN;
						stRobot.emNavState =  NAV_LINE;
						
						
					}

				if(g_ucLoopRunFlag == 1)
				{
						if(((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==BLUE_FIELD)
							||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART)
							||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART))&&
							(stRobot.stPot.ssPosY>500)&&(g_ACFlag4==0))
						{
							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
		 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_L2_UP_C*M1COFF_POT);//��A�����½��ĸ߶�
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							g_ACFlag4=1;	
						}
					if((stRobot.emRobotTactic==RED_FIELD)
						||(stRobot.emRobotTactic==BLUE_FIELD)
						||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
						||(stRobot.emRobotTactic==RED_FIELD_A_RESTART)
						||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
						||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART))
						{
							if(stRobot.stPot.ssPosY>500)
								{
									//stRobot.pstCurPath->ssStartQ=0;
									
									SendByteByUART1(AC_A_BASKET_CMD);
									SendByteByUART1(AC_A_BASKET_CMD);
									SendByteByUART1(AC_A_BASKET_CMD);
									SendByteByUART1(AC_A_BASKET_CMD);
									SendByteByUART1(AC_A_BASKET_CMD);
									SendByteByUART1(AC_A_BASKET_CMD);
									SendByteByUART1(AC_A_BASKET_CMD);
									SendByteByUART1(AC_A_BASKET_CMD);
									SendByteByUART1(AC_A_BASKET_CMD);
									SendByteByUART1(AC_A_BASKET_CMD);
									SendByteByUART1(AC_A_BASKET_CMD);
									SendByteByUART1(AC_A_BASKET_CMD);
									// �ڴ����ѭ��ִ�д���
								}

							if(stRobot.stPot.ssPosY>550)
								{
									CopyIntSlowlyEx(&(stRobot.pstCurPath->ssEndQ),0,1);
									//����x����Ѳ��
									g_fpPostionXTemp=-g_ucLineValue + g_ucLineMidValue;
				                                   
									CopyFloatSlowly(&stDoubleOMNI.fpLastPosX,&g_fpPostionXTemp, 1);
	
									//CopyFloatSlowly(&g_fpModifyAngle,&g_fpAngleTemp, 1);
								}
							else//�ھ���С��400ʱ��������x
								{
									stDoubleOMNI.fpLastPosX=0;
									stRobot.stPot.ssPosX = 0;
									//stRobot.pstCurPath->ssStartQ=stRobot.stPot.ssPosQ;
									//stRobot.pstCurPath->ssEndQ=stRobot.stPot.ssPosQ;
									

									//g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������
									
								}
						}

					else
						{
						   /*********����ʱ���ٷ�������ֹA���Ҷ�
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);*/
							// �ڴ����ѭ��ִ�д���
						
							//stRobot.pstCurPath->ssStartQ=stRobot.stPot.ssPosQ;
							//stRobot.pstCurPath->ssEndQ=stRobot.stPot.ssPosQ;//����ʱ�������Ƕȣ���ֹ�����˻ζ�
							
							
				                      //����x����Ѳ��
				                      g_fpPostionXTemp=g_ucLineValue - g_ucLineMidValue;
							//CopyFloatSlowly(&stDoubleOMNI.fpLastPosX,&g_fpPostionXTemp, 1);
						}

						// �ڴ��ж��Ƿ���ת
						if(stRobot.emPathState == PATH_END || IS_SWITCH_ON(FRONT_LEG_SWITCH))
						{
							g_ucJumpRunFlag = 1;
						}
					
				}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{
							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
							g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//������ת�Ĳ���
							stRobot.emRobotTask = TASK_INSERT_4;
						}
					}
				break;
		       case TASK_INSERT_4://׼�����ϵ�֮ǰ����X
			   	 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
					g_ucTaskCnt = 0;
					
					g_ucRobotXEnable = 1;
					g_ucRobotYEnable = 2;
					g_ucRobotQEnable = 1;

					g_siRobotXDesShadow = 0;
					g_siRobotQDesShadow = 0;
					/***************************���²���ֵ*******************************************************/
					g_siRobotXDes =  -g_ucLineValue + g_ucLineMidValue;;
					g_siRobotQDes = stRobot.stPot.ssPosQ;

					stRobot.stVeltDes.fpVy = 249;
					stRobot.emNavState = NAV_XYQ;
					
				}

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���

						if((GET_LINE_VALUE(LINE_CHAN_0)>50)&&(GET_LINE_VALUE(LINE_CHAN_0)<110))
							{
								stDoubleOMNI.fpLastPosX = -g_ucLineValue + g_ucLineMidValue;
								stRobot.stPot.ssPosX = -g_ucLineValue + g_ucLineMidValue;
								if(abs(stRobot.stPot.ssPosX) < 20)
								{
								      g_ucTaskCnt++;
								}
							}
						else
							{
								stRobot.stVeltDes.fpVy = 4;
								stDoubleOMNI.fpLastPosX = -g_ucLineValue+ g_ucLineMidValue;
								stRobot.stPot.ssPosX = -g_ucLineValue+ g_ucLineMidValue;
								 g_ucTaskCnt = 0;
							}

	                                   // �ڴ��ж��Ƿ���ת
						
						if(g_ucTaskCnt>=3)
							{
							     g_ucJumpRunFlag = 1;
							}
						
					}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//������ת�Ĳ���
							stRobot.emRobotTask = TASK3;
						}
					}
			   	break;
			case TASK3:
				//�����г̿��ش��������棬���������ݽǶ�

				if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						g_ucRobotXEnable = 2;
						g_ucRobotYEnable = 2;
						g_ucRobotQEnable = 2;
						g_ACFlag5=0;
						
						stRobot.emNavState = NAV_XYQ;
						
					}

				if(g_ucLoopRunFlag == 1)
				{
					// �ڴ����ѭ��ִ�д���
					if(IS_SWITCH_OFF(LEFT_POS_SWITCH) && IS_SWITCH_OFF(RIGHT_POS_SWITCH))
					{
						stRobot.stVeltDes.fpVy = 249;
						stRobot.stVeltDes.fpVx = 0;
						stRobot.stVeltDes.fpW = 0;
						g_ucTaskCnt = 0;
					}
					else if(IS_SWITCH_ON(LEFT_POS_SWITCH) && IS_SWITCH_OFF(RIGHT_POS_SWITCH))
					{
						stRobot.stVeltDes.fpVy = 45.6;
						stRobot.stVeltDes.fpVx = 0;
						stRobot.stVeltDes.fpW = 120;
						g_ucTaskCnt = 0;
					}
					else if(IS_SWITCH_OFF(LEFT_POS_SWITCH) && IS_SWITCH_ON(RIGHT_POS_SWITCH))
					{
						stRobot.stVeltDes.fpVy = 45.6;
						stRobot.stVeltDes.fpVx = 0;
						stRobot.stVeltDes.fpW = -120;
						g_ucTaskCnt = 0;
					}
					else
					{
						stRobot.stVeltDes.fpVy = 10;
						stRobot.stVeltDes.fpVx = 0;
						stRobot.stVeltDes.fpW =  0;
						g_fpModifyAngle = g_fpGyroAngle;
						g_ucTaskCnt++;
					}
          
					if(g_ucTaskCnt > 30 && IS_SWITCH_ON(RIGHT_POS_SWITCH)&&IS_SWITCH_ON(LEFT_POS_SWITCH) )//ԭ����10
					{
						g_fpModifyAngle = g_fpGyroAngle;
						
					// �ڴ��ж��Ƿ���ת
						g_ucJumpRunFlag = 1;
						
					}

					if((IS_SWITCH_ON(RIGHT_POS_SWITCH)||IS_SWITCH_ON(LEFT_POS_SWITCH))&&(g_ACFlag5==0))
						{
							g_uiTaskTime1 =GetCurTime();
							g_ACFlag5=1;
						}
					else if(IS_SWITCH_OFF(RIGHT_POS_SWITCH)&&IS_SWITCH_OFF(LEFT_POS_SWITCH))
						{
							g_uiTaskTime1 =GetCurTime();
						}

					if((GetCurTime()-g_uiTaskTime1)>2500000)
						{
							g_fpModifyAngle = g_fpGyroAngle;
						
							// �ڴ��ж��Ƿ���ת
							g_ucJumpRunFlag = 1;
						}
					
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
						g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//������ת�Ĳ���
						stRobot.emRobotTask = TASK4;
					}
					
				}
				break;
			case TASK4://�ǵ��ĵ������
				//�������
				 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						stAction_M3.fpVeltDes=-50;
						stAction_M3.emState = MOTOR_VELT_CLOSE_LOOP_CTRL;//����õ��ת����ֹ������
						stAction_M3.emState = MOTOR_OPEN_LOOP_CTRL;
						stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
						if((g_ucCurModeSel==C_MODE_TOP_SPEED_8)||(g_ucCurModeSel==C_MODE_MID_SPEED_7))
							{
								SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid,M1_CLIMBSTEP_C*M1COFF_POT);//�ȵ�ʱ��������ĸ߶�ԭ���Ƕ�ֵΪ-780,-230
							}
						else
							{
								SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathLow,M1_CLIMBSTEP_C*M1COFF_POT);//�ȵ�ʱ��������ĸ߶�ԭ���Ƕ�ֵΪ-780,-230
							}
						stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						g_uiTaskTime1 = GetCurTime();
					}

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���
				
						if((stAction_M1.emPathState == PATH_END)||(GetCurTime()-g_uiTaskTime1>1500000))
						{
						// �ڴ��ж��Ƿ���ת
							g_ucJumpRunFlag = 1;
						}

					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                          g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//������ת�Ĳ���
							stRobot.emRobotTask = TASK5;
						}
					}
			       break;
		       case TASK5://���������������ǰ����С�����ʼ����
			   	 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						g_M3TempCoder=0;
						g_M3TempCoder=READ_CODER(stAction_M3.ucCoderChan);
						stAction_M3.fpVeltDes=-250;
						stAction_M3.emState = MOTOR_VELT_CLOSE_LOOP_CTRL;
						g_uiTaskTime1 = GetCurTime();

						if((stRobot.emRobotTactic==RED_FIELD)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
						||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
						||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
							{
								USE_RED_FIELD_CFG(LINE_CHAN_0);
							}
						else
							{
								USE_BLUE_FIELD_CFG(LINE_CHAN_0);
							}

						g_ucRobotXEnable = 2;
						g_ucRobotYEnable = 2;
						g_ucRobotQEnable = 2;
						stRobot.emNavState = NAV_XYQ;
						stRobot.stVeltDes.fpVy = 0;
						stRobot.stVeltDes.fpVx = 0;
						stRobot.stVeltDes.fpW =  0;
					}

				if(g_ucLoopRunFlag == 1)
					{
					           if(abs( READ_CODER(stAction_M3.ucCoderChan)-g_M3TempCoder)>15000  && abs( READ_CODER(stAction_M3.ucCoderChan)-g_M3TempCoder)<19000)
					           	{
					           	       stAction_M3.fpVeltDes= -250 + (250 - 80)*((abs(READ_CODER(stAction_M3.ucCoderChan)-g_M3TempCoder) - 15000)/5000.0);
								stAction_M3.emState = MOTOR_VELT_CLOSE_LOOP_CTRL;
					           	}

							 if( abs( READ_CODER(stAction_M3.ucCoderChan)-g_M3TempCoder)>19000)
					           	{
					           	       stAction_M3.fpVeltDes= -80;
								stAction_M3.emState = MOTOR_VELT_CLOSE_LOOP_CTRL;
					           	}
						// �ڴ����ѭ��ִ�д���
	                                   if(IS_SWITCH_ON(BACK_LEG_SWITCH_FRONT))
	                                   {
							//OPEN_VALVE(BACK_LEG_VALVE);//ԭ����װ�󲿹������׵����
							
						/*******************����˫�ջ�������������ʱ��*********************************/
							stAction_M1.emState=MOTOR_POS_CLOSE_LOOP_CTRL;
							stAction_M1.fpPotDes=M1_DOUBLEPOTLOOP_C*M1COFF_POT;//������ȫ�Ӵ�����
						/********************************************************/
															
							 stDoubleOMNI.fpLastPosY = 0;//ԭ��-10
							 
							stAction_M3.fpVeltDes=0;
							stAction_M3.emState = MOTOR_VELT_CLOSE_LOOP_CTRL;
							 
							 // �ڴ��ж��Ƿ���ת
							 
							 g_ucJumpRunFlag = 1;
	                                   }

						
					}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                      			g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//������ת�Ĳ���


/********************************���Ĺ�״̬*****************************************************/
						
						//stRobot.emRobotTask = TASK14;
						stRobot.emRobotTask = TASK_INSERT_1;

/******************************************************************************************************************/
						
					}
				}
			   	break;
			case TASK_INSERT_1://�����λ�ñջ��Ƿ�ﵽλ��
				//�������
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
					
				}

				if(g_ucLoopRunFlag == 1)
				{
					// �ڴ����ѭ��ִ�д���
					
					if(fabs(stAction_M1.fpPotFB-M1_DOUBLEPOTLOOP_C*M1COFF_POT)<5)
						{
							// �ڴ��ж��Ƿ���ת
				
							g_ucJumpRunFlag = 1;
						}
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                                          g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//������ת�Ĳ���
						stRobot.emRobotTask = TASK6;
					}
				}
				break;
			case TASK6://���彫Ҫ�Ӵ������һЩ�����������Ӿ���λ
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					
					g_ucTaskCnt=0;
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
					g_uiTaskTime1 = GetCurTime();
					if((stRobot.emRobotTactic==RED_FIELD)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
						||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
						||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
						{
							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
							SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_PICKUPBAN_TEMP_C*M1COFF_POT);//�˸߶���C��ȡ��ͷʱ��ʱ�����ĸ߶ȣ���ֹ�в���棬�������ᵽȡ��ͷ�ĸ߶�
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						}
					else 
						{
							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
							SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_PICKUPBAN_TEMP_C*M1COFF_POT);//�˸߶���C��ȡ��ͷʱ��ʱ�����ĸ߶ȣ���ֹ�в���棬�������ᵽȡ��ͷ�ĸ߶�
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;	
						}
					
				}

				 #if 1
				if(g_ucLoopRunFlag == 1)
				{
					// �ڴ����ѭ��ִ�д���

					stDoubleOMNI.fpLastPosX=-g_ucLineValue + g_ucLineMidValue;	
					stRobot.stPot.ssPosX=-g_ucLineValue + g_ucLineMidValue;
					#if 1//�Ӿ���λ����
                                    g_fpVisionValue =(FP32) (RetVisionLineValue(&stVisionDetecLine));
					if(g_fpVisionValue > 0)
					{
					     g_fpVisionValue = g_fpVisionValue / 10.0;
					     g_fpVisionPotY = g_fpVisionValue  +60* sin(stRobot.stPot.ssPosQ * RADIAN_10)- 36;
					     g_fpPotTemp = stRobot.stPot.ssPosY;
					}
					
					#endif
					  if((stAction_M1.emPathState==PATH_END)||((GetCurTime()-g_uiTaskTime1) >2000000))//��ֹ��ֱ����������⣬���������ƶ����ʴ˴�������ʱ
		                        {

								stRobot.emNavState=NAV_XYQ;
								g_ucRobotQEnable=0;
								g_ucRobotXEnable=0;
								g_ucRobotYEnable=2;

								stRobot.stVeltDes.fpVy=100;
								stRobot.stVeltDes.fpVx=0;
								stRobot.stVeltDes.fpW=0;

								
								stAction_M3.ssPwmDuty = 0;
								stAction_M3.emState = MOTOR_OPEN_LOOP_CTRL;
								 
								stDoubleOMNI.fpLastPosX=-g_ucLineValue + g_ucLineMidValue;
								stRobot.stPot.ssPosX=-g_ucLineValue + g_ucLineMidValue;

								// �ڴ��ж��Ƿ���ת
								g_ucJumpRunFlag =1;	
								
		                           }
					
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;

					
					// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
					if(JS_R1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                        			g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//������ת�Ĳ���
						stRobot.emRobotTask = TASK7;
					}
				}
				#endif
				break;
				
			case TASK7://�ǵ���ͨ������Ŀ��ض�λC��Y����룬///////�˴�Ҳ��������ʽ4��ֱ��ȥMȡC����״̬�ж�
				if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					g_ucTaskCnt=0;
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
					
				}

				if(g_ucLoopRunFlag == 1)
				{
					// �ڴ����ѭ��ִ�д���
					if(IS_SWITCH_ON(BACK_LEG_SWITCH_BACK)&&(g_ucTaskCnt==0))
						{
							stDoubleOMNI.fpLastPosY=40;//ԭ����65
							stRobot.stPot.ssPosY=40;
							stRobot.stPot.ssPosX=-g_ucLineValue + g_ucLineMidValue;
							g_ucTaskCnt=1; 
						}
					if((stRobot.stPot.ssPosY>=150 )||(g_ucTaskCnt==1))
						{
							g_ucRobotQEnable=0;
							g_ucRobotXEnable=0;
							g_ucRobotYEnable=0;
							stRobot.stVeltDes.fpVy=0;
							stRobot.stVeltDes.fpVx=0;
							stRobot.stVeltDes.fpW=0;
							// �ڴ��ж��Ƿ���ת
		                          		g_ucJumpRunFlag = 1;   
						}
                                  

				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                                      //g_ucTaskCnt��׼��ֵΪ0����Ϊ������õ�
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//������ת�Ĳ���


/***************************�ڴ˴������˻���������4������ж�************************************************************************/
						if((stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART4))
							{
								stRobot.emRobotTask = TASK_INSERT_6;	
							}
						else
							{
								stRobot.emRobotTask = TASK8;
							}
					}
				}
				break;
				
			case TASK8:	//�˴�Ϊȥ��ͷ��·��			
 #if 1
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬

					/*************ϣ������ȡ��ͷʱ���������Ľϵͣ��ʽ��˶δ������***********************************
					
					*****************************************************/

					stDoubleOMNI.fpLastPosX=-g_ucLineValue + g_ucLineMidValue;
					stRobot.stPot.ssPosX=-g_ucLineValue + g_ucLineMidValue;
					g_pstCatchBanPath->ssStartX = stRobot.stPot.ssPosX;
					g_pstCatchBanPath->ssStartY = stRobot.stPot.ssPosY;
					g_pstCatchBanPath->ssStartQ = stRobot.stPot.ssPosQ;
					stRobot.pstPrePath=NULL;
					stRobot.pstCurPath = g_pstCatchBanPath;
					stRobot.emPathState = PATH_RUN;
					stRobot.emNavState =  NAV_LINE;


					if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)
						{
							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
							SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_PICKUPBAN_LAST_C*M1COFF_POT);//20,�˸߶�Ҳ��ȡ��ͷ�ĸ߶�
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
									
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_PICKUPBANRES3_TEMP_C*M2COFF_POT);//����M2������һ�ξ��룬�Է�ֹ�겻��
						}
					else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
						{
							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
							SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_PICKUPBAN_LAST_C*M1COFF_POT);//20,�˸߶�Ҳ��ȡ��ͷ�ĸ߶�
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_PICKUPBANRES3_TEMP_C*M2COFF_POT);//����M2������һ�ξ��룬�Է�ֹ�겻��
						}
					else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
						{	
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_PICKUPBANRES2_LAST_C*M2COFF_POT);//����M2������һ�ξ��룬�Է�ֹ�겻��
						}
					else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
						{	
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_PICKUPBANRES2_LAST_C*M2COFF_POT);//����M2������һ�ξ��룬�Է�ֹ�겻��
						}


					else if((stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART)
						||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
						||(stRobot.emRobotTactic==BLUE_FIELD))
						{
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_PICKUPBANRES1_LAST_C*M2COFF_POT);//�˳���Ϊ����������������ʽ2����
						}
					else if((stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==RED_FIELD_A_RESTART)
						||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
						||(stRobot.emRobotTactic==RED_FIELD))
						{
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_PICKUPBANRES1_LAST_C*M2COFF_POT);//�˳���Ϊ����������������ʽ2����
						}
					
				}

				if(g_ucLoopRunFlag == 1)
				{
					// �ڴ����ѭ��ִ�д���

					if((stRobot.emRobotTactic==BLUE_FIELD)
						||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)&&(g_ucTaskCnt==1))

						{
							

							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
							SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_PICKUPBAN_LAST_C*M1COFF_POT);//20,�˸߶�Ҳ��ȡ��ͷ�ĸ߶�
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

							g_ucJumpRunFlag = 1;
							
						}
					else if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)&&(g_ucTaskCnt==1))
						{
							//if(stRobot.stPot.ssPosY>500)//�˴�Ҫ����ʵ��λ�����Ӵ˴�������ֵ
								{

									stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
									SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_PICKUPBAN_LAST_C*M1COFF_POT);//20,�˸߶�Ҳ��ȡ��ͷ�ĸ߶�
									stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

									g_ucJumpRunFlag = 1;
								}
						}
					else if((stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)&&(g_ucTaskCnt==1))//����ʱ·����һ����������ĸ߶�Ҳ������ͬ
						{
							//if(stRobot.stPot.ssPosY>500)//if(stRobot.stPot.ssPosY>500)//�˴�Ҫ����ʵ��λ�����Ӵ˴�������ֵ
								{
									//���ڴ˴��³��Ļ����䶯���ʴ˴��ĽǶ�
									//g_pstCatchBanPath->ssEndQ=RED_L2_RESTART2_CATCHBAN_Q;//��ȷ��C������Ķ�λ���ش����󣬲ſ�ʼת��
									//g_pstReleaseBanPath->ssStartQ=RED_L2_RESTART2_CATCHBAN_Q;

									stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
									SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_PICKUPBAN_LAST_C*M1COFF_POT);//20,�˸߶�Ҳ��ȡ��ͷ�ĸ߶�
									stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
									g_ucJumpRunFlag = 1;
								}
						}
					else if((stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)&&(g_ucTaskCnt==1))
						{
							//g_pstCatchBanPath->ssEndQ=BLUE_L2_RESTART2_CATCHBAN_Q;//��ȷ��C������Ķ�λ���ش����󣬲ſ�ʼת��
							//g_pstReleaseBanPath->ssStartQ=BLUE_L2_RESTART2_CATCHBAN_Q;

							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
							SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_PICKUPBAN_LAST_C*M1COFF_POT);//20,�˸߶�Ҳ��ȡ��ͷ�ĸ߶�
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							g_ucJumpRunFlag = 1;
						}
					else if((stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)&&(g_ucTaskCnt==1))
						{
							if((g_pstCatchBanPath->ssEndY-stRobot.stPot.ssPosY)<70)//�˴�Ҫ����ʵ��λ�����Ӵ˴�������ֵ
								{

									stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
									stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
									SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_PICKUPBANRES3_LAST_C*M1COFF_POT);//����·��3��M2�������������ľ���

									g_ucJumpRunFlag = 1;
								}
						}

					else if((stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)&&(g_ucTaskCnt==1))
						{
							if((g_pstCatchBanPath->ssEndY-stRobot.stPot.ssPosY)<70)
								{

									

									stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
									stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
									SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_PICKUPBANRES3_LAST_C*M1COFF_POT);//����·��3��M2�������������ľ���

									g_ucJumpRunFlag = 1;
								}
						}
					
					if(IS_SWITCH_ON(BACK_LEG_SWITCH_BACK)&&(g_ucTaskCnt==0))
						{
							//stDoubleOMNI.fpLastPosY=75;//ԭ����װ�������׵����
							stDoubleOMNI.fpLastPosY=40;
							stRobot.stPot.ssPosY=40;
							g_ucTaskCnt=1; 
							
						}
					
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                        			g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//������ת�Ĳ���
						stRobot.emRobotTask = TASK9;
					}
				}
				
#endif
				break;

			case TASK9://�ж�·���˴��Ƿ��������׼����ȡ��ͷ
				 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
		
					}

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���

						if(abs(stRobot.stPot.ssPosX -g_pstCatchBanPath->ssEndX)<3 && abs(stRobot.stPot.ssPosY -g_pstCatchBanPath->ssEndY) <4 && abs(stRobot.stPot.ssPosQ-g_pstCatchBanPath->ssEndQ)<4 && (stAction_M2.emPathState == PATH_END))
						{
							g_ucTaskCnt++;
						}

							
						// �ڴ��ж��Ƿ���ת
						if(g_ucTaskCnt>=3)
						{
							g_ucJumpRunFlag = 1;
						}
						
					}

				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					/******************************����ˮƽ���ǰ��ָ��***************************************/
						 if(KEY_F5(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
						        
						       	stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
			 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid, g_fpActionM1PotFB + g_fpM1DeltaPos2);
							}	

						if(KEY_F4(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
							
								stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathMid, g_fpActionM1PotFB - g_fpM1DeltaPos2);
							 }
					
				/********************************������ֱ�������ָ��*******************************************************/
						if(KEY_F1(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB + g_fpM2DeltaPos2);
									
							}

						if(KEY_F8(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB - g_fpM2DeltaPos2);
								
							}

					// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                        			g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//������ת�Ĳ���
						stRobot.emRobotTask = TASK10;
					}
				}
				
				break;
			case TASK10://��ȡ�в���ͷ����ز���

#if 1
			     if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						if((stRobot.emRobotTactic==RED_FIELD)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
						||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
						||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
							{
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
								SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_CATCHMIDDLEBAN_C*M1COFF_POT);//20,�˸߶�Ҳ��ȡ��ͷ�ĸ߶�
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						else
							{
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
								SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_CATCHMIDDLEBAN_C*M1COFF_POT);//20,�˸߶�Ҳ��ȡ��ͷ�ĸ߶�
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						
						OPEN_VALVE(TOP_CATCH_VALVE);
						OPEN_VALVE(MID_CATCH_VALVE);
						g_uiTaskTime1 = GetCurTime();
						
					}

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���

						// �ڴ��ж��Ƿ���ת
						if((GetCurTime() - g_uiTaskTime1) >= 300000)
						{
							g_ucJumpRunFlag = 1;
						}
		
					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						/******************************����ˮƽ���ǰ��ָ��***************************************/
						 if(KEY_F5(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
						        
						       	stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
			 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid, g_fpActionM1PotFB + g_fpM1DeltaPos2);
							}	

						if(KEY_F4(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
							
								stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathMid, g_fpActionM1PotFB - g_fpM1DeltaPos2);
							 }
					
				/********************************������ֱ�������ָ��*******************************************************/
						if(KEY_F1(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB + g_fpM2DeltaPos2);
									
							}

						if(KEY_F8(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB - g_fpM2DeltaPos2);
								
							}

						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//������ת�Ĳ���
							stRobot.emRobotTask = TASK11;
						}
					}
				
#endif

				break;
			case TASK11://��ȡ��ͷ֮��M1M2�����ʼ�˶�
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;

					#if 1
					/*******************������ͷ֮��M2�����һ�ξ���********************************************/
					if((stRobot.emRobotTactic==BLUE_FIELD)
						||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART)
						||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART))
						{
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,(M2_BLUE_PICKUPBANRES1_LAST_C-20)*M2COFF_POT);
						}
					
					else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
						{
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,(M2_BLUE_PICKUPBANRES2_LAST_C-20)*M2COFF_POT);
					
						}
					else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)
						{
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,(M2_BLUE_PICKUPBANRES3_LAST_C-20)*M2COFF_POT);
						}

					
					else if((stRobot.emRobotTactic==RED_FIELD)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==RED_FIELD_A_RESTART)
						||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART))
						{
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,(M2_RED_PICKUPBANRES1_LAST_C-20)*M2COFF_POT);
						}
					else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
						{
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,(M2_RED_PICKUPBANRES2_LAST_C-20)*M2COFF_POT);
						}
					else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
						{
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,(M2_RED_PICKUPBANRES3_LAST_C-20)*M2COFF_POT);	
						}
					#endif
					/*****************************************************************/
								   
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
					if((stRobot.emRobotTactic==RED_FIELD)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
						||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
						||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
						{
							
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
								 SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_LIFT_BAN*M1COFF_POT);//�˸߶��Ƿ���ͷ�ĸ߶�
							       stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								 //OPEN_VALVE(MID_LIFTUP_VALVE);//��Ϊ���׻Ῠס��ɽ�в㣬�ʽ��˴�����
						}
					else 
						{
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
								 SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_LIFT_BAN*M1COFF_POT);//�˸߶��Ƿ���ͷ�ĸ߶�
							       stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						}
					
				}

				if(g_ucLoopRunFlag == 1)
				{
					// �ڴ����ѭ��ִ�д���


					// �ڴ��ж��Ƿ���ת
					if(stAction_M1.fpPotFB>(115*M1COFF_POT))//ֻҪM1�������ͷ�������뿪��ɽ��C���Ϳ����ƶ�
						{
							g_ucJumpRunFlag = 1;
						}
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					/******************************����ˮƽ���ǰ��ָ��***************************************/
						 if(KEY_F5(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
						        
						       	stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
			 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid, g_fpActionM1PotFB + g_fpM1DeltaPos2);
							}	

						if(KEY_F4(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
							
								stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathMid, g_fpActionM1PotFB - g_fpM1DeltaPos2);
							 }
					
				/********************************������ֱ�������ָ��*******************************************************/
						if(KEY_F1(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB + g_fpM2DeltaPos2);
									
							}

						if(KEY_F8(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB - g_fpM2DeltaPos2);
								
							}

					// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                                                g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//������ת�Ĳ���
						stRobot.emRobotTask =  TASK_INSERT_5;
					}
				}
				
				break;
			case TASK_INSERT_5: //�����Ƿ���������·���жϣ���Ҫ��������ʽ2��ʱ��ʹ��
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
					if((stRobot.emRobotTactic==RED_FIELD)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
						||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
						||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
						{
						
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_DROPBAN_C*M2COFF_POT);	//������ͷ֮ǰ�ջ�M2���
						}
					else
						{
						
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_DROPBAN_C*M2COFF_POT);	//������ͷ֮ǰ�ջ�M2���
						}
							#if 0//
					 if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
						{	

						
							stRobot.pstCurPath =&stNavPath[16];
							
							stRobot.pstCurPath->ssStartX = stRobot.stPot.ssPosX;
							stRobot.pstCurPath->ssStartY = stRobot.stPot.ssPosY;
							stRobot.pstCurPath->ssStartQ = stRobot.stPot.ssPosQ;
							
							stRobot.pstCurPath->ssEndQ= stRobot.stPot.ssPosQ;
							
							stRobot.emPathState = PATH_RUN;
							stRobot.pstPrePath = NULL;
							stRobot.emNavState = NAV_LINE;
					
							
						}
					  if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
						{							
							
							;
						}
					  	#endif

					
				}

				if(g_ucLoopRunFlag == 1)
				{
					// �ڴ����ѭ��ִ�д���
					
					
							// �ڴ��ж��Ƿ���ת
							g_ucJumpRunFlag =1;
						
						
						
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                                                g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//������ת�Ĳ���
						stRobot.emRobotTask = TASK12;
					}
				}
				break;
			
			case TASK12://��Ϊ����·����һ������Ҫ��һЩ����

			#if 1
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
					g_CFlag6=0;
					g_pstReleaseBanPath->ssStartX = stRobot.stPot.ssPosX;
					g_pstReleaseBanPath->ssStartY = stRobot.stPot.ssPosY;
					g_pstReleaseBanPath->ssStartQ = stRobot.stPot.ssPosQ;
					stRobot.pstCurPath =g_pstReleaseBanPath;
					stRobot.emPathState = PATH_RUN;
					stRobot.pstPrePath = NULL;
					stRobot.emNavState = NAV_LINE;
				}

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���
					
						if((abs(stRobot.stPot.ssPosX)<150)&&(g_CFlag6==0))//���˴��Ĵ򿪷��Ĳ����ŵ�����ط���ʹ���в��ֲ��������в��ɽ
							{
								OPEN_VALVE(MID_LIFTUP_VALVE);
							if((stRobot.emRobotTactic==RED_FIELD)
								||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
								||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
								||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
								||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
								||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
								||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
								{
									
										stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
										 SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_WAIT_M_C*M1COFF_POT);//�˸߶��Ƿ���ͷ�ĸ߶�
									       stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
										 //OPEN_VALVE(MID_LIFTUP_VALVE);//��Ϊ���׻Ῠס��ɽ�в㣬�ʽ��˴�����
								}
							else 
								{
										stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
										 SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_WAIT_M_C*M1COFF_POT);//�˸߶��Ƿ���ͷ�ĸ߶�
									       stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								}
							g_CFlag6=1;
					
							}
						if(abs(stRobot.stPot.ssPosX -g_pstReleaseBanPath->ssEndX)<3 && abs(stRobot.stPot.ssPosY -g_pstReleaseBanPath->ssEndY) <4 && abs(stRobot.stPot.ssPosQ-g_pstReleaseBanPath->ssEndQ)<4)
							{
								g_ucTaskCnt++;
							}

						// �ڴ��ж��Ƿ���ת
						if(g_ucTaskCnt>=3)
							{
								g_ucJumpRunFlag = 1;
							}

					}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
							{

								g_ucOnceRunFlag = 0;
								g_ucLoopRunFlag = 1;
								g_ucJumpRunFlag = 0;
		                                                g_ucTaskCnt = 0;
								g_uiTaskTime1 = 0;
								g_uiTaskTime2 = 0;
								g_siTaskDeltaTime = 0;
								//������ת�Ĳ���
								stRobot.emRobotTask = TASK13;
							}
					}
				
				#endif
				break;
			case TASK13://�ȴ�M����������ͷ
				 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						g_uiTaskTime1=GetCurTime();
						g_siRobotQDes = stRobot.stPot.ssPosQ;
						g_siRobotXDes = stRobot.stPot.ssPosX;
						g_siRobotYDes = stRobot.stPot.ssPosY;					
					}

				if(g_ucLoopRunFlag == 1)
				{

					if(GetCurTime()-g_uiTaskTime1>200000)
						{
					// �ڴ����ѭ��ִ�д���

						
						

						if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))

							{
								if(IS_SWITCH_ON(RIGHT_LIGHT_SWITCH)||(JS_R1(JOYSTICK_RESERVED) ))
									{
										// �ڴ��ж��Ƿ���ת

										g_ucJumpRunFlag =1;

										
										/********************************�Ĺ�************************************************************/
										 g_ucAllContinueFlag = 1;
										/******************************************************************************************/
										 
									}
							}
						else 
							{
								if(IS_SWITCH_ON(LEFT_LIGHT_SWITCH)||(JS_R1(JOYSTICK_RESERVED) ))
									{
										// �ڴ��ж��Ƿ���ת

										g_ucJumpRunFlag =1;

										
										/********************************�Ĺ�************************************************************/
										 g_ucAllContinueFlag = 1;
										/******************************************************************************************/
										 
									}
							}
						}

				}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//������ת�Ĳ���
							stRobot.emRobotTask = TASK14;
							
						}
					}
				
				
				break;
			

	/*******************************my code**********************************************************/
			case TASK14://�ͷ���ͷ�Ĳ�����Ϊ����ĳ�ʼ��������׼��
				
               		if(g_ucOnceRunFlag == 0)
					{
					
	                                   g_ucOnceRunFlag = 1;
						
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬	
						g_uiTaskTime1=GetCurTime();
						CLOSE_VALVE(TOP_CATCH_VALVE);
						CLOSE_VALVE(MID_CATCH_VALVE);
						
						if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))

							{

								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
								SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_DROPBAN_C*M1COFF_POT);//������ͷ��ʱ�򽵵͸߶ȣ�ʹ��ͷ�����ܵ������µ��ٶ�
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						else
							{
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
								SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_DROPBAN_C*M1COFF_POT);//������ͷ��ʱ�򽵵͸߶ȣ�ʹ��ͷ�����ܵ������µ��ٶ�
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
														 
						g_ucRobotQEnable = 1;
						g_ucRobotXEnable = 1;
						g_ucRobotYEnable = 1;
										 
						g_siRobotQDesShadow= 0;
						g_siRobotXDesShadow= stRobot.stPot.ssPosX;
						g_siRobotYDesShadow= stRobot.stPot.ssPosY;
				 
						stRobot.emNavState = NAV_XYQ;
										 
					}
						

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���
						if(GetCurTime()-g_uiTaskTime1>300000)
							{
								// �ڴ��ж��Ƿ���ת		
								g_ucJumpRunFlag = 1;
							}
								
					}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							stRobot.emRobotTask=TASK16;
							//������ת�Ĳ���
						}
					}
				
				break;
		#if 0

			case TASK15:
				if(g_ucOnceRunFlag == 0)
					{
						stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
 						SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,110*3.4043f);
						stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						g_ucOnceRunFlag = 1;
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
					}

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���
						while(IrComm1IsDataInRxBuf())
							{
							     switch(IrComm1GetRxBufDat())
				   			 	 {
								 	case MC_INIT_CMD:
										g_ucMC_InitCnt++;
										break;
									
									default:
										break;
									
							    	 }
								 
							}

						// �ڴ��ж��Ƿ���ת

						if(g_ucMC_InitCnt>g_ucMC_CntTime)
							{
								g_ucJumpRunFlag = 1;
								g_ucAllContinueFlag =1;
							}
					/////////////�ֶ���ʼ��ʹ��//////////////////////////
						if(JS_L1(JOYSTICK_RESERVED))
						{

							g_ucJumpRunFlag = 1;
							g_ucAllContinueFlag =1;
							//������ת�Ĳ���
						}
						
					}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							stRobot.emRobotTask=TASK16;
							//������ת�Ĳ���
						}
					}
			
				break;
			#endif
			case TASK_INSERT_6://������ʽ4ֱ��ȥMȡC�������
				  if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						g_pstReleaseBanPath->ssStartX = stRobot.stPot.ssPosX;
						g_pstReleaseBanPath->ssStartY = stRobot.stPot.ssPosY;
						g_pstReleaseBanPath->ssStartQ = stRobot.stPot.ssPosQ;
						stRobot.pstCurPath =g_pstGoBasketPath;
						stRobot.emPathState = PATH_RUN;
						stRobot.pstPrePath = NULL;
						stRobot.emNavState = NAV_LINE;
						
					}

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���
						if(abs(stRobot.stPot.ssPosX -g_pstReleaseBanPath->ssEndX)<3 
							&& abs(stRobot.stPot.ssPosY -g_pstReleaseBanPath->ssEndY) <4 
							&& abs(stRobot.stPot.ssPosQ-g_pstReleaseBanPath->ssEndQ)<4)
							{
								g_ucTaskCnt++;
							}

						// �ڴ��ж��Ƿ���ת
						if(g_ucTaskCnt>=3)
							{
								g_ucJumpRunFlag = 1;
							}

						
					}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//������ת�Ĳ���
						}
					}
				
				break;
			case TASK16://��������ת�ֱ�,ˮƽ��ֱ�����ʼ�˶�
				if(g_ucOnceRunFlag == 0)
					{
									
						 g_ucOnceRunFlag = 1;
						//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						CLOSE_VALVE(MID_CATCH_VALVE);
						CLOSE_VALVE(TOP_CATCH_VALVE);
						OPEN_VALVE(ROT_VALVE);

						g_uiTaskTime1=GetCurTime();
						if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))

							{
						
						 		stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
				 				SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid,M1_RED_CATCHTOPBAN_C*M1COFF_POT);//��ֱ����쵽ȡ������ͷ�ĸ߶�
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
										 
								stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
				 				SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_PICKUPTOPBAN_C*M2COFF_POT);//ˮƽ����쵽ȡ������ͷ�ĳ���
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;	
							}
						else

							{
						
						 		stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
				 				SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid,M1_BLUE_CATCHTOPBAN_C*M1COFF_POT);//��ֱ����쵽ȡ������ͷ�ĸ߶�
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
										 
								stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
				 				SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_PICKUPTOPBAN_C*M2COFF_POT);//ˮƽ����쵽ȡ������ͷ�ĳ���
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;	
							}
					}
				

				if(g_ucLoopRunFlag == 1)
					{

						// �ڴ����ѭ��ִ�д���
							/*if((stAction_M2.emPathState==PATH_END)||(GetCurTime()-g_uiTaskTime1>4000000))//��ֹM2������ֿ������ʽ��˴�����ʱ
								{
									stRobot.emNavState=NAV_NULL;
									stRobot.emBaseState=BASE_STOP_QUICKLY;
									
									
									// �ڴ��ж��Ƿ���ת
									g_ucJumpRunFlag = 1;
								}*/
						if((IS_SWITCH_OFF(TOP_ROTAIRPOS_SWITCH)||(GetCurTime()-g_uiTaskTime1>1500000)))
							{
								// �ڴ��ж��Ƿ���ת
								g_ucJumpRunFlag = 1;
								CLOSE_VALVE(ROT_VALVE);
							}
						

					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//������ת�Ĳ���
							stRobot.emRobotTask=TASK_INSERT_9;
						
						}
					}
				
				break;
			case TASK_INSERT_9:
				if(g_ucOnceRunFlag == 0)
					{
						g_ucOnceRunFlag = 1;
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						g_uiTaskTime1=GetCurTime();
						
					}

				if(g_ucLoopRunFlag == 1)
					{

						// �ڴ����ѭ��ִ�д���
						if(GetCurTime()-g_uiTaskTime1>200000)
							{
								// �ڴ��ж��Ƿ���ת
								g_ucJumpRunFlag = 1;
								OPEN_VALVE(ROT_VALVE);
							}

					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//������ת�Ĳ���
							stRobot.emRobotTask=TASK_INSERT_8;
						
						}
					}
				break;
			case TASK_INSERT_8:
				if(g_ucOnceRunFlag == 0)
					{
						g_ucOnceRunFlag = 1;
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						g_uiTaskTime1=GetCurTime();
						while(IrComm1IsDataInRxBuf())
						{
							IrComm1GetRxBufDat();
						}
						
					}
				
				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���
						
					while(IrComm1IsDataInRxBuf())
						{
						     switch(IrComm1GetRxBufDat())
						     {
							 	case MC_UP_CMD:
									g_ucMC_M1UpCnt++;
									break;
								case MC_DOWN_CMD:
									g_ucMC_M1DownCnt++;
									break;
								case MC_OUT_CMD:
									g_ucMC_M2OutCnt++;
									break;
								case MC_IN_CMD:
									g_ucMC_M2InCnt++;
									break;
							       case MC_CATCH_CMD:
								   	g_ucMC_CatchCnt++;
								   	break;
								case MC_RELEASE_CMD:
									g_ucMC_ReleaseCnt++;
									break;
								case MC_ROTFIT_CMD:
									g_ucMC_RotFitCnt++;
									break;
								case MC_ROTOPST_CMD:
									g_ucMC_RotOpstCnt++;
									break;
								case MC_AUTO_CMD:
									g_ucMC_AutoCnt++;
									break;
								default:
									break;
						
				    			 }
						}

						if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))

							{
						
								if((abs(stAction_M2.fpPotFB-M2_RED_PICKUPTOPBAN_C*M2COFF_POT)<M2_RED_CATCHTOPBAN_COM_ON_C*M2COFF_POT)||(GetCurTime()-g_uiTaskTime1>4000000))//��ֹM2������ֿ������ʽ��˴�����ʱ
										{
											stRobot.emNavState=NAV_NULL;
											stRobot.emBaseState=BASE_STOP_QUICKLY;
											
											
											// �ڴ��ж��Ƿ���ת
											g_ucJumpRunFlag = 1;
										}
							}
						else
							{
						
								if((abs(stAction_M2.fpPotFB-M2_BLUE_PICKUPTOPBAN_C*M2COFF_POT)<M2_BLUE_CATCHTOPBAN_COM_ON_C*M2COFF_POT)||(GetCurTime()-g_uiTaskTime1>4000000))//��ֹM2������ֿ������ʽ��˴�����ʱ
										{
											stRobot.emNavState=NAV_NULL;
											stRobot.emBaseState=BASE_STOP_QUICKLY;
											
											
											// �ڴ��ж��Ƿ���ת
											g_ucJumpRunFlag = 1;
										}
							}
											
					}


					if(g_ucJumpRunFlag == 1)
						{
		                                    g_ucLoopRunFlag = 0;
							// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
							if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
							{
								g_ucOnceRunFlag = 0;
								g_ucLoopRunFlag = 1;
								g_ucJumpRunFlag = 0;
		                                                g_ucTaskCnt = 0;
								g_uiTaskTime1 = 0;
								g_uiTaskTime2 = 0;
								g_siTaskDeltaTime = 0;
								stRobot.emRobotTask=TASK17;
								//������ת�Ĳ���
							}
						}
				break;
				
			case TASK17:
				
				if(g_ucOnceRunFlag == 0)
					{
						g_ucOnceRunFlag = 1;
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬

						/*while(IrComm1IsDataInRxBuf())
						{
							IrComm1GetRxBufDat();
						}*/
						
					}
				
				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���
					/******************************����ˮƽ���ǰ��ָ��***************************************/
						 if(KEY_F5(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
						        
						       	stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
			 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid, g_fpActionM1PotFB + g_fpM1DeltaPos2);
							}	

						if(KEY_F4(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
							
								stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathMid, g_fpActionM1PotFB - g_fpM1DeltaPos2);
							 }
					
				/********************************������ֱ�������ָ��*******************************************************/
						if(KEY_F1(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB + g_fpM2DeltaPos2);
									
							}

						if(KEY_F8(usKeyValue))
							{
								 //�ڴ˵ȴ������ͷ�
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //�ڴ˴����������Ӧ
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB - g_fpM2DeltaPos2);
								
							}



						
					while(IrComm1IsDataInRxBuf())
						{
						     switch(IrComm1GetRxBufDat())
						     {
							 	case MC_UP_CMD:
									g_ucMC_M1UpCnt++;
									break;
								case MC_DOWN_CMD:
									g_ucMC_M1DownCnt++;
									break;
								case MC_OUT_CMD:
									g_ucMC_M2OutCnt++;
									break;
								case MC_IN_CMD:
									g_ucMC_M2InCnt++;
									break;
							       case MC_CATCH_CMD:
								   	g_ucMC_CatchCnt++;
								   	break;
								case MC_RELEASE_CMD:
									g_ucMC_ReleaseCnt++;
									break;
								case MC_ROTFIT_CMD:
									g_ucMC_RotFitCnt++;
									break;
								case MC_ROTOPST_CMD:
									g_ucMC_RotOpstCnt++;
									break;
								case MC_ROT_VALVE_CMD:
									g_ucMC_Rot_ValveCnt++;
									break;
								default:
									break;
						
				    			 }
						}

				/******************************����ˮƽ���ǰ��ָ��***************************************/
						if(g_ucMC_M1UpCnt >= g_ucMC_CntTime )
						{
							g_ucMC_M1UpCnt  = 0;
							stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
							g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
		 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid, g_fpActionM1PotFB + g_fpM1DeltaPos);
							
						}

						if(g_ucMC_M1DownCnt>= g_ucMC_CntTime )
						{
							g_ucMC_M1DownCnt  = 0;
							stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
							g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathMid, g_fpActionM1PotFB - g_fpM1DeltaPos);
						}
				/********************************������ֱ�������ָ��*******************************************************/
						if(g_ucMC_M2OutCnt >= g_ucMC_CntTime  )
						{
							g_ucMC_M2OutCnt = 0;
							g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB + g_fpM2DeltaPos);
							
						}

						if(g_ucMC_M2InCnt >= g_ucMC_CntTime )
						{
							g_ucMC_M2InCnt = 0;
							g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB - g_fpM2DeltaPos);
							
						}

				/*****************************����ץȡ��ͷָ��********************************************/
						if(g_ucMC_CatchCnt >= g_ucMC_CntTime)
						{
							g_ucMC_CatchCnt = 0;
							OPEN_VALVE(TOP_CATCH_VALVE);
						}

						if(g_ucMC_ReleaseCnt >= g_ucMC_CntTime)
						{
							g_ucMC_ReleaseCnt = 0;
							CLOSE_VALVE(TOP_CATCH_VALVE);
						}

					/********************�Զ���תָ��*******************************/
						if(g_ucMC_Rot_ValveCnt >= g_ucMC_CntTime)
						{
							g_ucMC_Rot_ValveCnt = 0;
							if(IS_VALVE_OPEN(TOP_CATCH_VALVE))
								{
									// �ڴ��ж��Ƿ���ת	
									g_ucJumpRunFlag =1;
								}
							
						}

						if(g_ucMC_AutoCnt  >= g_ucMC_CntTime)
							{
								g_ucMC_AutoCnt = 0;
							}
	
											
					}


					if(g_ucJumpRunFlag == 1)
						{
		                                    g_ucLoopRunFlag = 0;
							// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
							if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
							{
								g_ucOnceRunFlag = 0;
								g_ucLoopRunFlag = 1;
								g_ucJumpRunFlag = 0;
		                                                g_ucTaskCnt = 0;
								g_uiTaskTime1 = 0;
								g_uiTaskTime2 = 0;
								g_siTaskDeltaTime = 0;
								stRobot.emRobotTask=TASK18;
								//������ת�Ĳ���
							}
						}
				
				break;
			
			case TASK18://ȡ����ͷ֮��,M1�������������ͷ�ó���ɽ
				if(g_ucOnceRunFlag == 0)
					{
						if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))

							{
							
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
		 						SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_PICKUPTOPBAN_C*M1COFF_POT);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						else
							{

								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
		 						SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_PICKUPTOPBAN_C*M1COFF_POT);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							
							}
						
						stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
						g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
 						SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,g_fpActionM2PotFB -40*M2COFF_POT);//ˮƽ�������һ�ξ��룬��ֹ��ͷ����ɽ
					       stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

						
						g_ucOnceRunFlag = 1;
						
					}

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���
						//g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
						if(stAction_M1.emPathState==PATH_END)
						//(g_fpActionM2PotFB>200.4043f)
							{
								
							g_ucJumpRunFlag =1;
								
							}
						// �ڴ��ж��Ƿ���ת

					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							stRobot.emRobotTask=TASK19;
							//������ת�Ĳ���
						}
					}
				break;

			case TASK19://��ʱ�������κβ���
				if(g_ucOnceRunFlag == 0)
					{

						g_ucOnceRunFlag = 1;
						while(IrComm1IsDataInRxBuf())
						{
							IrComm1GetRxBufDat();
						}
						
					}

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���
						while(IrComm1IsDataInRxBuf())
						{
						     switch(IrComm1GetRxBufDat())
						     {
							 	case MC_UP_CMD:
									g_ucMC_M1UpCnt++;
									break;
								case MC_DOWN_CMD:
									g_ucMC_M1DownCnt++;
									break;
								case MC_OUT_CMD:
									g_ucMC_M2OutCnt++;
									break;
								case MC_IN_CMD:
									g_ucMC_M2InCnt++;
									break;
							       case MC_CATCH_CMD:
								   	g_ucMC_CatchCnt++;
								   	break;
								case MC_RELEASE_CMD:
									g_ucMC_ReleaseCnt++;
									break;
								case MC_ROTFIT_CMD:
									g_ucMC_RotFitCnt++;
									break;
								case MC_ROTOPST_CMD:
									g_ucMC_RotOpstCnt++;
									break;
								case MC_AUTO_CMD:
									g_ucMC_AutoCnt++;
									break;
								default:
									break;
						
				    			 }
						}

				/******************************����ˮƽ���ǰ��ָ��***************************************/
						if(g_ucMC_M1UpCnt >= g_ucMC_CntTime )
						{
							g_ucMC_M1UpCnt  = 0;
							stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
							g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
		 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid, g_fpActionM1PotFB + g_fpM1DeltaPos);
							
						}

						if(g_ucMC_M1DownCnt>= g_ucMC_CntTime )
						{
							g_ucMC_M1DownCnt  = 0;
							stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
							g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathMid, g_fpActionM1PotFB - g_fpM1DeltaPos);
						}
				/********************************������ֱ�������ָ��*******************************************************/
						if(g_ucMC_M2OutCnt >= g_ucMC_CntTime  )
						{
							g_ucMC_M2OutCnt = 0;
							g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB + g_fpM2DeltaPos);
							
						}

						if(g_ucMC_M2InCnt >= g_ucMC_CntTime )
						{
							g_ucMC_M2InCnt = 0;
							g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB - g_fpM2DeltaPos);
							
						}

				/*****************************����ץȡ��ͷָ��********************************************/
						if(g_ucMC_CatchCnt >= g_ucMC_CntTime)
						{
							g_ucMC_CatchCnt = 0;
							OPEN_VALVE(TOP_CATCH_VALVE);
						}

						if(g_ucMC_ReleaseCnt >= g_ucMC_CntTime)
						{
							g_ucMC_ReleaseCnt = 0;
							CLOSE_VALVE(TOP_CATCH_VALVE);
						}

					/********************�Զ���תָ��*******************************/
						if(g_ucMC_AutoCnt >= g_ucMC_CntTime)
						{
							g_ucMC_AutoCnt = 0;
							if(IS_VALVE_OPEN(TOP_CATCH_VALVE))
								{
									// �ڴ��ж��Ƿ���ת	
									g_ucJumpRunFlag =1;
								}
							
						}								
						// �ڴ��ж��Ƿ���ת
					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							stRobot.emRobotTask=TASK20;
							//������ת�Ĳ���
						}
					}
				break;
			
				case TASK20://ȡ����ͷ���ֱۻ�ת��ȡ��ͷ��λ�ã�����ˮƽ���ǰ��
				if(g_ucOnceRunFlag == 0)
					{
						
						g_ucOnceRunFlag = 1;

						CLOSE_VALVE(ROT_VALVE);
						g_uiTaskTime1 = GetCurTime();
						g_uiTaskTime2= GetCurTime();
						g_ucTaskFlag=0;
						
					}

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���
						if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))

							{
							
								if(IS_SWITCH_OFF(TOP_ROTAIRPOS_SWITCH))
									{
										g_ucTaskFlag=1;
										
									}
								if(((GetCurTime()-g_uiTaskTime1)>10000000)||(g_ucTaskFlag==1))//�˴�ʱ�����˸��ģ���ֹ
								//if((GetCurTime()-g_uiTaskTime1)>1000000)
									{
										stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
		 								SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_DROPTOPBAN_C*M2COFF_POT);//ˮƽ����쵽����ͷ��λ��
										stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
										// �ڴ��ж��Ƿ���ת
										
										g_ucJumpRunFlag =1;
									}
							}
						else
							{
								if(IS_SWITCH_OFF(TOP_ROTAIRPOS_SWITCH))
									{
										g_ucTaskFlag=1;
										
									}
								if(((GetCurTime()-g_uiTaskTime1)>10000000)||(g_ucTaskFlag==1))//�˴�ʱ�����˸��ģ���ֹ
								//if((GetCurTime()-g_uiTaskTime1)>1000000)
									{
										stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
		 								SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_DROPTOPBAN_C*M2COFF_POT);//ˮƽ����쵽����ͷ��λ��
										stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
										// �ڴ��ж��Ƿ���ת
										
										g_ucJumpRunFlag =1;
									}
							}
					}

				if(g_ucJumpRunFlag == 1)
					{
						//������ת�Ĳ���
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							//g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							stRobot.emRobotTask=TASK21;
							
						}
					}
				break;
			case TASK21://��ʱһ��ʱ��֮����ֱ����½�
				if(g_ucOnceRunFlag == 0)
					{
						
						g_ucOnceRunFlag = 1;
						g_CFlag6=0;
						g_ucTaskFlag=0;
						g_uiTaskTime1=GetCurTime();
						if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))

							{
							
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
		 						SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_DROPTOPBAN_C*M1COFF_POT);//��ֱ����½�������ͷ��λ��
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						else
							{
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
		 						SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_DROPTOPBAN_C*M1COFF_POT);//��ֱ����½�������ͷ��λ��
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
														
					}

				if(g_ucLoopRunFlag == 1)
					{

						if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))

							{
								// �ڴ����ѭ��ִ�д���
								if(GetCurTime()-g_uiTaskTime1>35000)
									{
										CLOSE_VALVE(ROT_VALVE);
									
									}

								
								if(IS_SWITCH_OFF(TOP_ROTAIRREVERSEPOS_SWITCH))
									{
										g_ucTaskFlag=1;
										OPEN_VALVE(ROT_VALVE);
									}
								if(((GetCurTime()-g_uiTaskTime1)>1500000)||(g_ucTaskFlag==1))//�˴�ʱ�����˸��ģ���ֹ
								
									{
										// �ڴ��ж��Ƿ���ת
										g_ucJumpRunFlag =1;
									}
								
							}
						else
							{
								// �ڴ����ѭ��ִ�д���
								if(GetCurTime()-g_uiTaskTime1>35000)
									{
										CLOSE_VALVE(ROT_VALVE);
										
									
									}
								if(IS_SWITCH_OFF(TOP_ROTAIRREVERSEPOS_SWITCH))
									{
										g_ucTaskFlag=1;
										OPEN_VALVE(ROT_VALVE);
										
									}
								if(((GetCurTime()-g_uiTaskTime1)>1500000)||(g_ucTaskFlag==1))//�˴�ʱ�����˸��ģ���ֹ
								
									{
										
										// �ڴ��ж��Ƿ���ת
										g_ucJumpRunFlag =1;
									}
								
							}
								
						
					}
				
				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{
							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							//g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							
							//������ת�Ĳ���
							stRobot.emRobotTask=TASK_INSERT_10;
						}
					}
				break;
				case TASK_INSERT_10:
					if(g_ucOnceRunFlag == 0)
					{
						g_uiTaskTime1=GetCurTime();
						OPEN_VALVE(ROT_VALVE);
						//������ת�Ĳ���
						g_ucOnceRunFlag = 1;
						g_ucTaskFlag=0;
						
						
					}
					if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���
						if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))

							{
								// �ڴ����ѭ��ִ�д���
								
								if((GetCurTime()-g_uiTaskTime1)>((GetCurTime()-g_uiTaskTime2)*g_fpValveCoff/100.0))//�˴�ʱ�����˸��ģ���ֹ
								
									{
										CLOSE_VALVE(ROT_VALVE);
										// �ڴ��ж��Ƿ���ת
										g_ucJumpRunFlag =1;
									}
							}
						else
							{
								// �ڴ����ѭ��ִ�д���
								
								if((GetCurTime()-g_uiTaskTime1)>((GetCurTime()-g_uiTaskTime2)*g_fpValveCoff/100.0))//�˴�ʱ�����˸��ģ���ֹ
								
									{
										CLOSE_VALVE(ROT_VALVE);
										// �ڴ��ж��Ƿ���ת
										g_ucJumpRunFlag =1;
									}
								
							}
							
						
						
					}
				
				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{
							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							
							//������ת�Ĳ���
							stRobot.emRobotTask=TASK22;
						}
					}
					break;

				case TASK22:

				if(g_ucOnceRunFlag == 0)
					{
						//������ת�Ĳ���
						g_ucOnceRunFlag = 1;
						while(IrComm1IsDataInRxBuf())
						{
						     IrComm1GetRxBufDat();
						}
						g_ucMC_ReleaseCnt = 0;
						
					}

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���

						while(IrComm1IsDataInRxBuf())
						{
						     switch(IrComm1GetRxBufDat())
						     {
								case MC_RELEASE_CMD:
									g_ucMC_ReleaseCnt++;
									break;
								default:
									break;
								
						     }
						
						}

						// �ڴ��ж��Ƿ���ת
						if(g_ucMC_ReleaseCnt>g_ucMC_CntTime)
							{
								CLOSE_VALVE(TOP_CATCH_VALVE);
								g_ucMC_ReleaseCnt=0;
								g_ucJumpRunFlag =1;
								
							}
						
					}


					if(g_ucJumpRunFlag == 1)
						{
							
		                                    g_ucLoopRunFlag = 0;
							// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
							if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
							{

								g_ucOnceRunFlag = 0;
								g_ucLoopRunFlag = 1;
								g_ucJumpRunFlag = 0;
		                                                g_ucTaskCnt = 0;
								g_uiTaskTime1 = 0;
								g_uiTaskTime2 = 0;
								g_siTaskDeltaTime = 0;
								stRobot.emRobotTask=TASK23;
								//������ת�Ĳ���
							}
						}
					break;
				case TASK23:
					
				if(g_ucOnceRunFlag == 0)
					{
						//������ת�Ĳ���
						g_ucOnceRunFlag = 1;
						g_uiTaskTime1=GetCurTime();
						if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))

							{
							
								stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
		 						SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,(M2_RED_DROPTOPBAN_C-50)*M2COFF_POT);//ˮƽ����쵽����ͷ��λ�ú�س�һ�ξ����ֹ�������
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						else
							{
								stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
		 						SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,(M2_BLUE_DROPTOPBAN_C-50)*M2COFF_POT);//ˮƽ����쵽����ͷ��λ�ú�س�һ�ξ����ֹ�������
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						
						
						
					}

				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���

						if((GetCurTime()-g_uiTaskTime1)>500000)
							{
								// �ڴ��ж��Ƿ���ת
							
								g_ucJumpRunFlag =1;
							}
					
					}


					if(g_ucJumpRunFlag == 1)
						{
							
		                                    g_ucLoopRunFlag = 0;
							// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
							if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
							{

								g_ucOnceRunFlag = 0;
								g_ucLoopRunFlag = 1;
								g_ucJumpRunFlag = 0;
		                                                g_ucTaskCnt = 0;
								g_uiTaskTime1 = 0;
								g_uiTaskTime2 = 0;
								g_siTaskDeltaTime = 0;
								stRobot.emRobotTask=TASK24;
								//������ת�Ĳ���
							}
						}
					break;	
				case TASK24:
					if(g_ucOnceRunFlag == 0)
						{
							//������ת�Ĳ���
							g_ucOnceRunFlag = 1;
							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
	 						SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,15*M1COFF_POT);//
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

						}

					if(g_ucLoopRunFlag == 1)
						{
							// �ڴ����ѭ��ִ�д���

							
							// �ڴ��ж��Ƿ���ת
						
							g_ucJumpRunFlag =1;
						
						}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
								if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
								{

									g_ucOnceRunFlag = 0;
									g_ucLoopRunFlag = 1;
									g_ucJumpRunFlag = 0;
			                                                g_ucTaskCnt = 0;
									g_uiTaskTime1 = 0;
									g_uiTaskTime2 = 0;
									g_siTaskDeltaTime = 0;
									stRobot.emRobotTask=TASK25;
									//������ת�Ĳ���
								}
							}
					break;
				case TASK25:
					if(g_ucOnceRunFlag == 0)
					{
						g_ucOnceRunFlag = 1;
					//�ڴ����ִֻ��һ�εĴ���,  Ҫȷ�ϸ��������״̬
						while(IrComm1IsDataInRxBuf())
						{
							IrComm1GetRxBufDat();
						}
						
					}
				
				if(g_ucLoopRunFlag == 1)
					{
						// �ڴ����ѭ��ִ�д���
						
					while(IrComm1IsDataInRxBuf())
						{
						     switch(IrComm1GetRxBufDat())
						     {
							 	case MC_UP_CMD:
									g_ucMC_M1UpCnt++;
									break;
								case MC_DOWN_CMD:
									g_ucMC_M1DownCnt++;
									break;
								case MC_OUT_CMD:
									g_ucMC_M2OutCnt++;
									break;
								case MC_IN_CMD:
									g_ucMC_M2InCnt++;
									break;
							       case MC_CATCH_CMD:
								   	g_ucMC_CatchCnt++;
								   	break;
								case MC_RELEASE_CMD:
									g_ucMC_ReleaseCnt++;
									break;
								case MC_ROTFIT_CMD:
									g_ucMC_RotFitCnt++;
									break;
								case MC_ROTOPST_CMD:
									g_ucMC_RotOpstCnt++;
									break;
								default:
									break;
						
				    			 }
						}

				/******************************����ˮƽ���ǰ��ָ��***************************************/
						if(g_ucMC_M1UpCnt >= g_ucMC_CntTime )
						{
							g_ucMC_M1UpCnt  = 0;
							stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
							g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
		 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid, g_fpActionM1PotFB + g_fpM1DeltaPos);
							
						}

						if(g_ucMC_M1DownCnt>= g_ucMC_CntTime  )
						{
							g_ucMC_M1DownCnt  = 0;
							stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
							g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathMid, g_fpActionM1PotFB - g_fpM1DeltaPos);
						}
				/********************************������ֱ�������ָ��*******************************************************/
						if(g_ucMC_M2OutCnt >= g_ucMC_CntTime  )
						{
							g_ucMC_M2OutCnt = 0;
							g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB + g_fpM2DeltaPos);
							
						}

						if(g_ucMC_M2InCnt >= g_ucMC_CntTime  )
						{
							g_ucMC_M2InCnt = 0;
							g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB - g_fpM2DeltaPos);
							
						}

				/*****************************����ץȡ��ͷָ��********************************************/
						if(g_ucMC_CatchCnt >= g_ucMC_CntTime)
						{
							g_ucMC_CatchCnt = 0;
							OPEN_VALVE(TOP_CATCH_VALVE);
						}

						if(g_ucMC_ReleaseCnt >= g_ucMC_CntTime)
						{
							g_ucMC_ReleaseCnt = 0;
							CLOSE_VALVE(TOP_CATCH_VALVE);
						}

											
					}


					if(g_ucJumpRunFlag == 1)
						{
		                                    g_ucLoopRunFlag = 0;
							// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
							if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
							{
								g_ucOnceRunFlag = 0;
								g_ucLoopRunFlag = 1;
								g_ucJumpRunFlag = 0;
		                                                g_ucTaskCnt = 0;
								g_uiTaskTime1 = 0;
								g_uiTaskTime2 = 0;
								g_siTaskDeltaTime = 0;
								//stRobot.emRobotTask=TASK25;
								//������ת�Ĳ���
							}
						}
					break;
					case TASK_TOPAIR1:
						if(g_ucOnceRunFlag == 0)
						{
							//������ת�Ĳ���
							g_ucOnceRunFlag = 1;
							CLOSE_VALVE(ROT_VALVE);
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
		 					SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_PICKUPTOPBAN_C*M2COFF_POT);//ˮƽ����쵽����ͷ��λ�ú�س�һ�ξ����ֹ�������
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
		 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathLow,M1_RED_CATCHTOPBAN_C*M1COFF_POT);
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
					
						}

						if(g_ucLoopRunFlag == 1)
							{
								// �ڴ����ѭ��ִ�д���

								if((stAction_M2.emPathState==PATH_END)&&(stAction_M1.emPathState==PATH_END))
									{
										// �ڴ��ж��Ƿ���ת
									
										g_ucJumpRunFlag =1;
									}
							
							}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
								if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
								{

									g_ucOnceRunFlag = 0;
									g_ucLoopRunFlag = 1;
									g_ucJumpRunFlag = 0;
			                                                g_ucTaskCnt = 0;
									g_uiTaskTime1 = 0;
									g_uiTaskTime2 = 0;
									g_siTaskDeltaTime = 0;
									stRobot.emRobotTask=TASK_TOPAIR2;
									//������ת�Ĳ���
								}
							}
						break;
					case TASK_TOPAIR2:
						if(g_ucOnceRunFlag == 0)
							{
								//������ת�Ĳ���
								g_ucOnceRunFlag = 1;
								OPEN_VALVE(ROT_VALVE);
				
							}

						if(g_ucLoopRunFlag == 1)
							{
								// �ڴ����ѭ��ִ�д���

								if(IS_SWITCH_OFF(TOP_ROTAIRPOS_SWITCH))
									{
										// �ڴ��ж��Ƿ���ת
									
										g_ucJumpRunFlag =1;
										g_ucAllContinueFlag = 1;
										CLOSE_VALVE(ROT_VALVE);
									}
							}

						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
								if(JS_R1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
								{

									g_ucOnceRunFlag = 0;
									g_ucLoopRunFlag = 1;
									g_ucJumpRunFlag = 0;
			                                                g_ucTaskCnt = 0;
									g_uiTaskTime1 = 0;
									g_uiTaskTime2 = 0;
									g_siTaskDeltaTime = 0;
									stRobot.emRobotTask=TASK_TOPAIR7;
									//������ת�Ĳ���
								}
							}
						break;
						case TASK_TOPAIR7:
							if(g_ucOnceRunFlag == 0)
							{
								//������ת�Ĳ���
								g_ucOnceRunFlag = 1;
								g_uiTaskTime1=GetCurTime();
								
				
							}

						if(g_ucLoopRunFlag == 1)
							{
								// �ڴ����ѭ��ִ�д���

								if(GetCurTime()-g_uiTaskTime1>200000)
									{
										// �ڴ��ж��Ƿ���ת
										OPEN_VALVE(ROT_VALVE);
									
										g_ucJumpRunFlag =1;
										g_ucAllContinueFlag = 0;
									}
							
							}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
								if(JS_R1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
								{

									g_ucOnceRunFlag = 0;
									g_ucLoopRunFlag = 1;
									g_ucJumpRunFlag = 0;
			                                                g_ucTaskCnt = 0;
									g_uiTaskTime1 = 0;
									g_uiTaskTime2 = 0;
									g_siTaskDeltaTime = 0;
									stRobot.emRobotTask=TASK_TOPAIR3;
									//������ת�Ĳ���
								}
							}
							break;
						case TASK_TOPAIR3:
						if(g_ucOnceRunFlag == 0)
							{
								//������ת�Ĳ���
								g_ucOnceRunFlag = 1;
								CLOSE_VALVE(ROT_VALVE);

								g_uiTaskTime2 = GetCurTime();
							}

						if(g_ucLoopRunFlag == 1)
							{
								// �ڴ����ѭ��ִ�д���
								

								if(IS_SWITCH_OFF(TOP_ROTAIRPOS_SWITCH)) 
									{
										// �ڴ��ж��Ƿ���ת
									
										g_ucJumpRunFlag =1;
										
										g_ucAllContinueFlag = 1;
									}
							
							}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
								if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
								{

									g_ucOnceRunFlag = 0;
									g_ucLoopRunFlag = 1;
									g_ucJumpRunFlag = 0;
			                                                g_ucTaskCnt = 0;
									g_uiTaskTime1 = 0;
									//g_uiTaskTime2 = 0;
									g_siTaskDeltaTime = 0;
									stRobot.emRobotTask=TASK_TOPAIR4;
									//������ת�Ĳ���
								}
							}
							break;
						case TASK_TOPAIR4:
						if(g_ucOnceRunFlag == 0)
							{
								//������ת�Ĳ���
								g_ucOnceRunFlag = 1;
								g_uiTaskTime1=GetCurTime();
								//OPEN_VALVE(ROT_VALVE);
							}

						if(g_ucLoopRunFlag == 1)
							{
								// �ڴ����ѭ��ִ�д���

								//if(GetCurTime()-g_uiTaskTime1>25000)

									{
										CLOSE_VALVE(ROT_VALVE);
										// �ڴ��ж��Ƿ���ת
										
										g_ucJumpRunFlag =1;
										g_ucAllContinueFlag =1;
									}
							}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
								if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
								{

									g_ucOnceRunFlag = 0;
									g_ucLoopRunFlag = 1;
									g_ucJumpRunFlag = 0;
			                                                g_ucTaskCnt = 0;
									g_uiTaskTime1 = 0;
									//g_uiTaskTime2 = 0;
									g_siTaskDeltaTime = 0;
									stRobot.emRobotTask=TASK_TOPAIR5;
									//������ת�Ĳ���
								}
							}
							break;
						case TASK_TOPAIR5:
						if(g_ucOnceRunFlag == 0)
							{
								//������ת�Ĳ���
								g_ucOnceRunFlag = 1;
								CLOSE_VALVE(ROT_VALVE);
							}

						if(g_ucLoopRunFlag == 1)
							{
								// �ڴ����ѭ��ִ�д���

								if(IS_SWITCH_OFF(TOP_ROTAIRREVERSEPOS_SWITCH)) 
									{
										// �ڴ��ж��Ƿ���ת
									
										g_ucJumpRunFlag =1;
										g_TimeTemp=GetCurTime()-g_uiTaskTime2;
										g_ucAllContinueFlag = 1;
									}
							
							}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
								if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
								{

									g_ucOnceRunFlag = 0;
									g_ucLoopRunFlag = 1;
									g_ucJumpRunFlag = 0;
			                                                g_ucTaskCnt = 0;
									g_uiTaskTime1 = 0;
									g_uiTaskTime2 = 0;
									g_siTaskDeltaTime = 0;
									stRobot.emRobotTask=TASK_TOPAIR6;
									//������ת�Ĳ���
								}
							}
							break;
						case TASK_TOPAIR6:
						if(g_ucOnceRunFlag == 0)
							{
								//������ת�Ĳ���
								g_ucOnceRunFlag = 1;
								g_uiTaskTime1=GetCurTime();
								OPEN_VALVE(ROT_VALVE);
							}

						if(g_ucLoopRunFlag == 1)
							{
								// �ڴ����ѭ��ִ�д���

								if(GetCurTime()-g_uiTaskTime1>(g_TimeTemp*g_fpValveCoff/100.0))//120

									{
										CLOSE_VALVE(ROT_VALVE);
										// �ڴ��ж��Ƿ���ת
										
										g_ucJumpRunFlag =1;
										g_ucAllContinueFlag =0;
									}
							}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// �ڴ������ת�����ڴ�Ҫ���RunFlag����������Ͷ�ʱ,������������������ת�ж�
								if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
								{

									g_ucOnceRunFlag = 0;
									g_ucLoopRunFlag = 1;
									g_ucJumpRunFlag = 0;
			                                                g_ucTaskCnt = 0;
									g_uiTaskTime1 = 0;
									g_uiTaskTime2 = 0;
									g_siTaskDeltaTime = 0;
									stRobot.emRobotTask=TASK_TOPAIR1;
									//������ת�Ĳ���
								}
							}
							break;
							
			
			default:
				
				break;
		}
		
		OSTimeDly(5);		
	}
}



/****************************************************************************************************
�������ƣ�NavTask ()
�����ܣ���������
****************************************************************************************************/
void NavTask (void *pdata)
{
//	stRobot.pstCurPath = (ST_PATH*) &stNavPath[7][0];
	stRobot.emNavState = NAV_OFF;//�ֶ�����

	while(1)
	{

                     switch(stRobot.emNavState)
			{
				case NAV_MANUAL:
					DesRobotV(&stJSValue, &g_stRobotDesV);
					ManualBaseVeltAllocate(&stRobot,&g_stRobotDesV, g_siDirAngle);
					stRobot.emBaseState = BASE_CLOSE_LOOP_CTRL;
					break;
				
				case NAV_LINE:
				//����ø�״̬Ӧ��ȷ����·��������·����״̬����ΪPATH_RUN
					NavLineEx1(&stRobot);
					stRobot.emBaseState = BASE_CLOSE_LOOP_CTRL;
					break;
				

				case NAV_BASE_DIRECT:
					// ʲôҲ���������ڸ�״̬�У�ֱ���޸��������ӵ��ٶ�
					stRobot.emBaseState = BASE_CLOSE_LOOP_CTRL;
					break;
				case NAV_XYQ:
					// �ڴ�״̬�У����ʹ��Ϊ1�����޸�g_siRobot*DesShadow,���� ������õ�pid��ֵ��
					//ʹ��λΪ2ʱ�����ⲿ�޸�VeltDes,�ܹ�ʵ��
					//Ϊ0ʱ��Ԥ���ٶ�Ϊ0
					if(g_ucRobotXEnable == 1)
						{
							CopyIntSlowly(&g_siRobotXDes, &g_siRobotXDesShadow, 1);
							stRobot.stNavPidLine.stPidTrvs.fpE = g_siRobotXDes - stRobot.stPot.ssPosX;
							CalPosPID(&stRobot.stNavPidLine.stPidTrvs);
							stRobot.stVeltDes.fpVx = stRobot.stNavPidLine.stPidTrvs.fpU;
						}
					else if(g_ucRobotXEnable == 2)
						{
							// ʲô������д�����ⲿ�޸�  X ������ٶ�,ʵ���ٶȷ��䣬�����ڴ��γ�б���ź�
						}
					else
						{
							stRobot.stVeltDes.fpVx = 0;
						}

					

					if(g_ucRobotYEnable == 1)
						{
							CopyIntSlowly(&g_siRobotYDes,&g_siRobotYDesShadow, 1);
							stRobot.stNavPidLine.stPidVtc.fpE = g_siRobotYDes -stRobot.stPot.ssPosY;
							CalPosPID(&stRobot.stNavPidLine.stPidVtc);
							stRobot.stVeltDes.fpVy = stRobot.stNavPidLine.stPidVtc.fpU;
						}
					else if(g_ucRobotYEnable == 2)
						{
							// ʲô������д�����ⲿ�޸�  Y ������ٶȣ�ʵ���ٶȷ���
						}
					else
						{
							stRobot.stVeltDes.fpVy = 0;
						}
						

					if(g_ucRobotQEnable == 1)
						{
							CopyIntSlowly(&g_siRobotQDes,&g_siRobotQDesShadow,1);//ԭ����2�����ڸ�Ϊ1
							stRobot.stNavPidLine.stPidRot.fpE = g_siRobotQDes - stRobot.stPot.ssPosQ;
							CalPosPID(&stRobot.stNavPidLine.stPidRot);
							stRobot.stVeltDes.fpW= stRobot.stNavPidLine.stPidRot.fpU;
						}
					else if(g_ucRobotQEnable == 2)
						{
							// ʲô������д�����ⲿ�޸�  Y ������ٶȣ�ʵ���ٶȷ���
						}
					else
						{
							stRobot.stVeltDes.fpW = 0;
						}
						//�ڴ˷����ٶ�
						AutoBaseVeltAllocate(&stRobot, &stRobot.stVeltDes, 0);

						stRobot.emBaseState = BASE_CLOSE_LOOP_CTRL;
					break;
			       case NAV_NULL:
				   	
				   	//ʲôҲ�����������޸ĵ��̵�״̬������״̬Ϊ��
				   	break;
				
				default:
					stRobot.emBaseState = BASE_BREAK;				
					break;
			}
	 
	#if 0
		DoubleVerticalOMNILocate(&stDoubleOMNI, &stRobot);
	//	DoubleVerticalOMNILocate(&stDoubleOMNI, &stRobot);

		if(stRobot.emState == ROBOT_MOVE)//����������
		{
			switch(stRobot.emNavState)
			{
				case NAV_MANUAL:
					ManualCtrl(&stRobot, &stRobotDesV);//����
					stRobot.emBaseState = BASE_CLOSE_LOOP_CTRL;
					break;
				case NAV_ERROR:
					stRobot.emBaseState = BASE_BREAK;
					break;
				case NAV_OFF:
					stRobot.emBaseState = BASE_STOP;
					break;
				default://�Զ�����
					if(stRobot.pstCurPath->ucType == CIRCLE || stRobot.pstCurPath->ucType == CIRCLE2)
					{
						stRobot.emNavState = NAV_CIR;
						NavCircle(&stRobot);
					}
					else
					{
						stRobot.emNavState = NAV_LINE;
						NavLine(&stRobot);
					}
					stRobot.emBaseState = BASE_CLOSE_LOOP_CTRL;					
					break;
			}
		}
		else
		{
		}
		#endif
		OSTimeDly(5);
	}
}

/****************************************************************************************************
��������:�Ե��̵ĵ�����п���
����
****************************************************************************************************/
void BaseCtrlTask (void *pdata)
{
	stRobot.emBaseState = BASE_BREAK;
	stRobot.stBaseWaMotor.fpVeltDes = 0;
	stRobot.stBaseWbMotor.fpVeltDes = 0;
	stRobot.stBaseWcMotor.fpVeltDes = 0;
	while(1)
	{
		
		stRobot.stBaseWaMotor.fpVeltFB = CalVelt(&stRobot.stBaseWaMotor);
		stRobot.stBaseWbMotor.fpVeltFB = CalVelt(&stRobot.stBaseWbMotor);
		stRobot.stBaseWcMotor.fpVeltFB = CalVelt(&stRobot.stBaseWcMotor);
		
		switch (stRobot.emBaseState)
		{
			case BASE_CLOSE_LOOP_CTRL:
				VeltLoopCtrl(&stRobot.stBaseWaMotor);
				VeltLoopCtrl(&stRobot.stBaseWbMotor);
				VeltLoopCtrl(&stRobot.stBaseWcMotor);
				break;
			case BASE_BREAK:
				stRobot.stBaseWaMotor.TurnMotor(0);
				stRobot.stBaseWbMotor.TurnMotor(0);
                  		stRobot.stBaseWcMotor.TurnMotor(0);
				break;
			case BASE_OPEN_LOOP_CTRL:
				stRobot.stBaseWaMotor.TurnMotor(stRobot.stBaseWaMotor.ssPwmDuty);
				stRobot.stBaseWbMotor.TurnMotor(stRobot.stBaseWbMotor.ssPwmDuty);
                     	stRobot.stBaseWcMotor.TurnMotor(stRobot.stBaseWcMotor.ssPwmDuty);
				break;
			case BASE_STOP_QUICKLY:
				stRobot.stBaseWaMotor.fpVeltDes = 0;
				stRobot.stBaseWbMotor.fpVeltDes = 0;
				stRobot.stBaseWcMotor.fpVeltDes = 0;
				VeltLoopCtrl(&stRobot.stBaseWaMotor);
				VeltLoopCtrl(&stRobot.stBaseWbMotor);
               		VeltLoopCtrl(&stRobot.stBaseWcMotor);
				break;
			case BASE_STOP_SLOWLY:
				//Լ��400ms��ֹͣ
				stRobot.stBaseWaMotor.fpVeltDes *= 0.99;
				stRobot.stBaseWbMotor.fpVeltDes *= 0.99;
				stRobot.stBaseWcMotor.fpVeltDes *= 0.99;
				if(fabs(stRobot.stBaseWaMotor.fpVeltDes) < 5)
				{
					stRobot.stBaseWaMotor.fpVeltDes = 0;
				}
				if(fabs(stRobot.stBaseWbMotor.fpVeltDes) < 5)
				{
					stRobot.stBaseWbMotor.fpVeltDes = 0;
				}
				if(fabs(stRobot.stBaseWcMotor.fpVeltDes) < 5)
				{
					stRobot.stBaseWcMotor.fpVeltDes = 0;
				}
				VeltLoopCtrl(&stRobot.stBaseWaMotor);
				VeltLoopCtrl(&stRobot.stBaseWbMotor);
               		VeltLoopCtrl(&stRobot.stBaseWcMotor);			
				break;
			default:
				stRobot.stBaseWaMotor.TurnMotor(0);
				stRobot.stBaseWbMotor.TurnMotor(0);
                      	stRobot.stBaseWcMotor.TurnMotor(0);
		}
	
		OSTimeDly(2);
	}
}
/****************************************************************************************************
�������ƣ�ActionMotorCtrlTask()
�����ܣ�ִ�е��PID����
****************************************************************************************************/
void ActionMotorCtrlTask (void *pdata)
{
	stAction_M1.emState = MOTOR_BREAK;
	stAction_M1.pstCurPath =&g_stActionM1DymPathMid;
	stAction_M1.emPathState = PATH_RUN;
	
	stAction_M2.emState = MOTOR_BREAK;	
	stAction_M2.pstCurPath = &g_stActionM2DymPathMid;
	stAction_M2.emPathState = PATH_RUN;
	
	stAction_M3.emState = MOTOR_BREAK;	
	stAction_M3.pstCurPath =NULL;
	stAction_M3.emPathState = PATH_RUN;
	
	while(1)
	{
             //��λ�ñջ��У��Ѿ����ڼ����ٶȣ������������ٶ����̫��
             //ֻ����VeltLoopCtrl�ĺ�����Ҫ���ü����ٶȺ���CalVelt
		/*
		stAction_M1.fpVeltFB = CalVelt(&stAction_M1);
		stAction_M2.fpVeltFB = CalVelt(&stAction_M2);
		stAction_M3.fpVeltFB = CalVelt(&stAction_M3);
              */
	
		switch(stAction_M1.emState)
		{
			case MOTOR_PATH_CLOSE_LOOP_CTRL:
				DiffAutoPotLoopCtrl(&stAction_M1);
				break;
			case MOTOR_POS_CLOSE_LOOP_CTRL:
				PotDoubleLoopCtrl(&stAction_M1);
				break;
			case MOTOR_VELT_CLOSE_LOOP_CTRL:
				stAction_M1.fpVeltFB = CalVelt(&stAction_M1);
				CopyFloatSlowly(&stAction_M1.fpVeltDes, &g_fpActionM1VeltDesShadow,10);
				VeltLoopCtrl(&stAction_M1);
				break;
			case MOTOR_OPEN_LOOP_CTRL:
				stAction_M1.TurnMotor(stAction_M1.ssPwmDuty);
				break;
			case MOTOR_BREAK:
				stAction_M1.TurnMotor(0);
				break;
			case MOTOR_STOP_QUICKLY:
				stAction_M1.fpVeltDes = 0;
				stAction_M1.fpVeltFB = CalVelt(&stAction_M1);
				VeltLoopCtrl(&stAction_M1);
				break;
			case MOTOR_STOP_SLOWLY:
				stAction_M1.fpVeltDes *=0.8;
				if(fabs(stAction_M1.fpVeltDes)<5)
				{
					stAction_M1.fpVeltDes = 0;
				}
				stAction_M1.fpVeltFB = CalVelt(&stAction_M1);
				VeltLoopCtrl(&stAction_M1);
				break;
			case MOTOR_OTHER:
				break;			
			default:
				break;
		}
		switch(stAction_M2.emState)
		{
			case MOTOR_PATH_CLOSE_LOOP_CTRL:
				DiffAutoPotLoopCtrl(&stAction_M2);
				break;
			case MOTOR_POS_CLOSE_LOOP_CTRL:
				PotDoubleLoopCtrl(&stAction_M2);
				break;
			case MOTOR_VELT_CLOSE_LOOP_CTRL:
				stAction_M2.fpVeltFB = CalVelt(&stAction_M2);
				CopyFloatSlowly(&stAction_M2.fpVeltDes, &g_fpActionM2VeltDesShadow,2);
				VeltLoopCtrl(&stAction_M2);
				break;
			case MOTOR_OPEN_LOOP_CTRL:
				stAction_M2.TurnMotor(stAction_M2.ssPwmDuty);
				break;
			case MOTOR_BREAK:
				stAction_M2.TurnMotor(0);
				break;
			case MOTOR_STOP_QUICKLY:
				stAction_M2.fpVeltDes = 0;
				stAction_M2.fpVeltFB = CalVelt(&stAction_M2);
				VeltLoopCtrl(&stAction_M2);
				break;
			case MOTOR_STOP_SLOWLY:
				stAction_M2.fpVeltDes *=0.8;
				if(fabs(stAction_M2.fpVeltDes)<5)
				{
					stAction_M2.fpVeltDes = 0;
				}
				stAction_M2.fpVeltFB = CalVelt(&stAction_M2);
				VeltLoopCtrl(&stAction_M2);
				break;
			case MOTOR_OTHER:
				break;			
			default:
				break;
		}
		switch(stAction_M3.emState)
		{
			case MOTOR_PATH_CLOSE_LOOP_CTRL:
				DiffAutoPotLoopCtrl(&stAction_M3);
				break;
			case MOTOR_POS_CLOSE_LOOP_CTRL:
				PotDoubleLoopCtrl(&stAction_M3);
				break;
			case MOTOR_VELT_CLOSE_LOOP_CTRL:
				stAction_M3.fpVeltFB = CalVelt(&stAction_M3);
				
				VeltLoopCtrl(&stAction_M3);
				break;
			case MOTOR_OPEN_LOOP_CTRL:
				stAction_M3.TurnMotor(stAction_M3.ssPwmDuty);
				break;
			case MOTOR_BREAK:
				stAction_M3.TurnMotor(0);
				break;
			case MOTOR_STOP_QUICKLY:
				stAction_M3.fpVeltDes = 0;
				stAction_M3.fpVeltFB = CalVelt(&stAction_M3);
				VeltLoopCtrl(&stAction_M3);
				break;
			case MOTOR_STOP_SLOWLY:
				stAction_M3.fpVeltDes *=0.8;
				if(fabs(stAction_M3.fpVeltDes)<5)
				{
					stAction_M3.fpVeltDes = 0;
				}
				stAction_M3.fpVeltFB = CalVelt(&stAction_M3);
				VeltLoopCtrl(&stAction_M3);
				break;
			case MOTOR_OTHER:
				break;			
			default:
				break;
		}
		
		OSTimeDly(2);
	}
}


void InitMode(UCHAR8 ucMode)
{
	//�ս����ģʽʱ����ɸ����ʼ������
	switch(ucMode)
           {
                      case C_MODE_INIT_0:
/*---------------��ʼ��ģʽ������ѡ�������ģʽ-----------------------*/

                            stRobot.emRobotTask = ALL_BREAK;
                            g_ucCurLcdPage = 0;
				break;
			case C_MODE_TEST_SENSOR_1:
/*---------------�������ݡ����ء��������ֱ�-----------------------------*/
                             g_ucCurLcdPage = 1;
                            stRobot.emRobotTask = ALL_BREAK;
				break;
			case C_MODE_TEST_CODER_2:
/*---------------��������---------------------------------------------------------*/
                           g_ucCurLcdPage = 2;
                            stRobot.emRobotTask = ALL_BREAK;

				break;
			case C_MODE_TEST_LINE_3:
/*---------------����Ѳ��---------------------------------------------------------*/
                            g_ucCurLcdPage = 3;
                            stRobot.emRobotTask = ALL_BREAK;

				break;
			case C_MODE_CAL_GYRO_4:
/*---------------�궨����---------------------------------------------------------*/
                       g_ucCurLcdPage = 4;
                       stRobot.emRobotTask =NO_ACTION;
			  g_siDirAngle = 0;
			  
			  stAction_M1.emState = MOTOR_BREAK;
			  stAction_M2.emState = MOTOR_BREAK;
			  stAction_M3.emState = MOTOR_BREAK;
			  
			  stRobot.emNavState = NAV_MANUAL;

				break;
			case C_MODE_MANUAL_5:
/*---------------�ֶ�����---------------------------------------------------------*/

                       g_ucCurLcdPage = 5;
                       stRobot.emRobotTask =NO_ACTION;
			  g_siDirAngle = 0;
			  
			  
			  SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathLow, 10);
			  SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathLow, 10);
			  
			  stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
			  stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
			  stAction_M3.emState = MOTOR_BREAK;
			  
			  stRobot.emNavState = NAV_MANUAL;

				break;
			case C_MODE_LOW_SPEED_6:
/*---------------�Զ�����ģʽ---------------------------------------------------*/
			
                      g_ucCurLcdPage = 6;
/***************************************����L2��·��**********************************************************/
			if((stRobot.emRobotTactic==RED_FIELD)
				||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
				
				{
					g_pstL2Path=&stNavPathRed[0];//��A���ϵ�����·��
				}
			else if((stRobot.emRobotTactic==BLUE_FIELD)
				||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART))
				
				{
					g_pstL2Path=&stNavPathBlue[0];//��A���ϵ�����·��
				}
			
			else if((stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4))
				{
					g_pstL2Path=&stNavPathRed[9];//����·��
				}

			else if((stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART4))
				{
					g_pstL2Path=&stNavPathBlue[9];//����·��
				}
/*************************************************************************************************************/


/**********************************���ص��ϵ�һЩ·��������ץȡ���ͷ���ͷ***************************************************************************/			

		/*********************************�쳡·���ļ���****************************************************/
			if((stRobot.emRobotTactic==RED_FIELD)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
				{
					g_pstCatchBanPath=&stNavPathRed[1];
					g_pstReleaseBanPath=&stNavPathRed[2];
				}
			else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
				{
					g_pstCatchBanPath=&stNavPathRed[10];
					g_pstReleaseBanPath=&stNavPathRed[11];
					
				}
			else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
				{
					g_pstCatchBanPath=&stNavPathRed[17];
					g_pstReleaseBanPath=&stNavPathRed[18];
					
				}
			else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
				{
					g_pstGoBasketPath=&stNavPathRed[24];
				}
		/**********************************����·���ļ���**************************************************************************/
			else if((stRobot.emRobotTactic==BLUE_FIELD)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART))
				{
					g_pstCatchBanPath=&stNavPathBlue[1];
					g_pstReleaseBanPath=&stNavPathBlue[2];
				}
			else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
				{
					g_pstCatchBanPath=&stNavPathBlue[10];
					g_pstReleaseBanPath=&stNavPathBlue[11];
					
				}
			else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)
				{
					g_pstCatchBanPath=&stNavPathBlue[17];
					g_pstReleaseBanPath=&stNavPathBlue[18];
					
				}
			else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART4)
				{
					g_pstGoBasketPath=&stNavPathBlue[24];	
				}

				break;
				
			case C_MODE_MID_SPEED_7:
/*---------------�Զ�����ģʽ---------------------------------------------------*/
                      g_ucCurLcdPage = 7;

/***************************************����L2��·��**********************************************************/
			if((stRobot.emRobotTactic==RED_FIELD)
				||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
				{
					g_pstL2Path=&stNavPathRed[3];//��A���ϵ�����·��
				}
			else if((stRobot.emRobotTactic==BLUE_FIELD)
				||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART))
				{
					g_pstL2Path=&stNavPathBlue[3];//��A���ϵ�����·��
				}

			else if((stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4))
				{
					g_pstL2Path=&stNavPathRed[9];//����·��
				}
			else if((stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART4))
				{
					g_pstL2Path=&stNavPathBlue[9];//����·��
				}
/****************************************************************************************************************/

/**********************************���ص��ϵ�һЩ·��������ץȡ���ͷ���ͷ***************************************************************************/			

		/*********************************�쳡·���ļ���****************************************************/		
			if((stRobot.emRobotTactic==RED_FIELD)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
				{
				
				
					g_pstCatchBanPath=&stNavPathRed[4];
					g_pstReleaseBanPath=&stNavPathRed[5];
				}
			else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
				{
					g_pstCatchBanPath=&stNavPathRed[12];
					g_pstReleaseBanPath=&stNavPathRed[13];
					
				}
			else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
				{
					g_pstCatchBanPath=&stNavPathRed[19];
					g_pstReleaseBanPath=&stNavPathRed[20];
					
				}
			else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
				{
					g_pstGoBasketPath=&stNavPathRed[24];
				}


/**********************************����·���ļ���**************************************************************************/
			else if((stRobot.emRobotTactic==BLUE_FIELD)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART))
				{
					g_pstCatchBanPath=&stNavPathBlue[4];
					g_pstReleaseBanPath=&stNavPathBlue[5];
				}
			else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
				{
					g_pstCatchBanPath=&stNavPathBlue[12];
					g_pstReleaseBanPath=&stNavPathBlue[13];
					
				}
			else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)
				{
					g_pstCatchBanPath=&stNavPathBlue[19];
					g_pstReleaseBanPath=&stNavPathBlue[20];
					
				}			


			else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART4)
				{
					g_pstGoBasketPath=&stNavPathBlue[24];
				}

				break;
			case C_MODE_TOP_SPEED_8:
				g_ucCurLcdPage = 8;
/*---------------�Զ�����ģʽ---------------------------------------------------*/
                   /***************************************����L2��·��**********************************************************/
			if((stRobot.emRobotTactic==RED_FIELD)
				||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
				{
					g_pstL2Path=&stNavPathRed[6];//��A���ϵ�����·��
				}
			else if((stRobot.emRobotTactic==BLUE_FIELD)
				||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART))
				{
					g_pstL2Path=&stNavPathBlue[6];//��A���ϵ�����·��
				}

			else if((stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4))
				{
					g_pstL2Path=&stNavPathRed[9];//����·��
				}
			else if((stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART4))
				{
					g_pstL2Path=&stNavPathBlue[9];//����·��
				}
/****************************************************************************************************************/

/**********************************���ص��ϵ�һЩ·��������ץȡ���ͷ���ͷ***************************************************************************/			

		/*********************************�쳡·���ļ���****************************************************/		
			if((stRobot.emRobotTactic==RED_FIELD)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
				{
				
				
					g_pstCatchBanPath=&stNavPathRed[7];
					g_pstReleaseBanPath=&stNavPathRed[8];
				}
			else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
				{
					g_pstCatchBanPath=&stNavPathRed[14];
					g_pstReleaseBanPath=&stNavPathRed[15];
					
				}
			else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
				{
					g_pstCatchBanPath=&stNavPathRed[21];
					g_pstReleaseBanPath=&stNavPathRed[22];
					
				}
			else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
				{
					g_pstGoBasketPath=&stNavPathRed[24];
				}

/**********************************����·���ļ���**************************************************************************/
			else if((stRobot.emRobotTactic==BLUE_FIELD)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART))
				{
					g_pstCatchBanPath=&stNavPathBlue[7];
					g_pstReleaseBanPath=&stNavPathBlue[8];
				}
			else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
				{
					g_pstCatchBanPath=&stNavPathBlue[14];
					g_pstReleaseBanPath=&stNavPathBlue[15];
					
				}
			else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)
				{
					g_pstCatchBanPath=&stNavPathBlue[21];
					g_pstReleaseBanPath=&stNavPathBlue[22];
					
				}	
			else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART4)
				{
					g_pstGoBasketPath=&stNavPathBlue[24];
				}

				break;
			default:
				break;
           }
}

/****************************************************************************************************
�������ƣ�ReadKeyTask()
�����ܣ���ȡ�ֱ�/���̼�ֵ�Լ�һЩ������ֵ
****************************************************************************************************/
void ReadKeyTask (void *pdata)
{
       stJSValue.usJsLeft = 0x8080;
	stJSValue.usJsRight = 0x8080;
	OSTimeDly(500); //�ӳ٣������ϵ�˲��Ĳ�ȷ��״̬
	StopMEMSGryo();
	StartMEMSGryo();
	while(1)
	{     
		usKeyValue=PSKEY;
		stJSValue.usReserved = JOYSTICK_RESERVED;
		stJSValue.usJsState = JOYSTICK_STATE;

		if(stJSValue.usJsState == 0x73) //ģ������²Ÿ����ֱ�ģ��ֵ
		{
		     stJSValue.usJsLeft = JOYSTICK_LEFT;
		     stJSValue.usJsRight = JOYSTICK_RIGTH;
		}
		else
		{
			stJSValue.usJsLeft = 0x8080;
	              stJSValue.usJsRight = 0x8080;
		}


		/*****************************�������߳�ʱ���������ֵ�����*******************************/
		  if((stJSValue.usJsState == 0x73)&&JS_R2(stJSValue.usReserved))
		  	{
		  		 //�ڴ˵ȴ������ͷ�
				while(stJSValue.usReserved != 0xffff)
					{
						stJSValue.usReserved = JOYSTICK_RESERVED;
					}
				//�ڴ˴����������Ӧ
						         
				if(IS_VALVE_CLOSE(MID_CATCH_VALVE))
					{
						OPEN_VALVE(MID_CATCH_VALVE);
						OPEN_VALVE(TOP_CATCH_VALVE);
					}
				else
					{
						CLOSE_VALVE(MID_CATCH_VALVE);
						CLOSE_VALVE(TOP_CATCH_VALVE);
					}
			 }
		  
//     ��ҳ����ϵͳ��������ģʽ�޹�
//     ��ͣ������ģʽ�޹�
		 if(stJSValue.usReserved != 0xffff)
		 {
		 	 if(JS_L2(stJSValue.usReserved))
			   {
				   //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        g_ucCurLcdPage++;
					if(g_ucCurLcdPage>=g_ucTotalLcdPage)
						{
							g_ucCurLcdPage=0;
						}
		 	 }

			 if(JS_R2(stJSValue.usReserved))
			   {
				   //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					 if(g_ucCurLcdPage==0)
						 {
						      g_ucCurLcdPage =g_ucTotalLcdPage - 1;
						 }
					 else
					 	{
					 	    g_ucCurLcdPage--;
					 	}
				  
		 	 }
		 }

/************************************�����˳��ջ�**********************************************************/

		 if((stJSValue.usJsState == 0x73)&&JS_DOWN(stJSValue.usReserved)&&JS_B3(stJSValue.usReserved))
			{
				 while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				stRobot.emNavState =  NAV_NULL;
				stRobot.emBaseState=BASE_BREAK;
				//g_ucCurModeSel=C_MODE_MANUAL_5;
			
		 	}

/************************************���̽��бջ�**********************************************************/
		 if((stJSValue.usJsState == 0x73)&&JS_UP(stJSValue.usReserved)&&JS_B3(stJSValue.usReserved))
			{
				 while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				// g_ucCurModeSel=C_MODE_LOW_SPEED_6;
				stRobot.pstCurPath->ssEndX=stRobot.stPot.ssPosX;
				stRobot.pstCurPath->ssEndY=stRobot.stPot.ssPosY;
				stRobot.pstCurPath->ssEndQ=stRobot.stPot.ssPosQ;
				stRobot.emNavState =  NAV_LINE;
				stRobot.emBaseState=BASE_CLOSE_LOOP_CTRL;
		 	}
/**************************************************************************************************************/
		 if(KEY_F3(usKeyValue))
				{
					 //�ڴ˵ȴ������ͷ�
							        
					while(usKeyValue!= 0xffff)
					{
						 usKeyValue= PSKEY;
					}
					g_SelectState_Electric_On=1;	
				}	

 /***********************************����flash��***************************************************************************/
  		if(KEY_N7(usKeyValue))
				{
					//�ڴ˵ȴ������ͷ�
						        
					 while(usKeyValue!= 0xffff)
						{
							usKeyValue= PSKEY;
						}
					 g_ucFlashLoad=1;
			  	}
/***************************************************************************************************************/
/***********************************������ѡ���***************************************************************************/
#if 1
  		if(KEY_F2(usKeyValue))
				{
					//�ڴ˵ȴ������ͷ�
						        
					 while(usKeyValue!= 0xffff)
						{
							usKeyValue= PSKEY;
						}
					 if( g_ucKeyFlag==1)
					 	{
						 	g_ucKeyFlag=0;
					 	}
					 else
					 	{
					 		g_ucKeyFlag=1;
					 	}
					 g_siFlashValue[OFST_RED_OR_BLUE]=g_ucKeyFlag;
					 SaveAllWord32();
					 OSTimeDly(1500);
					 g_ucKeyFlag=g_siFlashValue[OFST_RED_OR_BLUE];
			  	}
#endif
           switch(g_ucCurModeSel)
           {
                      case C_MODE_INIT_0:
/*---------------��ʼ��ģʽ������ѡ�������ģʽ--�ֱ���ģʽ---------------------*/
                     if((stJSValue.usReserved != 0xffff)||(usKeyValue!=0xffff))
		          {
			        //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_UP(stJSValue.usReserved))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
						
					//�ڴ˴����������Ӧ
					         g_ucModeCnt++;
					         if(g_ucModeCnt >= g_ucTotalMode)
					         {
							 	 g_ucModeCnt = 0;
					         }
					        
				         }

					 if(KEY_F5(usKeyValue))
					         {
						        //�ڴ˵ȴ������ͷ�
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//�ڴ˴����������Ӧ
						         g_ucModeCnt++;
						         if(g_ucModeCnt >= g_ucTotalMode)
						         {
								 	 g_ucModeCnt = 7;
						         }
						        
					         }
				 if(JS_DOWN(stJSValue.usReserved))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					
					//�ڴ˴����������Ӧ
					        if(g_ucModeCnt == 0)
					        {
							g_ucModeCnt = g_ucTotalMode-1;	
					        }
						 else
						 {
						 	g_ucModeCnt--;
						 }
					         
				         }

				 if(g_ucKeyFlag==0)//�����쳡����
				 	{

						  if(JS_B1(stJSValue.usReserved)||KEY_N1(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
								while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							      //�ڴ˴����������Ӧ
							      stRobot.emRobotTactic = RED_FIELD_L2_RESTART1;
							      
							         
						         }
						  

						  if(JS_B2(stJSValue.usReserved)||KEY_N2(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
								while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							      //�ڴ˴����������Ӧ
							       stRobot.emRobotTactic = RED_FIELD_L2_RESTART2;
							      
							         
						         }



						  if(JS_B3(stJSValue.usReserved)||KEY_N3(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
								while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							      //�ڴ˴����������Ӧ
							       stRobot.emRobotTactic =RED_FIELD_L2_RESTART3;
							      
							         
						         }
						  


						  if(JS_B4(stJSValue.usReserved)||KEY_N4(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
								while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							      //�ڴ˴����������Ӧ
							      stRobot.emRobotTactic =RED_FIELD_L2_RESTART4;
							         
						         }

						   if(JS_LEFT(stJSValue.usReserved)||KEY_N0(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //�ڴ˴����������Ӧ
							      stRobot.emRobotTactic =RED_FIELD;
							         
						         }
						     if(JS_RIGHT(stJSValue.usReserved)||KEY_N5(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //�ڴ˴����������Ӧ
							      stRobot.emRobotTactic =RED_FIELD_S2_RESTART;
							         
						         }

							 if(JS_L1(stJSValue.usReserved)||KEY_N6(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //�ڴ˴����������Ӧ
							      stRobot.emRobotTactic =RED_FIELD_A_RESTART;
							         
						         }
							  if(JS_R1(stJSValue.usReserved))
						         {
							        //�ڴ˵ȴ������ͷ�
							     
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //�ڴ˴����������Ӧ
							      stRobot.emRobotTactic =TEST_TOPAIR;
							         
						         }
						     
				 	}



			
				else  if(g_ucKeyFlag==1)//������������
				 	{

						  if(JS_B1(stJSValue.usReserved)||KEY_N1(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //�ڴ˴����������Ӧ
							      stRobot.emRobotTactic = BLUE_FIELD_L2_RESTART1;
							      
							         
						         }
						  

						  if(JS_B2(stJSValue.usReserved)||KEY_N2(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //�ڴ˴����������Ӧ
							       stRobot.emRobotTactic =BLUE_FIELD_L2_RESTART2;
							      
							         
						         }



						  if(JS_B3(stJSValue.usReserved)||KEY_N3(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //�ڴ˴����������Ӧ
							       stRobot.emRobotTactic =BLUE_FIELD_L2_RESTART3;
							      
							         
						         }
						  


						  if(JS_B4(stJSValue.usReserved)||KEY_N4(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //�ڴ˴����������Ӧ
							      stRobot.emRobotTactic =BLUE_FIELD_L2_RESTART4;
							         
						         }

						   if(JS_LEFT(stJSValue.usReserved)||KEY_N0(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //�ڴ˴����������Ӧ
							      stRobot.emRobotTactic =BLUE_FIELD;
							         
						         }
						     if(JS_RIGHT(stJSValue.usReserved)||KEY_N5(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //�ڴ˴����������Ӧ
							      stRobot.emRobotTactic =BLUE_FIELD_S2_RESTART;
							         
						         }

							 if(JS_L1(stJSValue.usReserved)||KEY_N6(usKeyValue))
						         {
							        //�ڴ˵ȴ������ͷ�
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //�ڴ˴����������Ӧ
							      stRobot.emRobotTactic =BLUE_FIELD_A_RESTART;
							         
						         }
							   if(JS_R1(stJSValue.usReserved))
						         {
							        //�ڴ˵ȴ������ͷ�
							     
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //�ڴ˴����������Ӧ
							      stRobot.emRobotTactic =TEST_TOPAIR;
							         
						         }
						     
				 	}

				 

				  if(JS_SELECT(stJSValue.usReserved)||KEY_F6(usKeyValue))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(usKeyValue!= 0xffff)
						{
							usKeyValue= PSKEY;
						 }
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//�ڴ˴����������Ӧ
					       g_ucCurModeSel = g_ucModeCnt;
					       InitMode(g_ucCurModeSel);
	     
				         }
				 
				
			 
                         	}	 
                       
				break;
			case C_MODE_TEST_SENSOR_1:
/*---------------�������ݡ����ء��������ֱ�-----------------------------*/
                         if(stJSValue.usReserved != 0xffff)
			          {
				        //�ڴ���Ӷ�Ӧ�����Ĵ���
				         if(JS_L1(stJSValue.usReserved))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
						StartMEMSGryo();
				         }



					 if(JS_R1(stJSValue.usReserved))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
						StopMEMSGryo();
				         }
				         if(JS_UP(stJSValue.usReserved))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//�ڴ˴����������Ӧ
					         
					        if(IS_VALVE_CLOSE(TOP_CATCH_VALVE))
					        {
							OPEN_VALVE(TOP_CATCH_VALVE);
					        }
						else
						 {
						       CLOSE_VALVE(TOP_CATCH_VALVE);
						 }
				         }

					 

				         if(JS_DOWN(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//�ڴ˴����������Ӧ
						         
						        if(IS_VALVE_CLOSE(MID_CATCH_VALVE))
						        {
								OPEN_VALVE(MID_CATCH_VALVE);
						        }
							else
							 {
							       CLOSE_VALVE(MID_CATCH_VALVE);
							 }
					         }


					 if(JS_LEFT(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//�ڴ˴����������Ӧ
						         
						        if(IS_VALVE_CLOSE(FRONT_LEG_VALVE))
						        {
								OPEN_VALVE(FRONT_LEG_VALVE);
						        }
							else
							 {
							       CLOSE_VALVE(FRONT_LEG_VALVE);
							 }
					         }


					 if(JS_RIGHT(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//�ڴ˴����������Ӧ
						         
						        if(IS_VALVE_CLOSE(BACK_LEG_VALVE))
						        {
								OPEN_VALVE(BACK_LEG_VALVE);
						        }
							else
							 {
							       CLOSE_VALVE(BACK_LEG_VALVE);
							 }
					         }


					  if(JS_B1(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//�ڴ˴����������Ӧ
						         
						        if(IS_VALVE_CLOSE(ROT_VALVE))
						        {
								OPEN_VALVE(ROT_VALVE);
						        }
							else
							 {
							       CLOSE_VALVE(ROT_VALVE);
							 }
					         }


                          #if 0
					   if(JS_B2(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//�ڴ˴����������Ӧ
						         
						        if(IS_VALVE_CLOSE(ROT_SWITCH_VALVE))
						        {
								OPEN_VALVE(ROT_SWITCH_VALVE);
						        }
							else
							 {
							       CLOSE_VALVE(ROT_SWITCH_VALVE);
							 }
					         }

					   #endif



					    if(JS_B3(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//�ڴ˴����������Ӧ
						         
						        if(IS_VALVE_CLOSE(MID_LIFTUP_VALVE))
						        {
								OPEN_VALVE(MID_LIFTUP_VALVE);
						        }
							else
							 {
							       CLOSE_VALVE(MID_LIFTUP_VALVE);
							 }
					         }

					
                        
	/************************************������A��ͨ��*********************************************/

				           if(JS_B4(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						       
						        //�ڴ˴����������Ӧ
						   	SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
							SendByteByUART1(AC_A_BASKET_CMD);
					         }

	/************************************�������׶���Ƶ��*************************************************************/
					  if(JS_B2(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						     while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//�ڴ˴����������Ӧ
						        SaveOneWord32(stRobot.stPot.ssPosX, 0);
						  
					         }
					     
		/**********************************************************************************************************/
					
                         	}
				
				
				break;
			case C_MODE_TEST_CODER_2:
/*---------------��������---------------------------------------------------------*/

				break;
			case C_MODE_TEST_LINE_3:
/*---------------����Ѳ��---------------------------------------------------------*/
                        if(stJSValue.usReserved != 0xffff)
		          {
			        //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_UP(stJSValue.usReserved))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//�ڴ˴����������Ӧ
					         
					        CALIBRATE_WHITE_LINE(LINE_CHAN_0);
					        PutCur(1,0);
						 PrintChar("CalWhite        ");
				         }

			         //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_DOWN(stJSValue.usReserved))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//�ڴ˴����������Ӧ
					         
					        CALIBRATE_BLACK_LINE(LINE_CHAN_0);
					        PutCur(1,0);
						 PrintChar("CalBlack        ");
				         }

					  //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_RIGHT(stJSValue.usReserved))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//�ڴ˴����������Ӧ
					         
					        CALIBRATE_RED_LINE(LINE_CHAN_0);
					        PutCur(1,0);
						 PrintChar("CalRed        ");
				         }


			         //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_LEFT(stJSValue.usReserved))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//�ڴ˴����������Ӧ
					         
					        CALIBRATE_BLUE_LINE(LINE_CHAN_0);
					        PutCur(1,0);
						 PrintChar("CalBlue        ");
				         }


				 	  //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_B1(stJSValue.usReserved))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					        //�ڴ˴����������Ӧ
					         
					        USE_RED_FIELD_CFG(LINE_CHAN_0);
						  PutCur(1,0);
						 PrintChar("UseRed          ");
				         }
				  if(JS_B2(stJSValue.usReserved))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					        //�ڴ˴����������Ӧ
					         
					        USE_BLUE_FIELD_CFG(LINE_CHAN_0);
						  PutCur(1,0);
						 PrintChar("UseBlue         ");
				         }

				    if(JS_B3(stJSValue.usReserved))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					        //�ڴ˴����������Ӧ
					         
					        USE_BLACK_FIELD_CFG(LINE_CHAN_0);
						 PutCur(1,0);
						 PrintChar("UseBlack         ");
				         }

				   if(JS_B4(stJSValue.usReserved))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					        //�ڴ˴����������Ӧ
					         
					        CONFIG_TO_LINE(LINE_CHAN_0);
						 PutCur(1,0);
						 PrintChar("ConfigToLine     ");
				         }

			         if(JS_L1(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //�ڴ˴����������Ӧ
				        OPEN_LINE_TRACK(LINE_CHAN_0);
					 GET_SEND_ID_CMD(LINE_CHAN_0);
					 GET_RCV_ID_CMD(LINE_CHAN_0);
					 GET_CALIBRATE_CMD(LINE_CHAN_0);
					 GET_LIGHT_FORCE_CMD(LINE_CHAN_0);
					 GET_SPEED_CMD(LINE_CHAN_0);
					 GET_LIGHT_STATE_CMD(LINE_CHAN_0);
					 

				      
					 PutCur(1,0);
					 PrintChar("RetParas         ");
			         }
                        }

				break;
			case C_MODE_CAL_GYRO_4:
/*---------------�궨����---------------------------------------------------------*/
                        if(stJSValue.usReserved != 0xffff)
		          {
			        //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_UP(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				//�ڴ˴����������Ӧ
				         g_siDirAngle += 50;
			         }

			         //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_DOWN(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				//�ڴ˴����������Ӧ
				         g_siDirAngle -= 50;
			         }


				 //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_LEFT(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				//�ڴ˴����������Ӧ
				         g_siDirAngle -= 10;
			         }


                             //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_RIGHT(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				//�ڴ˴����������Ӧ
				         g_siDirAngle += 10;
			         }

			          //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_B1(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				//�ڴ˴����������Ӧ
				         g_siDirAngle += 1;
			         }

			        //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_B3(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				//�ڴ˴����������Ӧ
				         g_siDirAngle  -= 1 ;
			         }
					 
                        }

				break;
			case C_MODE_MANUAL_5:
/*---------------�ֶ�����---------------------------------------------------------*/
                
                       if(stJSValue.usReserved != 0xffff)
		          {
			        //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_UP(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //�ڴ˴����������Ӧ
				        stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
				        g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
					 SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathLow, g_fpActionM1PotFB + g_fpM1DeltaPos);
				       
			         }

				  //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_DOWN(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //�ڴ˴����������Ӧ
				        stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;//��ֱ�������
				        g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
					 SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathLow, g_fpActionM1PotFB -g_fpM1DeltaPos);
				       
			         }

			        //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_B3(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //�ڴ˴����������Ӧ
				        stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;//����ˮƽ���
				        g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
					 SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathLow, g_fpActionM2PotFB - g_fpM2DeltaPos);
				       
			         }

				 //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_B1(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //�ڴ˴����������Ӧ
				        stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
				        g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
					 SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathLow, g_fpActionM2PotFB + g_fpM2DeltaPos);
				       
			         }

				   //�ڴ���Ӷ�Ӧ�����Ĵ���
			         if(JS_LEFT(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //�ڴ˴����������Ӧ
				        if(IS_VALVE_CLOSE(TOP_CATCH_VALVE))
				        {
						OPEN_VALVE(TOP_CATCH_VALVE);
				        }
					 else
					 {
					 	CLOSE_VALVE(TOP_CATCH_VALVE);
					 }
				       
			         }

				   if(JS_RIGHT(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //�ڴ˴����������Ӧ
				        if(IS_VALVE_CLOSE(MID_CATCH_VALVE))
				        {
						OPEN_VALVE(MID_CATCH_VALVE);
				        }
					 else
					 {
					 	CLOSE_VALVE(MID_CATCH_VALVE);
					 }
				       
			         }


				    if(JS_B2(stJSValue.usReserved))
			         {
				        //�ڴ˵ȴ������ͷ�
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //�ڴ˴����������Ӧ
				        if(IS_VALVE_CLOSE(ROT_VALVE))
				        {
						OPEN_VALVE(ROT_VALVE);
				        }
					 else
					 {
					 	CLOSE_VALVE(ROT_VALVE);
					 }
				       
			         }
         
	/*********************************************************************************************************/


				break;
			case C_MODE_LOW_SPEED_6:
/*---------------�Զ�����ģʽ---------------------------------------------------*/
                    
				
				 if(stJSValue.usReserved != 0xffff)
		          	{
				        //�ڴ���Ӷ�Ӧ�����Ĵ���
				         if(JS_START(stJSValue.usReserved)&&(g_FirstRun==0))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }

						g_FirstRun=1;
					//�ڴ˴����������Ӧ
					         
					     switch(stRobot.emRobotTactic)
							{
								case  RED_FIELD:
									 stRobot.emRobotTask =  INIT;
									break;
								/*****״̬��δ��*************/	
								case RED_FIELD_A_RESTART:
									stRobot.emRobotTask=TASK_INSERT_3;
									break;
								case RED_FIELD_L2_RESTART1:
									 stRobot.emRobotTask =  TASK1;
									break;
								case RED_FIELD_L2_RESTART2:
									stRobot.emRobotTask =  TASK1 ;
									break;
								case RED_FIELD_L2_RESTART3:
									stRobot.emRobotTask = TASK1  ;
									break;
								case RED_FIELD_L2_RESTART4:
									break;
									
								case RED_FIELD_S2_RESTART:
									stRobot.emRobotTask = TASK_START_2 ;
									break;
								case BLUE_FIELD:
									stRobot.emRobotTask =INIT;
									break;


								/*****״̬��δ��*************/	
								case BLUE_FIELD_A_RESTART:
									break;
								case BLUE_FIELD_L2_RESTART1:
									 stRobot.emRobotTask =TASK1  ;
									break;
								case BLUE_FIELD_L2_RESTART2:
									 stRobot.emRobotTask =  TASK1 ;
									break;
								case BLUE_FIELD_L2_RESTART3:
									stRobot.emRobotTask =TASK1  ;
									break;
									
								case BLUE_FIELD_L2_RESTART4:
									stRobot.emRobotTask =  TASK1;
									break;
									
								case BLUE_FIELD_S2_RESTART:
									stRobot.emRobotTask =  TASK_START_2;
									break;

								case TEST_TOPAIR:
									stRobot.emRobotTask =  TASK_TOPAIR1;
									break;
								default:
									break;
							}
				         }
                     	}
				  if(JS_UP(stJSValue.usReserved))
				   {
					   //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					       g_fpValveCoff+=1;
			 	 }

				 if(JS_DOWN(stJSValue.usReserved))
					   {
						   //�ڴ˵ȴ������ͷ�
						        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							
						  g_fpValveCoff-=1;
				 	 }


/*************************************ѡ���Զ��ͷ��Զ�ģʽ*****************************************************/
				  if(stJSValue.usReserved != 0xffff)
			          	{
					        //�ڴ���Ӷ�Ӧ�����Ĵ���
					         if(JS_B1(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//�ڴ˴����������Ӧ
						         
						     g_ucAllContinueFlag =1;
					         }
	                     	}

				    if(stJSValue.usReserved != 0xffff)
			          	{
					        //�ڴ���Ӷ�Ӧ�����Ĵ���
					         if(JS_B3(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//�ڴ˴����������Ӧ
						         
						     g_ucAllContinueFlag =0;
					         }
	                     	}

					if(usKeyValue!= 0xffff)
			          	{
					        //�ڴ���Ӷ�Ӧ�����Ĵ���
					         if(KEY_F7(PSKEY))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(usKeyValue!= 0xffff)
						        {
							        usKeyValue = PSKEY;
						        }
						//�ڴ˴����������Ӧ
						
						    	g_ucAllContinueFlag =1;
							g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������

							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;
							stRobot.stPot.ssPosX=0;
							stRobot.stPot.ssPosY=0;
							stRobot.stPot.ssPosQ=0;
							
					         }
	                     	}
	

				break;
			case C_MODE_MID_SPEED_7:
/*---------------�Զ�����ģʽ---------------------------------------------------*/
			  if((stJSValue.usReserved != 0xffff)||(usKeyValue!= 0xffff))
		          	{
				        //�ڴ���Ӷ�Ӧ�����Ĵ���
				         if((JS_START(stJSValue.usReserved)||KEY_F6(usKeyValue))&&(g_FirstRun==0))
				         {
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
						 while(usKeyValue!= 0xffff)
						  {
							 usKeyValue= PSKEY;
						  }
						g_FirstRun=1;
					//�ڴ˴����������Ӧ
					         
					    switch(stRobot.emRobotTactic)
							{
								case  RED_FIELD:
									 stRobot.emRobotTask =  INIT;
									break;

								
								/*****״̬��δ��*************/	
								case RED_FIELD_A_RESTART:
									 stRobot.emRobotTask = TASK_INSERT_3 ;
									break;


								case RED_FIELD_L2_RESTART1:
									 stRobot.emRobotTask =  TASK1;
									break;
								case RED_FIELD_L2_RESTART2:
									 stRobot.emRobotTask = TASK1 ;
									break;
								case RED_FIELD_L2_RESTART3:
									 stRobot.emRobotTask = TASK1 ;
									break;
								case RED_FIELD_L2_RESTART4:
									break;
								case RED_FIELD_S2_RESTART:
									stRobot.emRobotTask = TASK_START_2 ;
									break;
								case BLUE_FIELD:
									stRobot.emRobotTask =INIT;
									break;
								
								/*****״̬��δ��*************/	
								case BLUE_FIELD_A_RESTART:
									stRobot.emRobotTask =  TASK_INSERT_3;
									break;


								case BLUE_FIELD_L2_RESTART1:
									 stRobot.emRobotTask =TASK1  ;
									break;
								case BLUE_FIELD_L2_RESTART2:
									 stRobot.emRobotTask =  TASK1;
									break;
								case BLUE_FIELD_L2_RESTART3:
									stRobot.emRobotTask =  TASK1;
									break;
								case BLUE_FIELD_L2_RESTART4:
									stRobot.emRobotTask =  TASK1;
									break;
									
								case BLUE_FIELD_S2_RESTART:
									stRobot.emRobotTask =  TASK_START_2;
									break;
								default:
									break;
							}
				         }
                     	}


/*******************************ѡ���Զ��ͷ��Զ�ģʽ***************/
				  if(stJSValue.usReserved != 0xffff)
			          	{
					        //�ڴ���Ӷ�Ӧ�����Ĵ���
					         if(JS_B1(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//�ڴ˴����������Ӧ
						         
						     g_ucAllContinueFlag =1;
					         }
							 
	                     	}
				  
					if(usKeyValue!= 0xffff)
			          	{
					        //�ڴ���Ӷ�Ӧ�����Ĵ���
					         if(KEY_F7(PSKEY))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(usKeyValue!= 0xffff)
						        {
							        usKeyValue = PSKEY;
						        }
						//�ڴ˴����������Ӧ
						
						    	g_ucAllContinueFlag =1;
							g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������

							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;
							stRobot.stPot.ssPosX=0;
							stRobot.stPot.ssPosY=0;
							stRobot.stPot.ssPosQ=0;
							
					         }
	                     	}

				    if(stJSValue.usReserved != 0xffff)
			          	{
					        //�ڴ���Ӷ�Ӧ�����Ĵ���
					         if(JS_B3(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//�ڴ˴����������Ӧ
						         
						     	g_ucAllContinueFlag =0;
							
					         }
	                     	}

				break;
			case C_MODE_TOP_SPEED_8:
/*---------------�Զ�����ģʽ---------------------------------------------------*/
  			if((stJSValue.usReserved != 0xffff)||(usKeyValue!= 0xffff))
		          	{
				        //�ڴ���Ӷ�Ӧ�����Ĵ���
				         if((JS_START(stJSValue.usReserved)||KEY_F6(usKeyValue))&&(g_FirstRun==0))
				         {
				         	g_FirstRun=1;
					        //�ڴ˵ȴ������ͷ�
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//�ڴ˴����������Ӧ

						switch(stRobot.emRobotTactic)
							{
								case  RED_FIELD:
									 stRobot.emRobotTask =  INIT;
									break;
								
								/*****״̬��δ��*************/	
								case RED_FIELD_A_RESTART:
									stRobot.emRobotTask =  TASK_INSERT_3;
									break;

								case RED_FIELD_L2_RESTART1:
									 stRobot.emRobotTask =  TASK1;
									break;
								case RED_FIELD_L2_RESTART2:
									 stRobot.emRobotTask =  TASK1;
									break;
								case RED_FIELD_L2_RESTART3:
									stRobot.emRobotTask =  TASK1;
									break;

								case RED_FIELD_L2_RESTART4:
									break;
	
								case RED_FIELD_S2_RESTART:
									stRobot.emRobotTask = TASK_START_2 ;
									break;

						/////////////////////////////////////////////////			
								case BLUE_FIELD:
									stRobot.emRobotTask =INIT;
									break;

								case BLUE_FIELD_A_RESTART:
									stRobot.emRobotTask = TASK_INSERT_3 ;
									break;
								case BLUE_FIELD_L2_RESTART1:
									 stRobot.emRobotTask =TASK1  ;
									break;
								case BLUE_FIELD_L2_RESTART2:
									 stRobot.emRobotTask =  TASK1 ;
									break;
								case BLUE_FIELD_L2_RESTART3:
									 stRobot.emRobotTask =  TASK1 ;
									break;

								case BLUE_FIELD_L2_RESTART4:
									stRobot.emRobotTask =  TASK1 ;
									break;
									
								case BLUE_FIELD_S2_RESTART:
									stRobot.emRobotTask =  TASK_START_2;
									break;
								default:
									break;
							}
					         
				         }
                     	}


/*******************************ѡ���Զ��ͷ��Զ�ģʽ***************/
				  if(stJSValue.usReserved != 0xffff)
			          	{
					        //�ڴ���Ӷ�Ӧ�����Ĵ���
					         if(JS_B1(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//�ڴ˴����������Ӧ
						         
						     g_ucAllContinueFlag =1;
					         }
	                     	}

				    if(stJSValue.usReserved != 0xffff)
			          	{
					        //�ڴ���Ӷ�Ӧ�����Ĵ���
					         if(JS_B3(stJSValue.usReserved))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//�ڴ˴����������Ӧ
						         
						     g_ucAllContinueFlag =0;
					         }
	                     	}

					if(usKeyValue!= 0xffff)
			          	{
					        //�ڴ���Ӷ�Ӧ�����Ĵ���
					         if(KEY_F7(PSKEY))
					         {
						        //�ڴ˵ȴ������ͷ�
						        while(usKeyValue!= 0xffff)
						        {
							        usKeyValue = PSKEY;
						        }
						//�ڴ˴����������Ӧ
						
						    	g_ucAllContinueFlag =1;
							g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������

							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;
							stRobot.stPot.ssPosX=0;
							stRobot.stPot.ssPosY=0;
							stRobot.stPot.ssPosQ=0;

							
					         }
	                     	}

/**************************************************************************************************/

				break;
			default:
				break;
           }
	

           }

		OSTimeDly(50);
	}
}
void CanTask (void *pdata)
{
	static UCHAR8 uc_LineCnt=0;
	CanTxMsg TxMessage = {0x55, 0x00, CAN_ID_STD, CAN_RTR_DATA, 3, 0,0,0,0,0,0,0,0};
	CanRxMsg RxMessage = {0xaa, 0x00, CAN_ID_STD, CAN_RTR_DATA, 3, 5,5,5,5,5,5,1,1};
	USE_BLACK_FIELD_CFG(LINE_CHAN_0);
	DECLARE_COMM_OBJS( VISIONDETECLINE_ID, stVisionDetecLine);
	while(1)
	{

             if(CAN1IsDataInTxBuf())//�����������ͷ���
              {
			  	CAN1BeginSend();
              }

	      if(CAN2IsDataInTxBuf())//�����������ͷ���
              {
			  	CAN2BeginSend();
              }
		  
		while(CAN2IsDataInRxBuf())//���CAN2 ���ܶ����������ݽ��д���,��ȫ���������
		{
			RxMessage = *CAN2GetRxBufDat();
			switch(RxMessage.StdId)
			{
				case CAN_SWITCH_ID:
					UPDATE_SWITCH(RxMessage);
					break;
				case CAN_LINE0_DATA_ID:
					UPDATE_LINE(RxMessage);
					//����Ѳ��ֵ
					       if((GET_LINE_VALUE(LINE_CHAN_0) >= g_ucLineMinValue)  &&( GET_LINE_VALUE(LINE_CHAN_0) <= g_ucLineMaxValue))
					       {
					       	
					       	//g_ucLineValue = GET_LINE_VALUE(LINE_CHAN_0);
					       	//�����޸�-------
					       	//if(abs(GET_LINE_VALUE(LINE_CHAN_0) -g_ucFliterLineValue[uc_LineCnt])<=65)
					       		{
									uc_LineCnt++;
									if(uc_LineCnt>=12)
										{
											uc_LineCnt=0;
										}
									g_ucFliterLineValue[uc_LineCnt]=GET_LINE_VALUE(LINE_CHAN_0);
									g_ucLineValue=(g_ucFliterLineValue[0]
										                 +g_ucFliterLineValue[1]
										                 +g_ucFliterLineValue[2]
										                 +g_ucFliterLineValue[3]
										                 +g_ucFliterLineValue[4]
										                 +g_ucFliterLineValue[5]
										                 +g_ucFliterLineValue[6]
										                 +g_ucFliterLineValue[7]
										                 +g_ucFliterLineValue[8]
										                 +g_ucFliterLineValue[9]
										                 +g_ucFliterLineValue[10]
										                 +g_ucFliterLineValue[11]
										                 -FindUCHAR8Max(g_ucFliterLineValue, 12)
										                 -FindUCHAR8Min(g_ucFliterLineValue, 12))/10.0;
							       }
					       }
					break;
				case CAN_LINE1_DATA_ID:
					UPDATE_LINE(RxMessage);
					break;
				case CAN_LINE2_DATA_ID:
					UPDATE_LINE(RxMessage);
					break;
				case CAN_LINE3_DATA_ID:
					UPDATE_LINE(RxMessage);
					break;
				case CAN_RADAR_RCV_ID:
					UPDATE_RADAR(RxMessage);
					break;
				default:
					;
					
			}
			
			
		}

              
				   		

		OSTimeDly(2);
	}
}








void LcdClear(void)
{
	PutCur(0,0);
	PrintChar("                ");
	
	PutCur(1,0);
	PrintChar("                ");

	PutCur(2,0);
	PrintChar("                ");

	PutCur(3,0);
	PrintChar("                ");
}

void ModeSelectPage(void)
{
	
	LcdClear();
	PutCur(0,0);
	switch(g_ucModeCnt)
           {
                     case C_MODE_INIT_0:
			PrintChar("Init            ");
				break;
			case C_MODE_TEST_SENSOR_1:
			PrintChar("TestSensor      ");
				break;
			case C_MODE_TEST_CODER_2:
			PrintChar("TestCoder       ");
				break;
			case C_MODE_TEST_LINE_3:
			PrintChar("TestLine");
				break;
			case C_MODE_CAL_GYRO_4:
                     PrintChar("CalGyroCoder ");
				break;
			case C_MODE_MANUAL_5:
                     PrintChar("Manual");
				break;
			case C_MODE_LOW_SPEED_6:
			PrintChar("LOWspeed");
				break;
			case C_MODE_MID_SPEED_7:
                     PrintChar("MIDspeed");
				break;
			case C_MODE_TOP_SPEED_8:
                     PrintChar("TOPspeed");
                         	break;
			default:
				break;
           }

	
	PutCur(0,12);
	PrintInt(PSKEY,4, HEX);
	PutCur(1,0);
	switch(stRobot.emRobotTactic)
	{
		case RED_FIELD:
			PrintChar("RED_FILED");
			break;
	       case RED_FIELD_L2_RESTART1:
		   	PrintChar("RED_L2_RESTART1");
		   	break;
		case RED_FIELD_L2_RESTART2:
		   	PrintChar("RED_L2_RESTART2");
		   	break;
		case RED_FIELD_L2_RESTART3:
		   	PrintChar("RED_L2_RESTART3");
		   	break;
		case RED_FIELD_L2_RESTART4:
		   	PrintChar("RED_L2_RESTART4");
		   	break;
		case RED_FIELD_A_RESTART:
		   	PrintChar("RED_FIELD_A_RESTART");
		   	break;
		case RED_FIELD_S2_RESTART:
		   	PrintChar("RED_S2_RESTART");
		   	break;
		case BLUE_FIELD:
			PrintChar("BLUE_FIELD");
			break;
		case BLUE_FIELD_L2_RESTART1:
			PrintChar("BLUE_L2_RESTART1");
		       break;
		case BLUE_FIELD_L2_RESTART2:
			PrintChar("BLUE_L2_RESTART2");
		       break;
		case BLUE_FIELD_L2_RESTART3:
			PrintChar("BLUE_L2_RESTART3");
		       break;
		case BLUE_FIELD_L2_RESTART4:
			PrintChar("BLUE_L2_RESTART4");
		       break;
		case BLUE_FIELD_A_RESTART:
			PrintChar("BLUE_FIELD_A_RESTART");
		       break;
		case BLUE_FIELD_S2_RESTART:
			PrintChar("BLUE_S2_RESTART");
		       break;
		case TEST_TOPAIR:
			PrintChar("TEST_TOPAIR");
		       break;
	       default:
		   	break;
	}

	PutCur(2,0);
	PrintChar("X   Y   Q      ");
	PutCur(2,1);
	PrintInt(stRobot.stPot.ssPosX,3,DEC);
	PutCur(2,5);
	PrintInt(stRobot.stPot.ssPosY, 3, DEC);
	PutCur(2,9);
	PrintInt(stRobot.stPot.ssPosQ,5,DEC);
	
	PutCur(3,0);
	PrintChar("L");
	PutCur(3,1);
	PrintInt(GET_LINE_VALUE(LINE_CHAN_0),3,DEC);
	PutCur(3,4);
	PrintChar("S");
	PutCur(3,5);
	PrintInt(GET_SWITCH_VALUE(),5,HEX);
	PutCur(3,11);
	PrintChar("C");
	PutCur(3,12);
	PrintInt(READ_CODER(stAction_M1.ucCoderChan),1,DEC);
	PutCur(3,13);
	PrintChar(",");
	PutCur(3,14);
	PrintInt(READ_CODER(stAction_M2.ucCoderChan),1,DEC);

	
}

void TestLinePage(void)
{
       PutCur(0,0);
	PrintChar("TestLine        ");
	PutCur(2,0);
	PrintChar("                ");
	PutCur(3,0);
	PrintChar("                ");
	

	PutCur(2,0);
	PrintChar("L:    ID:    ");
	PutCur(2,2);
	PrintInt(GET_LINE_VALUE(LINE_CHAN_0),3,DEC);
	PutCur(2,9);
	PrintInt(GET_RCV_ID_VALUE(LINE_CHAN_0),3,HEX);
	PrintChar(",");
	PrintInt(GET_SEND_ID_VALUE(LINE_CHAN_0),3,HEX);



	PutCur(3,0);
	PrintInt(GET_CALIBRATE_VALUE(LINE_CHAN_0),5,HEX);
	PrintChar(",");
	PrintInt(GET_SPEED_VALUE(LINE_CHAN_0),2,DEC);
	PrintChar(",");
	PrintInt(GET_LIGHT_FORCE_VALUE(LINE_CHAN_0),3,DEC);
       PrintChar(",");
	PrintInt(GET_LIGHT_STATE_VALUE(LINE_CHAN_0),5,HEX);;
	
	
}


void LineFilterPage(void)
{
      LcdClear();
      PutCur(0,0);
      PrintInt(g_ucFliterLineValue[0], 4 ,DEC);

      PutCur(0,4);
      PrintInt(g_ucFliterLineValue[1], 4 ,DEC);

      PutCur(0,8);
      PrintInt(g_ucFliterLineValue[2], 4 ,DEC);
	  
      PutCur(0,12);
      PrintInt(g_ucFliterLineValue[3], 4 ,DEC);


      PutCur(1,0);
      PrintInt(g_ucFliterLineValue[4], 4 ,DEC);

      PutCur(1,4);
      PrintInt(g_ucFliterLineValue[5], 4 ,DEC);

      PutCur(1,8);
      PrintInt(g_ucFliterLineValue[6], 4 ,DEC);
	  
      PutCur(1,12);
      PrintInt(g_ucFliterLineValue[7], 4 ,DEC);



      PutCur(2,0);
      PrintInt(g_ucFliterLineValue[8], 4 ,DEC);

      PutCur(2,4);
      PrintInt(g_ucFliterLineValue[9], 4 ,DEC);

      PutCur(2,8);
      PrintInt(g_uiCPUUsage,3,DEC);
	  
      PutCur(2,12);
      PrintInt(GET_LINE_VALUE(LINE_CHAN_0), 4 ,DEC);
}

void LowSpeedPage(void)
{
	LcdClear();
	PutCur(0,0);
	switch(stRobot.emRobotTask)
	{
		case INIT:
			PrintChar("INIT");
			break;
		case TASK1:
			PrintChar("TASK1");
		break;
		case TASK2:
			PrintChar("TASK2");
		break;
		case TASK3:
			PrintChar("TASK3");
		break;
		case TASK4:
			PrintChar("TASK4");
		break;
		case TASK5:
			PrintChar("TASK5");
			break;
		case TASK6:
			PrintChar("TASK6");
			break;
		case TASK7:
			PrintChar("TASK7");
			break;
		case TASK8:
			PrintChar("TASK8");
			break;
		case TASK9:
			PrintChar("TASK9");
			break;
		case TASK10:
			PrintChar("TASK10");
			break;
		case TASK11:
			PrintChar("TASK11");
			break;
		case TASK12:
			PrintChar("TASK12");
			break;
		case TASK13:
			PrintChar("TASK13");
			break;
		case TASK14:
			PrintChar("TASK14");
			break;
		case TASK15:
			PrintChar("TASK15");
			break;
		case TASK16:
			PrintChar("TASK16");
			break;	
		case TASK17:
			PrintChar("TASK17");
			break;
		case TASK18:
			PrintChar("TASK18");
			break;
		case TASK19:
			PrintChar("TASK19");
			break;
		case TASK20:
			PrintChar("TASK20");
			break;
		case TASK21:
			PrintChar("TASK21");
			break;
		case TASK22:
			PrintChar("TASK22");
			break;
		case TASK23:
			PrintChar("TASK23");
			break;
		case TASK24:
			PrintChar("TASK24");
			break;
		case TASK25:
			PrintChar("TASK25");
			break;
		case TASK26:
			PrintChar("TASK26");
			break;
		case TASK27:
			PrintChar("TASK27");
			break;
		case TASK28:
			PrintChar("TASK28");
			break;
		case TASK29:
			PrintChar("TASK29");
			break;
		case TASK30:
			PrintChar("TASK30");
			break;
		case TASK31:
			PrintChar("TASK31");
			break;
		case TASK_START_1:
			PrintChar("START_1");
			break;
		case TASK_START_2:
			PrintChar("START_2");
			break;
		case TASK_START_3:
			PrintChar("START_3");
			break;
		case TASK_INSERT_1:
			PrintChar("INSERT_1");
			break;
		case TASK_INSERT_2:
			PrintChar("INSERT_2");
			break;
		case TASK_INSERT_3:
			PrintChar("INSERT_3");
			break;
		case TASK_INSERT_4:
			PrintChar("INSERT_4");
			break;
		case TASK_INSERT_5:
			PrintChar("INSERT_5");
			break;
		case TASK_INSERT_6:
			PrintChar("INSERT_6");
			break;
		case TASK_INSERT_7:
			PrintChar("INSERT_7");
			break;
		case TASK_INSERT_8:
			PrintChar("INSERT_8");
			break;
		case TASK_TOPAIR1:
			PrintChar("TASK_TOPAIR1");
			break;
		case TASK_TOPAIR2:
			PrintChar("TASK_TOPAIR2");
			break;
		case TASK_TOPAIR3:
			PrintChar("TASK_TOPAIR3");
			break;
		case TASK_TOPAIR4:
			PrintChar("TASK_TOPAIR4");
			break;
		case TASK_TOPAIR5:
			PrintChar("TASK_TOPAIR5");
			break;
		case TASK_TOPAIR6:
			PrintChar("TASK_TOPAIR6");
			break;
		case TASK_TOPAIR7:
			PrintChar("TASK_TOPAIR7");
			break;
		
		default:
		      break;
		
	}

/*****************��ӡ�Զ��ͷ��Զ�ģʽ****************************************/

	PutCur(0,9);
	PrintInt(g_SelectState_Electric_On, 1, DEC);

	PutCur(0,11);
	PrintInt(g_ucFlashLoad, 1, DEC);

	PutCur(0,13);
	PrintInt(g_ucUseMemsGryo, 1, DEC);
	
	PutCur(0,15);

		if(g_ucAllContinueFlag==1)
			{
				PrintChar("A");
			}
		else
			{
				PrintChar("M");
			}
	
	
	
	PutCur(1,0);
	PrintChar("X");
	PutCur(1,1);
	PrintInt(stRobot.stPot.ssPosX,5,DEC);


	
	PutCur(1,6);
	PrintChar("L");
	PutCur(1,7);
	PrintInt(g_ucLineValue,3,DEC);

	
	PutCur(1,10);
	PrintChar("L");
	PutCur(1,11);
	PrintInt(GET_LINE_VALUE(LINE_CHAN_0),3,DEC);

	PutCur(2,0);
	PrintChar("Y");
	PutCur(2,1);
	PrintInt(stRobot.stPot.ssPosY,5,DEC);

	PutCur(2,6);
	PrintChar("UM");
	PutCur(2,8);
	PrintInt(stAction_M1.fpCoffPot*READ_CODER(stAction_M1.ucCoderChan),5,DEC);

	PutCur(3,0);
	PrintChar("Q");
	PutCur(3,1);
	PrintInt(stRobot.stPot.ssPosQ,5,DEC);
	

       
	PutCur(3,6);
	PrintChar("IR");
	PutCur(3,8);
	PrintInt(g_ucWatchComTemp,4,HEX);

	PutCur(3,12);
	PrintInt(g_fpVisionPotY,3,DEC);

	PutCur(2,13);
	PrintInt(g_ucVisionUsedCnt,3,DEC);
	
    
}

void CalPage(void)
{

       LcdClear();
	PutCur(0,0);
	PrintChar("Calibrate:");
	PutCur(0,10);
	PrintInt(g_siDirAngle,4,DEC);
	
	PutCur(1,0);
	PrintChar("X");
	PutCur(1,1);
	PrintInt(stRobot.stPot.ssPosX,5,DEC);

	PutCur(1,6);
	PrintChar("A:");
	PutCur(1,8);
	PrintInt(READ_CODER(OMNI_CODER_A),5,DEC);

	PutCur(2,0);
	PrintChar("Y");
	PutCur(2,1);
	PrintInt(stRobot.stPot.ssPosY,5,DEC);

	PutCur(2,6);
	PrintChar("B:");
	PutCur(2,8);
	PrintInt(READ_CODER(OMNI_CODER_B),5,DEC);

	PutCur(3,0);
	PrintChar("Q");
	PutCur(3,1);
	PrintInt(stRobot.stPot.ssPosQ,5,DEC);

	PutCur(3,6);
	PrintChar("Q:");
	PutCur(3,8);
	PrintInt(GRYO_Q,5,DEC);
	
}



void SensorPage(void)
{
       LcdClear();
	   
	PutCur(0,0);
	PrintChar("TestSensor");
	
	PutCur(1,0);
	PrintChar("Q");
	PutCur(1,1);
       PrintInt(COFF_OMNI_Q * GRYO_Q,6,DEC);
	PutCur(1,8);
	PrintChar("S");
	PutCur(1,9);
	PrintInt(GET_SWITCH_VALUE(),6,HEX);
	
	PutCur(2,0);
	PrintChar("A");
	PutCur(2,1);
       PrintInt(GET_VALVE_VALUE(),6,HEX);
	PutCur(2,8);
	PrintChar("J");
	PutCur(2,9);
	PrintInt(JOYSTICK_RESERVED,6,HEX);
	
	/*PutCur(2,0);
	PrintChar("AT");
	PutCur(2,4);
       PrintInt(g_uiConstTime,6,DEC);*/
	
	
       PutCur(3,0);
	PrintChar("L");
	PutCur(3,1);
       PrintInt(GET_LINE_VALUE(LINE_CHAN_0),6,DEC);


	PutCur(3,8);
	PrintChar("MQ");
	PutCur(3,10);
       PrintInt(g_ssMemsGryo,6,DEC);
	
	
	
}

void CoderPage1(void)
{
	LcdClear();
	
	PutCur(0,0);
	PrintChar("TestCoder1");
	
	
	PutCur(1,0);
	PrintChar("1:      2:      ");
	PutCur(1,2);
	PrintInt(READ_CODER(1),6,DEC);
	PutCur(1,10);
	PrintInt(READ_CODER(2),6,DEC);
	
	PutCur(2,0);
	PrintChar("3:      4:      ");
	PutCur(2,2);
	PrintInt(READ_CODER(3),6,DEC);
	PutCur(2,10);
	PrintInt(READ_CODER(4),6,DEC);

	PutCur(3,0);
	PrintChar("5:      6:      ");
	PutCur(3,2);
	PrintInt(READ_CODER(5),6,DEC);
	PutCur(3,10);
	PrintInt(READ_CODER(6),6,DEC);

	
}


void CoderPage2(void)
{
	LcdClear();
	
	PutCur(0,0);
	PrintChar("TestCoder2");
	
	
	PutCur(1,0);
	PrintChar("7:      8:      ");
	PutCur(1,2);
	PrintInt(READ_CODER(7),6,DEC);
	PutCur(1,10);
	PrintInt(READ_CODER(8),6,DEC);
	
	PutCur(2,0);
	PrintChar("9:      a:      ");
	PutCur(2,2);
	PrintInt(READ_CODER(9),6,DEC);
	PutCur(2,10);
	PrintInt(READ_CODER(10),6,DEC);

	PutCur(3,0);
	PrintChar("b:      c:      ");
	PutCur(3,2);
	PrintInt(READ_CODER(11),6,DEC);
	PutCur(3,10);
	PrintInt(READ_CODER(12),6,DEC);

	
}



void MotorCoderPage3(void)
{
	LcdClear();
	
	PutCur(0,0);
	PrintChar("MotorCoder3");
	
	
	PutCur(1,0);
	PrintChar("1:      2:      ");
	PutCur(1,2);
	PrintInt(READ_CODER(stAction_M1.ucCoderChan),6,DEC);
	PutCur(1,10);
	PrintInt(READ_CODER(stAction_M2.ucCoderChan),6,DEC);
	
	PutCur(2,0);
	PrintChar("a:      b:      ");
	PutCur(2,2);
	PrintInt(READ_CODER(stRobot.stBaseWaMotor.ucCoderChan),6,DEC);
	PutCur(2,10);
	PrintInt(READ_CODER(stRobot.stBaseWbMotor.ucCoderChan),6,DEC);

	PutCur(3,0);
	PrintChar("c:      L:      ");
	PutCur(3,2);
	PrintInt(READ_CODER(stRobot.stBaseWcMotor.ucCoderChan),6,DEC);
	PutCur(3,10);
	PrintInt(GET_LINE_VALUE(LINE_CHAN_0),6,DEC);

	
}

void MotorPwmPage(void)
{
	LcdClear();
	
	PutCur(0,0);
	PrintChar("MotorPwm");
	
	
	PutCur(1,0);
	PrintChar("1:      2:      ");
	PutCur(1,2);
	PrintInt(stAction_M1.ssPwmDuty,6,DEC);
	PutCur(1,10);
	PrintInt(stAction_M2.ssPwmDuty,6,DEC);
	
	PutCur(2,0);
	PrintChar("a:      b:      ");
	PutCur(2,2);
	PrintInt(stRobot.stBaseWaMotor.ssPwmDuty,6,DEC);
	PutCur(2,10);
	PrintInt(stRobot.stBaseWbMotor.ssPwmDuty,6,DEC);

	PutCur(3,0);
	PrintChar("c:      L:      ");
	PutCur(3,2);
	PrintInt(stRobot.stBaseWcMotor.ssPwmDuty,6,DEC);
	PutCur(3,10);
	PrintInt(GET_LINE_VALUE(LINE_CHAN_0),6,DEC);

}


void MotorVeltDesPage(void)
{
	LcdClear();
	
	PutCur(0,0);
	PrintChar("MotorVeltDes");
	
	
	PutCur(1,0);
	PrintChar("1:      2:      ");
	PutCur(1,2);
	PrintInt(stAction_M1.fpVeltDes,6,DEC);
	PutCur(1,10);
	PrintInt(stAction_M2.fpVeltDes,6,DEC);
	
	PutCur(2,0);
	PrintChar("a:      b:      ");
	PutCur(2,2);
	PrintInt(stRobot.stBaseWaMotor.fpVeltDes,6,DEC);
	PutCur(2,10);
	PrintInt(stRobot.stBaseWbMotor.fpVeltDes,6,DEC);

	PutCur(3,0);
	PrintChar("c:      L:      ");
	PutCur(3,2);
	PrintInt(stRobot.stBaseWcMotor.fpVeltDes,6,DEC);
	PutCur(3,10);
	PrintInt(GET_LINE_VALUE(LINE_CHAN_0),6,DEC);

}

void ManualPage(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("Manual");

	PutCur(1,0);
	PrintChar("X    Y    Q    ");
	PutCur(1,1);
	PrintInt(stRobot.stVeltDes.fpVx, 4,DEC);

	PutCur(1,6);
	PrintInt(stRobot.stVeltDes.fpVy, 4,DEC);


	PutCur(1,11);
	PrintInt(stRobot.stVeltDes.fpW, 4,DEC);


	PutCur(2,0);
	PrintChar("M1:     M2:     ");
	PutCur(2,3);
	PrintInt(stAction_M1.fpCoffPot*READ_CODER(stAction_M1.ucCoderChan),5,DEC);
	PutCur(2,11);
	PrintInt(stAction_M2.fpCoffPot*READ_CODER(stAction_M2.ucCoderChan),5,DEC);

       PutCur(3,0);
	PrintChar("S     A   L    ");
	PutCur(3,1);
	PrintInt(GET_SWITCH_VALUE(),5,HEX);

	PutCur(3,7);
	PrintInt(GET_VALVE_VALUE(),3,HEX);

	PutCur(3,11);
	PrintInt(GET_LINE_VALUE(LINE_CHAN_0),3,DEC);


}

void TestVisionPage(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("TestVision");

	PutCur(1,0);
	PrintChar("Num");
	PutCur(1,4);
	PrintInt(stVisionDetecLine.NumLines, 4,DEC);

	PutCur(1,9);
	PrintChar("R1:");
	PutCur(1,12);
	PrintInt(stVisionDetecLine.Rho1, 5,DEC);

	PutCur(2,0);
	PrintChar("R2:");
	PutCur(2,3);
	PrintInt(stVisionDetecLine.Rho2, 5,DEC);

	
	PutCur(2,9);
	PrintChar("D1:");
	PutCur(2,12);
	PrintInt(stVisionDetecLine.Dis1, 5,DEC);

	PutCur(3,0);
	PrintChar("D2:");
	PutCur(3,3);
	PrintInt(stVisionDetecLine.Dis2, 5,DEC);


	PutCur(3,6);
	PrintInt(RetVisionLineValue(&stVisionDetecLine),5,DEC );

	PutCur(3,10);
	PrintInt(g_uiVisionUpdateTime/1000000,5,DEC );
	

	

}

void RedFlashPage1(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("RedFlashPage1");

	PutCur(1,0);
	PrintChar("CatchBanRes1");

	PutCur(2,0);
	PrintChar("X:");
	PutCur(2,1);
	PrintInt(GetOneShort16(OFST_RED_CATCHBANPATH_RES1_X), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Y:");
	PutCur(2,9);
	PrintInt(GetOneShort16(OFST_RED_CATCHBANPATH_RES1_Y), 5,DEC);

	PutCur(3,0);
	PrintChar("Q:");
	PutCur(3,1);
	PrintInt(GetOneShort16(OFST_RED_CATCHBANPATH_RES1_Q), 5,DEC);


}

void RedFlashPage2(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("RedFlashPage2");

	PutCur(1,0);
	PrintChar("CatchBanRes2");

	PutCur(2,0);
	PrintChar("X:");
	PutCur(2,1);
	PrintInt(GetOneShort16(OFST_RED_CATCHBANPATH_RES2_X), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Y:");
	PutCur(2,9);
	PrintInt(GetOneShort16(OFST_RED_CATCHBANPATH_RES2_Y), 5,DEC);

	PutCur(3,0);
	PrintChar("Q:");
	PutCur(3,1);
	PrintInt(GetOneShort16(OFST_RED_CATCHBANPATH_RES2_Q), 5,DEC);


}

void RedFlashPage3(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("RedFlashPage3");

	PutCur(1,0);
	PrintChar("CatchBanRes3");

	PutCur(2,0);
	PrintChar("X:");
	PutCur(2,1);
	PrintInt(GetOneShort16(OFST_RED_CATCHBANPATH_RES3_X), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Y:");
	PutCur(2,9);
	PrintInt(GetOneShort16(OFST_RED_CATCHBANPATH_RES3_Y), 5,DEC);

	PutCur(3,0);
	PrintChar("Q:");
	PutCur(3,1);
	PrintInt(GetOneShort16(OFST_RED_CATCHBANPATH_RES3_Q), 5,DEC);
}

void RedFlashPage4(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("RedFlashPage4");

	PutCur(1,0);
	PrintChar("ReleBanRes1");

	PutCur(2,0);
	PrintChar("X:");
	PutCur(2,1);
	PrintInt(GetOneShort16(OFST_RED_RELEASEBANPATH_RES1_X), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Y:");
	PutCur(2,9);
	PrintInt(GetOneShort16(OFST_RED_RELEASEBANPATH_RES1_Y), 5,DEC);

	PutCur(3,0);
	PrintChar("Q:");
	PutCur(3,1);
	PrintInt(GetOneShort16(OFST_RED_RELEASEBANPATH_RES1_Q), 5,DEC);


}

void RedFlashPage5(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("RedFlashPage5");

	PutCur(1,0);
	PrintChar("ReleBanRes2");

	PutCur(2,0);
	PrintChar("X:");
	PutCur(2,1);
	PrintInt(GetOneShort16(OFST_RED_RELEASEBANPATH_RES2_X), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Y:");
	PutCur(2,9);
	PrintInt(GetOneShort16(OFST_RED_RELEASEBANPATH_RES2_Y), 5,DEC);

	PutCur(3,0);
	PrintChar("Q:");
	PutCur(3,1);
	PrintInt(GetOneShort16(OFST_RED_RELEASEBANPATH_RES2_Q), 5,DEC);


}

void RedFlashPage6(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("RedFlashPage6");

	PutCur(1,0);
	PrintChar("ReleBanRes3");

	PutCur(2,0);
	PrintChar("X:");
	PutCur(2,1);
	PrintInt(GetOneShort16(OFST_RED_RELEASEBANPATH_RES3_X), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Y:");
	PutCur(2,9);
	PrintInt(GetOneShort16(OFST_RED_RELEASEBANPATH_RES3_Y), 5,DEC);

	PutCur(3,0);
	PrintChar("Q:");
	PutCur(3,1);
	PrintInt(GetOneShort16(OFST_RED_RELEASEBANPATH_RES3_Q), 5,DEC);
}

void BlueFlashPage1(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("BlueFlashPage1");

	PutCur(1,0);
	PrintChar("CatchBanRes1");

	PutCur(2,0);
	PrintChar("X:");
	PutCur(2,1);
	PrintInt(GetOneShort16(OFST_BLUE_CATCHBANPATH_RES1_X), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Y:");
	PutCur(2,9);
	PrintInt(GetOneShort16(OFST_BLUE_CATCHBANPATH_RES1_Y), 5,DEC);

	PutCur(3,0);
	PrintChar("Q:");
	PutCur(3,1);
	PrintInt(GetOneShort16(OFST_BLUE_CATCHBANPATH_RES1_Q), 5,DEC);

	
}

void BlueFlashPage2(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("BlueFlashPage2");

	PutCur(1,0);
	PrintChar("CatchBanRes2");

	PutCur(2,0);
	PrintChar("X:");
	PutCur(2,1);
	PrintInt(GetOneShort16(OFST_BLUE_CATCHBANPATH_RES2_X), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Y:");
	PutCur(2,9);
	PrintInt(GetOneShort16(OFST_BLUE_CATCHBANPATH_RES2_Y), 5,DEC);

	PutCur(3,0);
	PrintChar("Q:");
	PutCur(3,1);
	PrintInt(GetOneShort16(OFST_BLUE_CATCHBANPATH_RES2_Q), 5,DEC);

	
}




void BlueFlashPage3(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("BlueFlashPage3");

	PutCur(1,0);
	PrintChar("CatchBanRes3");

	PutCur(2,0);
	PrintChar("X:");
	PutCur(2,1);
	PrintInt(GetOneShort16(OFST_BLUE_CATCHBANPATH_RES3_X), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Y:");
	PutCur(2,9);
	PrintInt(GetOneShort16(OFST_BLUE_CATCHBANPATH_RES3_Y), 5,DEC);

	PutCur(3,0);
	PrintChar("Q:");
	PutCur(3,1);
	PrintInt(GetOneShort16(OFST_BLUE_CATCHBANPATH_RES3_Q), 5,DEC);

	
}

void BlueFlashPage4(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("BlueFlashPage4");

	PutCur(1,0);
	PrintChar("ReleBanRes1");

	PutCur(2,0);
	PrintChar("X:");
	PutCur(2,1);
	PrintInt(GetOneShort16(OFST_BLUE_RELEASEBANPATH_RES1_X), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Y:");
	PutCur(2,9);
	PrintInt(GetOneShort16(OFST_BLUE_RELEASEBANPATH_RES1_Y), 5,DEC);

	PutCur(3,0);
	PrintChar("Q:");
	PutCur(3,1);
	PrintInt(GetOneShort16(OFST_BLUE_RELEASEBANPATH_RES1_Q), 5,DEC);

	
}

void BlueFlashPage5(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("BlueFlashPage5");

	PutCur(1,0);
	PrintChar("ReleBanRes2");

	PutCur(2,0);
	PrintChar("X:");
	PutCur(2,1);
	PrintInt(GetOneShort16(OFST_BLUE_RELEASEBANPATH_RES2_X), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Y:");
	PutCur(2,9);
	PrintInt(GetOneShort16(OFST_BLUE_RELEASEBANPATH_RES2_Y), 5,DEC);

	PutCur(3,0);
	PrintChar("Q:");
	PutCur(3,1);
	PrintInt(GetOneShort16(OFST_BLUE_RELEASEBANPATH_RES2_Q), 5,DEC);

	
}




void BlueFlashPage6(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("BlueFlashPage6");

	PutCur(1,0);
	PrintChar("ReleBanRes3");

	PutCur(2,0);
	PrintChar("X:");
	PutCur(2,1);
	PrintInt(GetOneShort16(OFST_BLUE_RELEASEBANPATH_RES3_X), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Y:");
	PutCur(2,9);
	PrintInt(GetOneShort16(OFST_BLUE_RELEASEBANPATH_RES3_Y), 5,DEC);

	PutCur(3,0);
	PrintChar("Q:");
	PutCur(3,1);
	PrintInt(GetOneShort16(OFST_BLUE_RELEASEBANPATH_RES3_Q), 5,DEC);

	
}

void HorMotorBluePage(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("HorMotorBluePage");

	PutCur(1,0);
	PrintChar("CatchMiddleBan");

	PutCur(2,0);
	PrintChar("Res1:");
	PutCur(2,1);
	PrintInt(GetOneInt32(OFST_BLUE_CATCHMIDDLEBAN_RES1), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Res2:");
	PutCur(2,9);
	PrintInt(GetOneInt32(OFST_BLUE_CATCHMIDDLEBAN_RES2), 5,DEC);

	PutCur(3,0);
	PrintChar("Res3:");
	PutCur(3,1);
	PrintInt(GetOneInt32(OFST_BLUE_CATCHMIDDLEBAN_RES3), 5,DEC);

	
}

void HorMotorRedPage(void)
{
	LcdClear();
	/*
       PutCur(0,0);
	PrintChar("HorMotorRedPage");

	PutCur(1,0);
	PrintChar("CatchMiddleBan");

	PutCur(2,0);
	PrintChar("Res1:");
	PutCur(2,1);
	PrintInt(GetOneInt32(OFST_RED_CATCHMIDDLEBAN_RES1), 5,DEC);
	
	PutCur(2,8);
	PrintChar("Res2:");
	PutCur(2,9);
	PrintInt(GetOneInt32(OFST_RED_CATCHMIDDLEBAN_RES2), 5,DEC);

	PutCur(3,0);
	PrintChar("Res3:");
	PutCur(3,1);
	PrintInt(GetOneInt32(OFST_RED_CATCHMIDDLEBAN_RES3), 5,DEC);
	*/
	PutCur(0,0);
	PrintChar("ROTVALVE");
	PutCur(1,0);
	PrintInt(g_TimeTemp, 10,DEC);
	
	PutCur(2,0);
	PrintInt(g_fpValveCoff, 10,DEC);

	
}

void OtherFlashPage(void)
{
	LcdClear();
       PutCur(0,0);
	PrintChar("OtherFlashPage");

	/*PutCur(1,0);
	PrintChar("Coff_Q:");
	PutCur(1,7);
	PrintInt(GetOneShort16(OFST_COFF_OMNI_Q_ADDR), 5,DEC);*/
	PutCur(1,0);
	PrintChar("V:");
	PutCur(1,3);
	PrintInt(stAction_M3.fpVeltFB,4,DEC);
	PutCur(1,9);
	PrintChar("V:");
	PutCur(1,11);
	PrintInt(stAction_M3.fpVeltDes,4,DEC);
	
	PutCur(2,0);
	PrintChar("Vy:");
	PutCur(2,10);
	PrintInt(ReadRobotFpVy(&stRobot), 5,DEC);

	PutCur(3,0);
	PrintChar("Pwm");
	PutCur(3,10);
	PrintInt(stAction_M3.ssPwmDuty,5,DEC);
	
	
}
/****************************************************************************************************
�������ƣ�LcdDispTask()
�����ܣ�Һ��ˢ����ʾ����
****************************************************************************************************/
void LcdDispTask(void *pdata)
{
	InitLcd();
	while(1)
	{  
	       g_uiCPUUsage = 0;
		if(g_uiCPUUsage > g_uiCPUUsageMax)
		{
		     g_uiCPUUsageMax = g_uiCPUUsage;
		}
	
		#if 0
		PutCur(2,0);
		PrintInt(stRobot.stPot.ssPosQ,5,DEC);
		PutCur(1,0);
		PrintInt(GRYO_Q,5,DEC);

		PutCur(3,0);
		PrintInt(GRYO_Q,5,DEC);
		PutCur(3,8);
		PrintInt(GRYO_Q,5,DEC);

		PutCur(2,8);
		PrintChar("aaaa");
		#endif
	
	
	
	#if 1
		switch(g_ucCurLcdPage)
		{
			case 0:
				ModeSelectPage();
				break;
			case 1:
				SensorPage();
				break;
			case 2:
				CoderPage1();
				break;
			case 3:
				TestLinePage();
				break;			
			case 4:
				CalPage();
				break;
				
			case 5:
                            ManualPage();
				break;
			case 6:
				LowSpeedPage();
				break;
			case 7:
				LowSpeedPage();
				break;
			case 8:
				LowSpeedPage();
				break;
				
			case 9:
				CoderPage2();
				break;
			case 10:
				MotorCoderPage3();
				break;
			case 11:
				MotorPwmPage();
				break;
			case 12:
				MotorVeltDesPage();
				break;
			case 13:
				LineFilterPage();
				break;
			case 14:
				TestVisionPage();
				break;
			case 15:
				RedFlashPage1();
				break;
			case 16:
				RedFlashPage4();
				break;
			case 17:
				RedFlashPage2();
				break;
			case 18:
				RedFlashPage5();
				break;
			case 19:
				RedFlashPage3();
				break;
			case 20:
				RedFlashPage6();
				break;
			case 21:
				BlueFlashPage1();
				break;
			case 22:
				BlueFlashPage4();
				break;
			case 23:
				BlueFlashPage2();
				break;
			case 24:
				BlueFlashPage5();
				break;
			case 25:
				BlueFlashPage3();
				break;
			case 26:
				BlueFlashPage6();
				break;
			case 27:
				OtherFlashPage();
				break;
			case 28:
				HorMotorRedPage();
				break;
			case 29:
				HorMotorBluePage();
				break;
			default:
				break;
		}

	
		#endif
		ARM_LED_FLASH();
		OSTimeDly(100);
	}
}



void DetectTask(void *pdata)
{
	UCHAR8 ucCnt_i=0;
	for(ucCnt_i=0;ucCnt_i<100;ucCnt_i++)
		{
			g_siFlashValue[ucCnt_i]=*(__IO SINT32*)(ADDR_FLASH_SECTOR_11+(ucCnt_i<<2));
		}

	g_ucKeyFlag=g_siFlashValue[OFST_RED_OR_BLUE];
	
	if(g_ucKeyFlag==0)
		{
			 stRobot.emRobotTactic=RED_FIELD;
		}
	else
		{
			 stRobot.emRobotTactic=BLUE_FIELD;
		}
	
	while(1)
	{
		/*����˼·Ϊ������ֵ����һ��ʱ�䣬�����޳�ʱ�����Զ�ֹͣ*/
		
			/**********************************�����˵��Զ���λ************************/
			switch(stRobot.emRobotReposit)
				{
					case REPOSIT_INIT:
						stJSValue.usReserved = JOYSTICK_RESERVED;
						stJSValue.usJsState = JOYSTICK_STATE;
						if((stJSValue.usJsState == 0x73)&&JS_R2(stJSValue.usReserved)&&JS_R1(stJSValue.usReserved)&&JS_L2(stJSValue.usReserved)&&JS_L1(stJSValue.usReserved))
							{
								while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
									
							        }
								stRobot.emRobotReposit=REPOSIT1;
								stRobot.emRobotTask = NO_ACTION;
							}
						break;
					case REPOSIT1:
						stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
						SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,20*4.6622f);	
						stRobot.emRobotReposit=REPOSIT2;
						break;
					case REPOSIT2:
						if(stAction_M2.emPathState==PATH_END)
							{
								CLOSE_VALVE(ROT_VALVE);
								CLOSE_VALVE(TOP_CATCH_VALVE);
								CLOSE_VALVE(MID_CATCH_VALVE);
								CLOSE_VALVE(MID_LIFTUP_VALVE);
								CLOSE_VALVE(FRONT_LEG_VALVE);
								CLOSE_VALVE(BACK_LEG_VALVE);
								g_uiTaskTime2=GetCurTime();
								stRobot.emRobotReposit=REPOSIT3;
							}
						break;
					case REPOSIT3:
						if((GetCurTime()-g_uiTaskTime2)>800000)
							{
								OPEN_VALVE(ROT_VALVE);
							}
						if((GetCurTime()-g_uiTaskTime2)>950000)
							{
								stRobot.emRobotReposit=REPOSIT4;
							}
						break;
					case REPOSIT4:
				
						CLOSE_VALVE(ROT_VALVE);
						stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
						SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,15*3.4043f);//�˸߶�Ҳ��ȡ��ͷ�ĸ߶�
						stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						stRobot.emRobotReposit=REPOSIT5;
						
						break;
					
					case REPOSIT5:
						default:
							break;
							
					}

			
			if((g_FirstRun==1)&&(g_SelectState_Electric_On==1))
				{

					stRobot.emNavState =  NAV_NULL;
					stRobot.emBaseState=BASE_BREAK;

					 if(KEY_N8(usKeyValue))//�Ƿ�������ݼ�
					         {
						        //�ڴ˵ȴ������ͷ�
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//�ڴ˴����������Ӧ
							
						 	g_ucUseMemsGryo=1;
							g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������	
							stRobot.stPot.ssPosQ=0;
							OPEN_VALVE(TOP_CATCH_VALVE);

					         }
					 //��������
					 if(KEY_N0(usKeyValue))
					         {
						        //�ڴ˵ȴ������ͷ�
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//�ڴ˴����������Ӧ
						    	g_SelectState_Electric_On=0;

							if(g_ucKeyFlag==0)
								{
							 		stRobot.emRobotTactic =RED_FIELD;
								}

							else 
								{
							 		stRobot.emRobotTactic =BLUE_FIELD;
								}
							 
							stRobot.emBaseState = BASE_BREAK;
							stRobot.emNavState = NAV_OFF;
							stRobot.pstCurPath=NULL;
							g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������

							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;
							stRobot.stPot.ssPosX=0;
							stRobot.stPot.ssPosY=0;
							stRobot.stPot.ssPosQ=0;

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
							g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;

							 InitMode(g_ucCurModeSel);
							
							g_ucAllContinueFlag =0;
							stRobot.emRobotTask = INIT;
							
					         }

				//��������1
					 if(KEY_N1(usKeyValue))
					         {
						        //�ڴ˵ȴ������ͷ�
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//�ڴ˴����������Ӧ
						    	g_SelectState_Electric_On=0;

							if(g_ucKeyFlag==0)
								{
							 		stRobot.emRobotTactic =RED_FIELD_L2_RESTART1;
								}

							else 
								{
							 		stRobot.emRobotTactic =BLUE_FIELD_L2_RESTART1;
								}
							 
							stRobot.emBaseState = BASE_BREAK;
							stRobot.emNavState = NAV_OFF;
							stRobot.pstCurPath=NULL;
							g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������

							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;
							stRobot.stPot.ssPosX=0;
							stRobot.stPot.ssPosY=0;
							stRobot.stPot.ssPosQ=0;

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
							g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;

							 InitMode(g_ucCurModeSel);
							
							g_ucAllContinueFlag =0;
							stRobot.emRobotTask = TASK1;
							
					         }
				//��������2

					  if(KEY_N2(usKeyValue))
					         {
						        //�ڴ˵ȴ������ͷ�
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//�ڴ˴����������Ӧ
						    	g_SelectState_Electric_On=0;

							if(g_ucKeyFlag==0)
								{
							 		stRobot.emRobotTactic =RED_FIELD_L2_RESTART2;
								}

							else 
								{
							 		stRobot.emRobotTactic =BLUE_FIELD_L2_RESTART2;
								}

							
							stRobot.emBaseState = BASE_BREAK;
							stRobot.emNavState = NAV_OFF;
							stRobot.pstCurPath=NULL;
							g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������

							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;
							stRobot.stPot.ssPosX=0;
							stRobot.stPot.ssPosY=0;
							stRobot.stPot.ssPosQ=0;


							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
							g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;

							 InitMode(g_ucCurModeSel);
							
							g_ucAllContinueFlag =0;
							stRobot.emRobotTask = TASK1;
					         }
				//��������3
					  
					  if(KEY_N3(usKeyValue))
					         {
						        //�ڴ˵ȴ������ͷ�
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//�ڴ˴����������Ӧ
						    	g_SelectState_Electric_On=0;

							if(g_ucKeyFlag==0)
								{
							 		stRobot.emRobotTactic =RED_FIELD_L2_RESTART3;
								}

							else 
								{
							 		stRobot.emRobotTactic =BLUE_FIELD_L2_RESTART3;
								}
							
							stRobot.emBaseState = BASE_BREAK;
							stRobot.emNavState = NAV_OFF;
							stRobot.pstCurPath=NULL;
							g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������

							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;
							stRobot.stPot.ssPosX=0;
							stRobot.stPot.ssPosY=0;
							stRobot.stPot.ssPosQ=0;

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
							g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;

							 InitMode(g_ucCurModeSel);
							
							g_ucAllContinueFlag =0;
							stRobot.emRobotTask = TASK1;
					         }
				//��������4
					  if(KEY_N4(usKeyValue))
					         {
						        //�ڴ˵ȴ������ͷ�
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//�ڴ˴����������Ӧ
						    	g_SelectState_Electric_On=0;
							if(g_ucKeyFlag==0)
								{
							 		stRobot.emRobotTactic =RED_FIELD_L2_RESTART4;
								}

							else 
								{
							 		stRobot.emRobotTactic =BLUE_FIELD_L2_RESTART4;
								}
							stRobot.emBaseState = BASE_BREAK;
							stRobot.emNavState = NAV_OFF;
							stRobot.pstCurPath=NULL;
							g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������

							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;
							stRobot.stPot.ssPosX=0;
							stRobot.stPot.ssPosY=0;
							stRobot.stPot.ssPosQ=0;

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
							g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;

							//stRobot.emRobotTask = TASK1;
					         }
				//��������
					   if(KEY_N5(usKeyValue))
					         {
						        //�ڴ˵ȴ������ͷ�
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//�ڴ˴����������Ӧ
						    	g_SelectState_Electric_On=0;

							if(g_ucKeyFlag==0)
								{
							 		stRobot.emRobotTactic =RED_FIELD_S2_RESTART;
								}

							else 
								{
							 		stRobot.emRobotTactic =BLUE_FIELD_S2_RESTART;
								}
							stRobot.emBaseState = BASE_BREAK;
							stRobot.emNavState = NAV_OFF;
							stRobot.pstCurPath=NULL;
							g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������

							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;
							stRobot.stPot.ssPosX=0;
							stRobot.stPot.ssPosY=0;
							stRobot.stPot.ssPosQ=0;

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
							g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;

							 InitMode(g_ucCurModeSel);
							
							g_ucAllContinueFlag =0;
							stRobot.emRobotTask =  TASK_START_2;
					         }

				//��������
					     if(KEY_N6(usKeyValue))
					         {
						        //�ڴ˵ȴ������ͷ�
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//�ڴ˴����������Ӧ
						    	g_SelectState_Electric_On=0;
						if(g_ucKeyFlag==0)
								{
							 		stRobot.emRobotTactic =RED_FIELD_A_RESTART;
								}

							else 
								{
							 		stRobot.emRobotTactic =BLUE_FIELD_A_RESTART;
								}
							stRobot.emBaseState = BASE_BREAK;
							stRobot.emNavState = NAV_OFF;
							stRobot.pstCurPath=NULL;
							g_fpModifyAngle = g_fpGyroAngle; //������ǰ��������

							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;
							stRobot.stPot.ssPosX=0;
							stRobot.stPot.ssPosY=0;
							stRobot.stPot.ssPosQ=0;

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
							g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;

							 InitMode(g_ucCurModeSel);
							
							g_ucAllContinueFlag =0;
							stRobot.emRobotTask = TASK_INSERT_3;
					         }
				}

			#if 0//��ʱδ��

			/******************************************��ʼ���Լ����***************************************************************************************/

				switch(stRobot.emRobotCheck)
					{
						case SELFCHECK_INIT:
							
							break;
						case SELFCHECK1:
						
							break;
						case SELFCHECK2:
							break;
						case SELFCHECK3:
							break;
						case SELFCHECK4:
							break;
						case SELFCHECK5:
							break;
						default:
							break;
					}
					
#endif

/***************************************************�������ȫλ�õ�һЩ����***************************************************************************/


					if(stAction_M2.pstCurPath->siEndCode<0)
						{
							stAction_M2.pstCurPath->siEndCode=10;
						}
					if(stAction_M2.pstCurPath->siEndCode>1445)
						{
							stAction_M2.pstCurPath->siEndCode=1445;
						}




/***************************************************��������ֵ��һЩ����******************************************************************/
			
			
			if((stJSValue.usJsState == 0x73)&&JS_B2(stJSValue.usReserved)&&JS_RIGHT(stJSValue.usReserved))
				{
					while(stJSValue.usReserved != 0xffff)
						{
							stJSValue.usReserved = JOYSTICK_RESERVED;
									
						}
					SaveAllWord32();
					
				}
			
			if((stJSValue.usJsState == 0x73)&&JS_B4(stJSValue.usReserved)&&JS_LEFT(stJSValue.usReserved))
				{
					while(stJSValue.usReserved != 0xffff)
						{
							stJSValue.usReserved = JOYSTICK_RESERVED;
									
						}
					
			/*********************�쳡·����Flash�洢****************************************************************/		
					if((stRobot.emRobotTactic==RED_FIELD)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)	
						||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART))
						{

							
							if(stRobot.pstCurPath==g_pstCatchBanPath)
								{
									g_siFlashValue[OFST_RED_CATCHMIDDLEBAN_RES1]=stAction_M1.fpPotFB;

									
									g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_X]=stRobot.stPot.ssPosX;
									g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Y]=stRobot.stPot.ssPosY;
									g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Q]=stRobot.stPot.ssPosQ;
									/*
									SaveOneWord32(stRobot.stPot.ssPosX, OFST_RED_CATCHBANPATH_RES1_X);	
									SaveOneWord32(stRobot.stPot.ssPosY, OFST_RED_CATCHBANPATH_RES1_Y);	
									SaveOneWord32(stRobot.stPot.ssPosQ, OFST_RED_CATCHBANPATH_RES1_Q);	
									*/
								}
							else if(stRobot.pstCurPath==g_pstReleaseBanPath)
								{
									g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_X]=stRobot.stPot.ssPosX;
									g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Y]=stRobot.stPot.ssPosY;
									g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Q]=stRobot.stPot.ssPosQ;
									/*
									SaveOneWord32(stRobot.stPot.ssPosX, OFST_RED_RELEASEBANPATH_RES1_X);	
									SaveOneWord32(stRobot.stPot.ssPosY, OFST_RED_RELEASEBANPATH_RES1_Y);	
									SaveOneWord32(stRobot.stPot.ssPosQ, OFST_RED_RELEASEBANPATH_RES1_Q);	*/	
								}
							
						}

					else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
						
						{
							
							if(stRobot.pstCurPath==g_pstCatchBanPath)
								{
									g_siFlashValue[OFST_RED_CATCHMIDDLEBAN_RES2]=stAction_M1.fpPotFB;

									
									g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_X]=stRobot.stPot.ssPosX;
									g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Y]=stRobot.stPot.ssPosY;
									g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Q]=stRobot.stPot.ssPosQ;
									/*
									SaveOneWord32(stRobot.stPot.ssPosX, OFST_RED_CATCHBANPATH_RES2_X);	
									SaveOneWord32(stRobot.stPot.ssPosY, OFST_RED_CATCHBANPATH_RES2_Y);	
									SaveOneWord32(stRobot.stPot.ssPosQ, OFST_RED_CATCHBANPATH_RES2_Q);	*/
								}
							else if(stRobot.pstCurPath==g_pstReleaseBanPath)
								{
									g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_X]=stRobot.stPot.ssPosX;
									g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Y]=stRobot.stPot.ssPosY;
									g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Q]=stRobot.stPot.ssPosQ;
									/*
									SaveOneWord32(stRobot.stPot.ssPosX, OFST_RED_RELEASEBANPATH_RES2_X);	
									SaveOneWord32(stRobot.stPot.ssPosY, OFST_RED_RELEASEBANPATH_RES2_Y);	
									SaveOneWord32(stRobot.stPot.ssPosQ, OFST_RED_RELEASEBANPATH_RES2_Q);*/		
								}
							
						}
					else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
						
						{
							
							if(stRobot.pstCurPath==g_pstCatchBanPath)
								{
									g_siFlashValue[OFST_RED_CATCHMIDDLEBAN_RES3]=stAction_M1.fpPotFB;

									
									g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_X]=stRobot.stPot.ssPosX;
									g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Y]=stRobot.stPot.ssPosY;
									g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Q]=stRobot.stPot.ssPosQ;
									/*
									SaveOneWord32(stRobot.stPot.ssPosX, OFST_RED_CATCHBANPATH_RES3_X);	
									SaveOneWord32(stRobot.stPot.ssPosY, OFST_RED_CATCHBANPATH_RES3_Y);	
									SaveOneWord32(stRobot.stPot.ssPosQ, OFST_RED_CATCHBANPATH_RES3_Q);	*/
								}
							else if(stRobot.pstCurPath==g_pstReleaseBanPath)
								{
									
									g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_X]=stRobot.stPot.ssPosX;
									g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Y]=stRobot.stPot.ssPosY;
									g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Q]=stRobot.stPot.ssPosQ;
									/*
									SaveOneWord32(stRobot.stPot.ssPosX, OFST_RED_RELEASEBANPATH_RES3_X);	
									SaveOneWord32(stRobot.stPot.ssPosY, OFST_RED_RELEASEBANPATH_RES3_Y);	
									SaveOneWord32(stRobot.stPot.ssPosQ, OFST_RED_RELEASEBANPATH_RES3_Q);		*/
								}
							
						}
					else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
						{
						
						}

					/********************************����·����Flash�洢**************************************************/
					else if((stRobot.emRobotTactic==BLUE_FIELD)
						||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)	
						||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART))
						{
							
							if(stRobot.pstCurPath==g_pstCatchBanPath)
								{
									g_siFlashValue[OFST_BLUE_CATCHMIDDLEBAN_RES1]=stAction_M1.fpPotFB;
									
									g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_X]=stRobot.stPot.ssPosX;
									g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Y]=stRobot.stPot.ssPosY;
									g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Q]=stRobot.stPot.ssPosQ;
									/*
									SaveOneWord32(stRobot.stPot.ssPosX, OFST_BLUE_CATCHBANPATH_RES1_X);	
									SaveOneWord32(stRobot.stPot.ssPosY, OFST_BLUE_CATCHBANPATH_RES1_Y);	
									SaveOneWord32(stRobot.stPot.ssPosQ, OFST_BLUE_CATCHBANPATH_RES1_Q);*/	
								}
							else if(stRobot.pstCurPath==g_pstReleaseBanPath)
								{
									g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_X]=stRobot.stPot.ssPosX;
									g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Y]=stRobot.stPot.ssPosY;
									g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Q]=stRobot.stPot.ssPosQ;
									/*
									SaveOneWord32(stRobot.stPot.ssPosX, OFST_BLUE_RELEASEBANPATH_RES1_X);	
									SaveOneWord32(stRobot.stPot.ssPosY, OFST_BLUE_RELEASEBANPATH_RES1_Y);	
									SaveOneWord32(stRobot.stPot.ssPosQ, OFST_BLUE_RELEASEBANPATH_RES1_Q);	*/	
								}
						}

					else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
						
						{
							if(stRobot.pstCurPath==g_pstCatchBanPath)
								{
									g_siFlashValue[OFST_BLUE_CATCHMIDDLEBAN_RES2]=stAction_M1.fpPotFB;
									
									g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_X]=stRobot.stPot.ssPosX;
									g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Y]=stRobot.stPot.ssPosY;
									g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Q]=stRobot.stPot.ssPosQ;
									/*
									SaveOneWord32(stRobot.stPot.ssPosX, OFST_BLUE_CATCHBANPATH_RES2_X);	
									SaveOneWord32(stRobot.stPot.ssPosY, OFST_BLUE_CATCHBANPATH_RES2_Y);	
									SaveOneWord32(stRobot.stPot.ssPosQ, OFST_BLUE_CATCHBANPATH_RES2_Q);*/	
								}
							else if(stRobot.pstCurPath==g_pstReleaseBanPath)
								{
									g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_X]=stRobot.stPot.ssPosX;
									g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Y]=stRobot.stPot.ssPosY;
									g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Q]=stRobot.stPot.ssPosQ;
									/*
									SaveOneWord32(stRobot.stPot.ssPosX, OFST_BLUE_RELEASEBANPATH_RES2_X);	
									SaveOneWord32(stRobot.stPot.ssPosY, OFST_BLUE_RELEASEBANPATH_RES2_Y);	
									SaveOneWord32(stRobot.stPot.ssPosQ, OFST_BLUE_RELEASEBANPATH_RES2_Q);		*/
								}
							
						}
					else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)
						
						{
							if(stRobot.pstCurPath==g_pstCatchBanPath)
								{
									g_siFlashValue[OFST_BLUE_CATCHMIDDLEBAN_RES3]=stAction_M1.fpPotFB;
									
									g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_X]=stRobot.stPot.ssPosX;
									g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Y]=stRobot.stPot.ssPosY;
									g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Q]=stRobot.stPot.ssPosQ;
									/*
									SaveOneWord32(stRobot.stPot.ssPosX, OFST_BLUE_CATCHBANPATH_RES3_X);	
									SaveOneWord32(stRobot.stPot.ssPosY, OFST_BLUE_CATCHBANPATH_RES3_Y);	
									SaveOneWord32(stRobot.stPot.ssPosQ, OFST_BLUE_CATCHBANPATH_RES3_Q);	*/
								}
							else if(stRobot.pstCurPath==g_pstReleaseBanPath)
								{
									g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_X]=stRobot.stPot.ssPosX;
									g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_Y]=stRobot.stPot.ssPosY;
									g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_Q]=stRobot.stPot.ssPosQ;
									/*
									SaveOneWord32(stRobot.stPot.ssPosX, OFST_BLUE_RELEASEBANPATH_RES3_X);	
									SaveOneWord32(stRobot.stPot.ssPosY, OFST_BLUE_RELEASEBANPATH_RES3_Y);	
									SaveOneWord32(stRobot.stPot.ssPosQ, OFST_BLUE_RELEASEBANPATH_RES3_Q);		*/
								}
							
						}
					else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART4)
						{
						
						}
				}



			if( g_ucFlashLoad==1)
				{
					g_ucFlashLoad=0;
					
					
					for(ucCnt_i=0;ucCnt_i<100;ucCnt_i++)
						{
							g_siFlashValue[ucCnt_i]=*(__IO SINT32*)(ADDR_FLASH_SECTOR_11+(ucCnt_i<<2));
						}
						#if	1
				////////////////////////************�쳡·������*******************/////////////////////////////////////////////


						/*************************������ʽ1(����ģʽ)***************************************/
						//////////////////����ȡ��ͷ//////////////////
						stNavPathRed[1].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_X];
						stNavPathRed[1].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Y];
						stNavPathRed[1].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Q];

						//////////////////��������ͷ//////////////////
						stNavPathRed[2].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_X];
						stNavPathRed[2].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Y];
						stNavPathRed[2].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Q];

						//////////////////����ȡ��ͷ//////////////////
						stNavPathRed[4].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_X];
						stNavPathRed[4].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Y];
						stNavPathRed[4].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Q];

						//////////////////��������ͷ//////////////////
						stNavPathRed[5].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_X];
						stNavPathRed[5].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Y];
						stNavPathRed[5].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Q];

						//////////////////����ȡ��ͷ//////////////////
						stNavPathRed[7].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_X];
						stNavPathRed[7].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Y];
						stNavPathRed[7].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Q];

						//////////////////��������ͷ//////////////////
						stNavPathRed[8].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_X];
						stNavPathRed[8].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Y];
						stNavPathRed[8].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Q];
						


						/*************************������ʽ2***************************************/
						//////////////////����ȡ��ͷ//////////////////
						stNavPathRed[10].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_X];
						stNavPathRed[10].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Y];
						stNavPathRed[10].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Q];
						//////////////////��������ͷ//////////////////
						stNavPathRed[11].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_X];
						stNavPathRed[11].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Y];
						stNavPathRed[11].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Q];

						//////////////////����ȡ��ͷ//////////////////
						stNavPathRed[12].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_X];
						stNavPathRed[12].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Y];
						stNavPathRed[12].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Q];
						//////////////////��������ͷ//////////////////
						stNavPathRed[13].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_X];
						stNavPathRed[13].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Y];
						stNavPathRed[13].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Q];

						//////////////////����ȡ��ͷ//////////////////
						stNavPathRed[14].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_X];
						stNavPathRed[14].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Y];
						stNavPathRed[14].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Q];
						//////////////////��������ͷ//////////////////
						stNavPathRed[15].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_X];
						stNavPathRed[15].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Y];
						stNavPathRed[15].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Q];

						/*************************������ʽ3***************************************/	
						//////////////////����ȡ��ͷ//////////////////
						stNavPathRed[17].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_X];
						stNavPathRed[17].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Y];
						stNavPathRed[17].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Q];
						//////////////////��������ͷ//////////////////
						stNavPathRed[18].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_X];
						stNavPathRed[18].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Y];
						stNavPathRed[18].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Q];

						//////////////////����ȡ��ͷ//////////////////
						stNavPathRed[19].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_X];
						stNavPathRed[19].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Y];
						stNavPathRed[19].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Q];
						//////////////////��������ͷ//////////////////
						stNavPathRed[20].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_X];
						stNavPathRed[20].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Y];
						stNavPathRed[20].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Q];

						
						//////////////////����ȡ��ͷ//////////////////
						stNavPathRed[21].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_X];
						stNavPathRed[21].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Y];
						stNavPathRed[21].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Q];
						//////////////////��������ͷ//////////////////
						stNavPathRed[22].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_X];
						stNavPathRed[22].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Y];
						stNavPathRed[22].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Q];
					
					#endif


#if 1
					#if 1
						////////////////////////************����·������*******************/////////////////////////////////////////////


						/*************************������ʽ1(����ģʽ)***************************************/
						//////////////////����ȡ��ͷ//////////////////
						stNavPathBlue[1].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_X];
						stNavPathBlue[1].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Y];
						stNavPathBlue[1].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Q];

						//////////////////��������ͷ//////////////////
						stNavPathBlue[2].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_X];
						stNavPathBlue[2].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Y];
						stNavPathBlue[2].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Q];
					#endif
#if 1
						//////////////////����ȡ��ͷ//////////////////
						stNavPathBlue[4].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_X];
						stNavPathBlue[4].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Y];
						stNavPathBlue[4].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Q];

						//////////////////��������ͷ//////////////////
						stNavPathBlue[5].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_X];
						stNavPathBlue[5].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Y];
						stNavPathBlue[5].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Q];

						//////////////////����ȡ��ͷ//////////////////
						stNavPathBlue[7].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_X];
						stNavPathBlue[7].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Y];
						stNavPathBlue[7].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Q];

						//////////////////��������ͷ//////////////////
						stNavPathBlue[8].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_X];
						stNavPathBlue[8].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Y];
						stNavPathBlue[8].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Q];
#endif

#if 1


						/*************************������ʽ2***************************************/
						//////////////////����ȡ��ͷ//////////////////
						stNavPathBlue[10].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_X];
						stNavPathBlue[10].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Y];
						stNavPathBlue[10].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Q];
						//////////////////��������ͷ//////////////////

						#if 1
						stNavPathBlue[11].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_X];
						stNavPathBlue[11].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Y];
						stNavPathBlue[11].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Q];
						#endif

						#if 1

						//////////////////����ȡ��ͷ//////////////////
						stNavPathBlue[12].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_X];
						stNavPathBlue[12].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Y];
						stNavPathBlue[12].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Q];

						
						//////////////////��������ͷ//////////////////
						stNavPathBlue[13].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_X];
						stNavPathBlue[13].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Y];
						stNavPathBlue[13].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Q];

						//////////////////����ȡ��ͷ//////////////////
						stNavPathBlue[14].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_X];
						stNavPathBlue[14].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Y];
						stNavPathBlue[14].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Q];
						//////////////////��������ͷ//////////////////
						stNavPathBlue[15].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_X];
						stNavPathBlue[15].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Y];
						stNavPathBlue[15].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Q];
						#endif
#endif

#if 1
						/*************************������ʽ3***************************************/	
						//////////////////����ȡ��ͷ//////////////////
						stNavPathBlue[17].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_X];
						stNavPathBlue[17].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Y];
						stNavPathBlue[17].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Q];
						//////////////////��������ͷ//////////////////
						stNavPathBlue[18].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_X];
						stNavPathBlue[18].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_Y];
						stNavPathBlue[18].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_Q];

						//////////////////����ȡ��ͷ//////////////////
						stNavPathBlue[19].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_X];
						stNavPathBlue[19].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Y];
						stNavPathBlue[19].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Q];
						//////////////////��������ͷ//////////////////
						stNavPathBlue[20].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_X];
						stNavPathBlue[20].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_Y];
						stNavPathBlue[20].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_Q];

						
						//////////////////����ȡ��ͷ//////////////////
						stNavPathBlue[21].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_X];
						stNavPathBlue[21].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Y];
						stNavPathBlue[21].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Q];
						//////////////////��������ͷ//////////////////
						stNavPathBlue[22].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_X];
						stNavPathBlue[22].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_Y];
						stNavPathBlue[22].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_Q];
					#endif
#endif
					OPEN_VALVE(MID_CATCH_VALVE);
					
				}
		
		OSTimeDly(2);
	}
}
 

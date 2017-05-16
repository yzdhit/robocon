/*****************************************NEW C　ｃａｒ二代2012.6.14*****************************************************************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：RobotCtrl.c
最近修改日期：2011.07.07
版本：1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
模块描述：机器人主控程序，主函数
函数列表： 

----------------------------------------------------------------------------------------------------------------------------------------------------------
修订记录：
作者        时间            版本     说明
任伟        2011.07.07      1.0      划分建立此模块
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

UCHAR8 uc_TestAirFlag=0;//测试气缸变量
UCHAR8 uc_TestAirStartFlag=0;//测试气缸变量
UINT32 g_uiTTestAirTime = 0; //测试气缸变量
UINT32 g_uiConstTime = 2000000; //测试气缸变量

/*---------------------------模式选择相关的变量------------------------------------*/
UCHAR8  g_ucCurModeSel = 0;
UCHAR8  g_ucModeCnt = 7;
const UCHAR8  g_ucTotalMode = 9;

/*---------------------------定位相关的变量------------------------------------------*/

ST_DOUBLE_OMNI_LOCATION stDoubleOMNI = {0,0,0,0,0,0,0,0, 0, 0};
//巡线相关的值

UCHAR8 g_ucFliterLineValue[12]={75,75,75,75,75,75,75,75,75,75,75,75};
UCHAR8 g_ucLineValue = 0;
const UCHAR8 g_ucLineMidValue = 80;
const UCHAR8 g_ucLineMinValue  = 10;
const UCHAR8 g_ucLineMaxValue = 140;
FP32 g_fpPostionXTemp=0;
//更新陀螺的坐标,要更新坐标时才用
FP32 g_fpModifyAngle = 0;
FP32 g_fpGyroAngle = 0;
FP32 g_fpPotTemp =0;

//mems陀螺的值
SSHORT16 g_ssMemsGryo = 0;


/*------------------------任务追踪时的相关变量------------------------------------*/

UCHAR8 g_ucOnceRunFlag = 0;  //控制任务只执行一次的代码
UCHAR8 g_ucLoopRunFlag = 1;  // 控制任务一直执行的代码
UCHAR8 g_ucJumpRunFlag = 0; //控制任务跳转的代码
UCHAR8 g_ucTaskFlag=0;//控制任务 中需要的标质量

UCHAR8 g_ucTaskCnt = 0;     // 任务中的计数用

UINT32 g_uiTaskTime1 = 0;   //任务中的时间
UINT32 g_uiTaskTime2 = 0;
SINT32 g_siTaskDeltaTime = 0;

//UCHAR8 g_ucAllContinueFlag = 1; //默认全部自动完成，非单步执行，为0时，单步执行
UCHAR8 g_ucAllContinueFlag =0;

/*--------------------------自动导航相关的变量-------------------------------------*/
//NAV_XYQ相关的变量
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

/*---------------------------手动控制的相关变量------------------------------------*/

//手动控制机器人时的目标速度
ST_ROBOT_VELT g_stRobotDesV = {0,0,0};
SINT32 g_siDirAngle = 0;

/*----------------------------执行电机变量的定义----------------------------------*/
//执行电机的预期速度的影子，在执行电机的速度闭环模式下使用
FP32 g_fpActionM1VeltDesShadow = 0;
FP32 g_fpActionM2VeltDesShadow = 0;
FP32 g_fpActionM3VeltDesShadow = 0;

//执行电机的位置闭环反馈的位置
FP32  g_fpActionM1PotFB = 0;
FP32  g_fpActionM2PotFB = 0;

//控制电机路径闭环时，每次的增量,用于手动控制
FP32   g_fpM1DeltaPos = 40;//竖直提升电机
FP32   g_fpM1DeltaPos1 = 5;//竖直提升电机
FP32   g_fpM1DeltaPos2 = 20;//竖直提升电机
FP32   g_fpM2DeltaPos = 80;//上部水平电机
FP32   g_fpM2DeltaPos2 = 40;//竖直提升电机


UCHAR8 g_ucWatchComTemp;//通信观测变量
/*--------------------------M车通信的变量定义--------------------------------------*/
UCHAR8 g_ucMC_M1UpCnt = 0;        //电机1向上动
UCHAR8 g_ucMC_M1DownCnt = 0;   //电机1向下动
UCHAR8 g_ucMC_M2OutCnt = 0;      //电机2向外伸
UCHAR8 g_ucMC_M2InCnt =0;         //电机2往回缩
UCHAR8 g_ucMC_CatchCnt = 0;       //抓包子 
UCHAR8 g_ucMC_ReleaseCnt = 0;   //放包子
UCHAR8 g_ucMC_RotFitCnt = 0;      //顺时针旋转
UCHAR8 g_ucMC_RotOpstCnt = 0;   //逆时针旋转
UCHAR8 g_ucMC_AutoCnt = 0;        // 自动放
UCHAR8 g_ucMC_Rot_ValveCnt = 0;        // 自动旋转气缸
UCHAR8 g_ucMC_InitCnt=0;		//初始化命令
UCHAR8  g_ucMCComStatusFlag=0;//MC通信是否关闭命令

/******************************A车通信变量定义***************************************************************/
UCHAR8 g_ucAC_LeaveStartCnt = 0;//A车离开后对C车的通信
UCHAR8 g_ucAC_L2Cnt=0;  //  A车到达L2之后的对C车的通信
UCHAR8 g_ucAC_BasketCnt=0; // C上岛后对A车的通信
 UCHAR8 g_ucAC_LiftUpCnt=0;//C在A车上是否被提起的通信

const UCHAR8 g_ucMC_CntTime = 3;

UCHAR8  g_ucACComStatusFlag=0;//AC通信是否关闭的命令

//M车手动控制C的标志量

/*----------------------------液晶变量的定义----------------------------------------*/
// 控制液晶的翻页
UCHAR8 g_ucCurLcdPage = 0;
const UCHAR8 g_ucTotalLcdPage = 30;

/**********************************路径声明默认都是红场低速*************************************************************/
ST_PATH  *g_pstL2Path=&stNavPathRed[0];
ST_PATH  *g_pstCatchBanPath=&stNavPathRed[1];
ST_PATH  *g_pstReleaseBanPath=&stNavPathRed[2];
ST_PATH  *g_pstGoBasketPath=&stNavPathRed[24];//默认是红场

/*************************************************************************************************/

/***********************************flash存储相关变量************************************************************/

SINT32 g_siTestFlashRead = 0;
UCHAR8 g_ucFlashFlag=0;
UCHAR8 g_ucFlashWrongNum=0;
SINT32 g_siFlashValue[100]={-1};
/*--------------------------------视觉相关的定义----------------------------*/
UINT32 g_uiVisionUpdateTime = 0;
UCHAR8  g_ucVisionUsedCnt = 0;
FP32 g_fpVisionValue = 0;
FP32 g_fpVisionPotY = 0;

/******************************红蓝场按键相关变量**************************************/
UCHAR8 g_ucKeyFlag=1;//红蓝场按键启用标质量，0代表红场，1代表蓝场

/******************************************************************/

/************************定义机器人是否是第一次运行的变量*****************************************/
UCHAR8 g_FirstRun=0;//0代表机器人未运行，1代表机器人已运行
/***************************************************************/

/*********************AC配合时可以不通信的标志量*******************************************/
UCHAR8 g_ACFlag1=0;//AC在A处离开时，不通过通信的标志量
UCHAR8 g_ACFlag2=0;//AC起始通信是否成功标志量
UCHAR8 g_ACFlag3=0;//AC在L2登岛处不通过通信的标志量
UCHAR8 g_ACFlag4=0;//AC在L2登岛处电机下降 的标志量
UCHAR8 g_ACFlag5=0;//C车在L2登岛时纠正角度的标志量
UCHAR8 g_CFlag6=0;//C车在取完馒头之后上升高度标志量
UCHAR8 g_CFlag7=0;//C车在在取顶层馒头的气缸旋转标志量
UCHAR8 g_CFlag8=0;//C车在在取顶层馒头的气缸旋转标志量

UCHAR8 g_AC_NoCom_On_Off=0;//等于1表示不通过通信的模式开启，等于0表示只通过通信
SINT32  g_M3TempCoder=0;//C车上岛时需要做闭环的脉冲

UCHAR8 g_SelectState_Electric_On=0;//是否选择带电进入重启状态键

UCHAR8 g_ucFlashLoad=0;//是否加载Flash标志量，0表示不加载，1表示加载Flash

UCHAR8  g_ucUseMemsGryo=0;//是否启用Mems陀螺，1带表启用Mems陀螺，0代表启用模拟陀螺

UINT32 g_uiCPUUsage = 0;
UINT32 g_uiCPUUsageMax = 0;

UINT32 g_TimeTemp=0;
UINT32 g_fpValveCoff=25;
int main(void )
{
	OSInit();				//UCOS INIT~
 	ChipHalInit();
	ChipOutHalInit();
	CREATE_OS_TASK(InitTask);		//启动初始化任务
	OSStart();
	while(1)
	{	
	}
}

/****************************************************************************************************
任务名称：InitTask()
任务功能：对设备以及一些机器人的参数进行初始化，启动机器人控制的其他相关任务
****************************************************************************************************/
void InitTask (void *pdata)
{
	pdata = pdata;
       OSTimeDly(500);//
	INIT_FPGA(); //初始化fpga 
	CODER_INIT();  //所有码盘通道打开
 	PWM_MOTOR_INIT();//所有电机的pwm都打开
 	PWM_SERVO_INIT();//所有舵机的pwm都打开

	OSTimeDly(100);//
	/*创建其他任务*/
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
任务名称：DispatchTask ()
任务功能：分配任务
****************************************************************************************************/


void DispatchTask (void *pdata)
{
       
	stRobot.emNavState = NAV_OFF;//默认导航关闭
	
	while(1)
	{

		//定位机器人
		DoubleVerticalOMNILocateEx1(&stDoubleOMNI, &stRobot);
		//DoubleVerticalOMNILocate(&stDoubleOMNI, &stRobot);
		//DoubleVerticalOMNILocatenew(&stDoubleOMNI, &stRobot);
		
		switch(stRobot.emRobotTask)
		{
		
			case NO_ACTION:
				/*什么也不做*/
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
			case INIT://初始提升一点电机位置
				if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//在此添加只执行一次的代码,  要确认各个电机的状态
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
					// 在此添加循环执行代码
                                     

					// 在此判定是否跳转
					if(stAction_M1.emPathState==PATH_END)
					{
						g_ucJumpRunFlag = 1;
						
					}

				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{
						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
						g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//进行跳转的操作
						stRobot.emRobotTask = TASK_START_1;	
					}
				}
				
				break;
			case TASK_START_1://M车抓住之后的通信，收回前部的支撑杆
				 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺角度，用在后面M取C,C车提升的判断上
						//在此添加只执行一次的代码,  要确认各个电机的状态
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
                                  // if(IS_SWITCH_ON(RIGHT_LIGHT_SWITCH)||IS_SWITCH_ON(LEFT_LIGHT_SWITCH))//加上此处判断，只有当M车提起C车的时候，才打开通信
                                          if(1)   //避免接触时，开关不亮，收不到通信
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
					// 在此判定是否跳转
					if((g_ucMC_M1DownCnt>g_ucMC_CntTime)||(abs(stRobot.stPot.ssPosQ)>200))//此处加了通过机器人的角度来判断，是否将车的架子提起来
						{
	                                       
							g_ucJumpRunFlag = 1;
							g_ucAllContinueFlag =1;
							
						}



					/********************************************************************************************************/	
					 //此处为了微调高度
						  if(KEY_F5(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
						        stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						        g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
							 SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathLow, g_fpActionM1PotFB + g_fpM1DeltaPos1);
						       
							}	
					        

						  if(KEY_F4(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
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
					
							// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
							{

								g_ucOnceRunFlag = 0;
								g_ucLoopRunFlag = 1;
								g_ucJumpRunFlag = 0;
								g_ucTaskCnt = 0;
								g_uiTaskTime1 = 0;
								g_uiTaskTime2 = 0;
								g_siTaskDeltaTime = 0;
			/*****************************改过***********************************************/
								//进行跳转的操作
								//stRobot.emRobotTask = TASK14;
								stRobot.emRobotTask =TASK_INSERT_2;
			/*******************************************************************************/
							}    
				}
			break;
			
			case TASK_INSERT_2://等待AC离开A的部分
				 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
									   
						//在此添加只执行一次的代码,  要确认各个电机的状态
						g_ACFlag1=0;
						g_ACFlag2=0;
						g_ucAC_LeaveStartCnt=0;
						stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
	 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1M_PUTDOWN_C*M1COFF_POT);//M提升C之后，C降下的高度
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
					// 在此添加循环执行代码
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
					

					if((g_ucAC_LeaveStartCnt>g_ucMC_CntTime)&&(g_ACFlag2==0))//此处再次纠正陀螺坐标，因为不通过通信纠正过的角度可能不准确,并且只纠正一次
						{
							g_fpModifyAngle = g_fpGyroAngle; 
							stRobot.stPot.ssPosQ = 0;
							g_ucAC_LeaveStartCnt=0;
							g_ACFlag2=1;//AC起始通信是否成功标质量
							g_ucJumpRunFlag = 1;
							g_ucAllContinueFlag = 1;//自动
						}

					if(g_ACFlag2==1)
						{
							/*****************AC通信是否成功标志量**************************/

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
							g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标
							stRobot.stPot.ssPosQ = 0;
							g_ACFlag2=1;//AC起始通信是否成功标质量
							g_ucJumpRunFlag = 1;
							g_ucAllContinueFlag = 1;//自动
						}
					/***********************不用通信也可在A上下降的程序*********************************************/
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
									// 在此判定是否跳转
									g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标
									stRobot.stPot.ssPosQ = 0;
									g_ACFlag1=1;
									g_ucJumpRunFlag = 1;
									g_ucAllContinueFlag = 1;//自动
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
					// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
						g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//进行跳转的操作
						stRobot.emRobotTask = TASK_INSERT_3;
					}
				}
				
				break;
			case TASK_INSERT_3://A车将要到达岛的时候，提升电机,此处也是AC在A处重启的入口
				if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						
									   
						//在此添加只执行一次的代码,  要确认各个电机的状态
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
	 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1A_DOWN_C*M1COFF_POT);//在A车上下降的高度
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
					// 在此添加循环执行代码
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

					
					/*******************AC通信成功自己纠正陀螺角度*********************************/
					if((g_ucAC_LeaveStartCnt>g_ucMC_CntTime)&&(g_ACFlag2==0))//此处再次纠正陀螺坐标，因为不通过通信纠正过的角度可能不准确,并且只纠正一次
						{
							g_fpModifyAngle = g_fpGyroAngle; 
							stRobot.stPot.ssPosQ = 0;
							g_ucAC_LeaveStartCnt=0;
							g_ACFlag2=1;//AC起始通信是否成功标质量
						}
					
					if((g_ucAC_LiftUpCnt>g_ucMC_CntTime)||JS_L1(JOYSTICK_RESERVED))
						{
							
			/*************当C车在A车上抬起时，更新XY坐标，使得C车在岛上的滑行距离也计算在历程之内******************/
							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;

							stRobot.stPot.ssPosX = 0;
							stRobot.stPot.ssPosY = 0;
						/***********************************/	

							//g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标有时候A车会晃，故将角度提前到C车在A车上下去的情况
							
							 //stRobot.stPot.ssPosQ = 0;
							// 在此判定是否跳转
							g_ACFlag2=1;//AC通信是否成功标质量
							g_ucJumpRunFlag = 1;
							g_ucAllContinueFlag =1;
						}

					
							
					/******C车自动登岛程序，不通过通信***********************/

					if(((GetCurTime()-g_uiTaskTime1)>3000000)&&(g_ACFlag2==0)&&(g_AC_NoCom_On_Off==1))//延时4S再启动不通过通信就可以上岛的程序
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
											// 在此判定是否跳转
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
							/*****************AC通信是否成功标志量**************************/

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
					// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
						g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//进行跳转的操作
						stRobot.emRobotTask = TASK_START_2;
					}
				}
				
				break;
			case TASK_START_2://在A车上,将要登岛的情况,此时电机升到将要上L2的高度，即S2重启之后的高度
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//在此添加只执行一次的代码,  要确认各个电机的状态
					stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1A_UP_C*M1COFF_POT);//AC将要上岛之前C车提升的高度
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
					/***********AC将要上岛时，底盘开环************************/
					stRobot.emNavState =  NAV_NULL;
					stRobot.emBaseState=BASE_BREAK;
					while(IrComm1IsDataInRxBuf())
						{
							IrComm1GetRxBufDat();
						}
				}

				if(g_ucLoopRunFlag == 1)
				{
					// 在此添加循环执行代码

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

									// 在此判定是否跳转
									if(g_ucAC_L2Cnt>g_ucMC_CntTime)
									{			
										g_ucAC_L2Cnt=0;
										
										g_ucJumpRunFlag = 1;

										g_ucAllContinueFlag = 1;//自动
									}
							/**********************不通过通信的情况下，C车自动登岛*******************************/

								if((g_ACFlag3==1)&&(g_AC_NoCom_On_Off==1))
										{
										if(stAction_M1.emPathState==PATH_END)
											{
											g_ucJumpRunFlag = 1;

											g_ucAllContinueFlag = 1;//自动
											}
										}

								/**********************************************************************/
								if(JS_R1(JOYSTICK_RESERVED) )
									{
										g_ucJumpRunFlag = 1;

										g_ucAllContinueFlag = 1;//自动
									}
								/************************************************************************/
					
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
						g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//进行跳转的操作
						stRobot.emRobotTask = TASK2;
					}
				}
				break;
/*****************************重启时进入的路径************************************************/		
				
			case TASK1://L2上重启时进入的路径
				
				//稍微提升电机，并收前腿
				 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//在此添加只执行一次的代码,  要确认各个电机的状态
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
						// 在此添加循环执行代码
	                                  
						if(stAction_M1.emPathState==PATH_END)
						{
	                                          OPEN_VALVE(FRONT_LEG_VALVE);
							// 在此判定是否跳转
							g_ucJumpRunFlag = 1;
						
						}
						
					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
							g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//进行跳转的操作
							stRobot.emRobotTask = TASK2;
						}
					}
					
				break;
			case TASK2://L2上的相关动作和路径
				//纠正陀螺角度后，跑一条路径
				if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//在此添加只执行一次的代码,  要确认各个电机的状态
						 g_ACFlag4=0 ;
						USE_BLACK_FIELD_CFG(LINE_CHAN_0);
						//g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标

					#if 0//当C车在A车上抬起时，更新XY坐标，使得C车在岛上的滑行距离也计算在历程之内，故此段代码禁用
						stDoubleOMNI.fpLastPosX=0;
						stDoubleOMNI.fpLastPosY=0;

					     /* g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标
						stRobot.stPot.ssPosQ = 0;将纠正角度提前到C车在A车上提升的时候*/
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
								stRobot.pstCurPath->usStartV=(ReadRobotFpVy(&stRobot)+150);//暂时未写检测机器人Y向速度
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
		 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_L2_UP_C*M1COFF_POT);//在A车上下降的高度
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
									// 在此添加循环执行代码
								}

							if(stRobot.stPot.ssPosY>550)
								{
									CopyIntSlowlyEx(&(stRobot.pstCurPath->ssEndQ),0,1);
									//纠正x，用巡线
									g_fpPostionXTemp=-g_ucLineValue + g_ucLineMidValue;
				                                   
									CopyFloatSlowly(&stDoubleOMNI.fpLastPosX,&g_fpPostionXTemp, 1);
	
									//CopyFloatSlowly(&g_fpModifyAngle,&g_fpAngleTemp, 1);
								}
							else//在距离小于400时，不纠正x
								{
									stDoubleOMNI.fpLastPosX=0;
									stRobot.stPot.ssPosX = 0;
									//stRobot.pstCurPath->ssStartQ=stRobot.stPot.ssPosQ;
									//stRobot.pstCurPath->ssEndQ=stRobot.stPot.ssPosQ;
									

									//g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标
									
								}
						}

					else
						{
						   /*********重启时不再发数，防止A车乱动
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
							// 在此添加循环执行代码
						
							//stRobot.pstCurPath->ssStartQ=stRobot.stPot.ssPosQ;
							//stRobot.pstCurPath->ssEndQ=stRobot.stPot.ssPosQ;//重启时不纠正角度，防止机器人晃动
							
							
				                      //纠正x，用巡线
				                      g_fpPostionXTemp=g_ucLineValue - g_ucLineMidValue;
							//CopyFloatSlowly(&stDoubleOMNI.fpLastPosX,&g_fpPostionXTemp, 1);
						}

						// 在此判定是否跳转
						if(stRobot.emPathState == PATH_END || IS_SWITCH_ON(FRONT_LEG_SWITCH))
						{
							g_ucJumpRunFlag = 1;
						}
					
				}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{
							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
							g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//进行跳转的操作
							stRobot.emRobotTask = TASK_INSERT_4;
						}
					}
				break;
		       case TASK_INSERT_4://准备在上岛之前纠正X
			   	 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//在此添加只执行一次的代码,  要确认各个电机的状态
					g_ucTaskCnt = 0;
					
					g_ucRobotXEnable = 1;
					g_ucRobotYEnable = 2;
					g_ucRobotQEnable = 1;

					g_siRobotXDesShadow = 0;
					g_siRobotQDesShadow = 0;
					/***************************更新残留值*******************************************************/
					g_siRobotXDes =  -g_ucLineValue + g_ucLineMidValue;;
					g_siRobotQDes = stRobot.stPot.ssPosQ;

					stRobot.stVeltDes.fpVy = 249;
					stRobot.emNavState = NAV_XYQ;
					
				}

				if(g_ucLoopRunFlag == 1)
					{
						// 在此添加循环执行代码

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

	                                   // 在此判定是否跳转
						
						if(g_ucTaskCnt>=3)
							{
							     g_ucJumpRunFlag = 1;
							}
						
					}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//进行跳转的操作
							stRobot.emRobotTask = TASK3;
						}
					}
			   	break;
			case TASK3:
				//两个行程开关触碰岛侧面，并纠正陀螺角度

				if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//在此添加只执行一次的代码,  要确认各个电机的状态
						g_ucRobotXEnable = 2;
						g_ucRobotYEnable = 2;
						g_ucRobotQEnable = 2;
						g_ACFlag5=0;
						
						stRobot.emNavState = NAV_XYQ;
						
					}

				if(g_ucLoopRunFlag == 1)
				{
					// 在此添加循环执行代码
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
          
					if(g_ucTaskCnt > 30 && IS_SWITCH_ON(RIGHT_POS_SWITCH)&&IS_SWITCH_ON(LEFT_POS_SWITCH) )//原来是10
					{
						g_fpModifyAngle = g_fpGyroAngle;
						
					// 在此判定是否跳转
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
						
							// 在此判定是否跳转
							g_ucJumpRunFlag = 1;
						}
					
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
						g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//进行跳转的操作
						stRobot.emRobotTask = TASK4;
					}
					
				}
				break;
			case TASK4://登岛的电机提升
				//电机提升
				 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//在此添加只执行一次的代码,  要确认各个电机的状态
						stAction_M3.fpVeltDes=-50;
						stAction_M3.emState = MOTOR_VELT_CLOSE_LOOP_CTRL;//早点让电机转，防止车弹回
						stAction_M3.emState = MOTOR_OPEN_LOOP_CTRL;
						stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
						if((g_ucCurModeSel==C_MODE_TOP_SPEED_8)||(g_ucCurModeSel==C_MODE_MID_SPEED_7))
							{
								SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid,M1_CLIMBSTEP_C*M1COFF_POT);//等到时电机提升的高度原来角度值为-780,-230
							}
						else
							{
								SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathLow,M1_CLIMBSTEP_C*M1COFF_POT);//等到时电机提升的高度原来角度值为-780,-230
							}
						stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						g_uiTaskTime1 = GetCurTime();
					}

				if(g_ucLoopRunFlag == 1)
					{
						// 在此添加循环执行代码
				
						if((stAction_M1.emPathState == PATH_END)||(GetCurTime()-g_uiTaskTime1>1500000))
						{
						// 在此判定是否跳转
							g_ucJumpRunFlag = 1;
						}

					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                          g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//进行跳转的操作
							stRobot.emRobotTask = TASK5;
						}
					}
			       break;
		       case TASK5://电机提升到顶部，前部的小电机开始动作
			   	 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//在此添加只执行一次的代码,  要确认各个电机的状态
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
						// 在此添加循环执行代码
	                                   if(IS_SWITCH_ON(BACK_LEG_SWITCH_FRONT))
	                                   {
							//OPEN_VALVE(BACK_LEG_VALVE);//原来安装后部狗腿气缸的情况
							
						/*******************采用双闭环控制缩短提升时间*********************************/
							stAction_M1.emState=MOTOR_POS_CLOSE_LOOP_CTRL;
							stAction_M1.fpPotDes=M1_DOUBLEPOTLOOP_C*M1COFF_POT;//轮子完全接触岛面
						/********************************************************/
															
							 stDoubleOMNI.fpLastPosY = 0;//原来-10
							 
							stAction_M3.fpVeltDes=0;
							stAction_M3.emState = MOTOR_VELT_CLOSE_LOOP_CTRL;
							 
							 // 在此判定是否跳转
							 
							 g_ucJumpRunFlag = 1;
	                                   }

						
					}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                      			g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//进行跳转的操作


/********************************更改过状态*****************************************************/
						
						//stRobot.emRobotTask = TASK14;
						stRobot.emRobotTask = TASK_INSERT_1;

/******************************************************************************************************************/
						
					}
				}
			   	break;
			case TASK_INSERT_1://检测电机位置闭环是否达到位置
				//电机提升
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//在此添加只执行一次的代码,  要确认各个电机的状态
					
				}

				if(g_ucLoopRunFlag == 1)
				{
					// 在此添加循环执行代码
					
					if(fabs(stAction_M1.fpPotFB-M1_DOUBLEPOTLOOP_C*M1COFF_POT)<5)
						{
							// 在此判定是否跳转
				
							g_ucJumpRunFlag = 1;
						}
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                                          g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//进行跳转的操作
						stRobot.emRobotTask = TASK6;
					}
				}
				break;
			case TASK6://车体将要接触岛面的一些操作，包括视觉定位
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					
					g_ucTaskCnt=0;
					//在此添加只执行一次的代码,  要确认各个电机的状态
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
							SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_PICKUPBAN_TEMP_C*M1COFF_POT);//此高度是C车取馒头时暂时提升的高度，防止剐蹭地面，后面再提到取馒头的高度
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						}
					else 
						{
							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
							SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_PICKUPBAN_TEMP_C*M1COFF_POT);//此高度是C车取馒头时暂时提升的高度，防止剐蹭地面，后面再提到取馒头的高度
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;	
						}
					
				}

				 #if 1
				if(g_ucLoopRunFlag == 1)
				{
					// 在此添加循环执行代码

					stDoubleOMNI.fpLastPosX=-g_ucLineValue + g_ucLineMidValue;	
					stRobot.stPot.ssPosX=-g_ucLineValue + g_ucLineMidValue;
					#if 1//视觉定位部分
                                    g_fpVisionValue =(FP32) (RetVisionLineValue(&stVisionDetecLine));
					if(g_fpVisionValue > 0)
					{
					     g_fpVisionValue = g_fpVisionValue / 10.0;
					     g_fpVisionPotY = g_fpVisionValue  +60* sin(stRobot.stPot.ssPosQ * RADIAN_10)- 36;
					     g_fpPotTemp = stRobot.stPot.ssPosY;
					}
					
					#endif
					  if((stAction_M1.emPathState==PATH_END)||((GetCurTime()-g_uiTaskTime1) >2000000))//防止竖直电机出现问题，而车不会移动，故此处加上延时
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

								// 在此判定是否跳转
								g_ucJumpRunFlag =1;	
								
		                           }
					
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;

					
					// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
					if(JS_R1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                        			g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//进行跳转的操作
						stRobot.emRobotTask = TASK7;
					}
				}
				#endif
				break;
				
			case TASK7://登岛后，通过后面的开关定位C车Y向距离，///////此处也是重启方式4，直接去M取C处的状态判断
				if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					g_ucTaskCnt=0;
					//在此添加只执行一次的代码,  要确认各个电机的状态
					
				}

				if(g_ucLoopRunFlag == 1)
				{
					// 在此添加循环执行代码
					if(IS_SWITCH_ON(BACK_LEG_SWITCH_BACK)&&(g_ucTaskCnt==0))
						{
							stDoubleOMNI.fpLastPosY=40;//原来是65
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
							// 在此判定是否跳转
		                          		g_ucJumpRunFlag = 1;   
						}
                                  

				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                                      //g_ucTaskCnt不准赋值为0，因为后面会用到
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//进行跳转的操作


/***************************在此处插入了机器人重启4的入口判断************************************************************************/
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
				
			case TASK8:	//此处为去馒头的路径			
 #if 1
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//在此添加只执行一次的代码,  要确认各个电机的状态

					/*************希望车在取馒头时尽可能重心较低，故将此段代码后移***********************************
					
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
							SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_PICKUPBAN_LAST_C*M1COFF_POT);//20,此高度也是取馒头的高度
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
									
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_PICKUPBANRES3_TEMP_C*M2COFF_POT);//先让M2电机伸出一段距离，以防止申不完
						}
					else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
						{
							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
							SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_PICKUPBAN_LAST_C*M1COFF_POT);//20,此高度也是取馒头的高度
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_PICKUPBANRES3_TEMP_C*M2COFF_POT);//先让M2电机伸出一段距离，以防止申不完
						}
					else if(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
						{	
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_PICKUPBANRES2_LAST_C*M2COFF_POT);//先让M2电机伸出一段距离，以防止申不完
						}
					else if(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
						{	
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_PICKUPBANRES2_LAST_C*M2COFF_POT);//先让M2电机伸出一段距离，以防止申不完
						}


					else if((stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART)
						||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
						||(stRobot.emRobotTactic==BLUE_FIELD))
						{
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_PICKUPBANRES1_LAST_C*M2COFF_POT);//此长度为正常启动和重启方式2共用
						}
					else if((stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==RED_FIELD_A_RESTART)
						||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
						||(stRobot.emRobotTactic==RED_FIELD))
						{
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_PICKUPBANRES1_LAST_C*M2COFF_POT);//此长度为正常启动和重启方式2共用
						}
					
				}

				if(g_ucLoopRunFlag == 1)
				{
					// 在此添加循环执行代码

					if((stRobot.emRobotTactic==BLUE_FIELD)
						||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)&&(g_ucTaskCnt==1))

						{
							

							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
							SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_PICKUPBAN_LAST_C*M1COFF_POT);//20,此高度也是取馒头的高度
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

							g_ucJumpRunFlag = 1;
							
						}
					else if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)&&(g_ucTaskCnt==1))
						{
							//if(stRobot.stPot.ssPosY>500)//此处要根据实际位置来加此处的坐标值
								{

									stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
									SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_PICKUPBAN_LAST_C*M1COFF_POT);//20,此高度也是取馒头的高度
									stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

									g_ucJumpRunFlag = 1;
								}
						}
					else if((stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)&&(g_ucTaskCnt==1))//重启时路径不一样电机提升的高度也不近相同
						{
							//if(stRobot.stPot.ssPosY>500)//if(stRobot.stPot.ssPosY>500)//此处要根据实际位置来加此处的坐标值
								{
									//由于此处新车的机构变动，故此处改角度
									//g_pstCatchBanPath->ssEndQ=RED_L2_RESTART2_CATCHBAN_Q;//在确保C车后面的定位开关触发后，才开始转动
									//g_pstReleaseBanPath->ssStartQ=RED_L2_RESTART2_CATCHBAN_Q;

									stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
									SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_PICKUPBAN_LAST_C*M1COFF_POT);//20,此高度也是取馒头的高度
									stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
									g_ucJumpRunFlag = 1;
								}
						}
					else if((stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)&&(g_ucTaskCnt==1))
						{
							//g_pstCatchBanPath->ssEndQ=BLUE_L2_RESTART2_CATCHBAN_Q;//在确保C车后面的定位开关触发后，才开始转动
							//g_pstReleaseBanPath->ssStartQ=BLUE_L2_RESTART2_CATCHBAN_Q;

							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
							SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_PICKUPBAN_LAST_C*M1COFF_POT);//20,此高度也是取馒头的高度
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							g_ucJumpRunFlag = 1;
						}
					else if((stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)&&(g_ucTaskCnt==1))
						{
							if((g_pstCatchBanPath->ssEndY-stRobot.stPot.ssPosY)<70)//此处要根据实际位置来加此处的坐标值
								{

									stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
									stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
									SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_PICKUPBANRES3_LAST_C*M1COFF_POT);//重启路径3中M2电机最终伸出来的距离

									g_ucJumpRunFlag = 1;
								}
						}

					else if((stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)&&(g_ucTaskCnt==1))
						{
							if((g_pstCatchBanPath->ssEndY-stRobot.stPot.ssPosY)<70)
								{

									

									stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
									stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
									SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_PICKUPBANRES3_LAST_C*M1COFF_POT);//重启路径3中M2电机最终伸出来的距离

									g_ucJumpRunFlag = 1;
								}
						}
					
					if(IS_SWITCH_ON(BACK_LEG_SWITCH_BACK)&&(g_ucTaskCnt==0))
						{
							//stDoubleOMNI.fpLastPosY=75;//原来安装狗腿气缸的情况
							stDoubleOMNI.fpLastPosY=40;
							stRobot.stPot.ssPosY=40;
							g_ucTaskCnt=1; 
							
						}
					
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                        			g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//进行跳转的操作
						stRobot.emRobotTask = TASK9;
					}
				}
				
#endif
				break;

			case TASK9://判断路径此处是否结束，好准备夹取馒头
				 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//在此添加只执行一次的代码,  要确认各个电机的状态
		
					}

				if(g_ucLoopRunFlag == 1)
					{
						// 在此添加循环执行代码

						if(abs(stRobot.stPot.ssPosX -g_pstCatchBanPath->ssEndX)<3 && abs(stRobot.stPot.ssPosY -g_pstCatchBanPath->ssEndY) <4 && abs(stRobot.stPot.ssPosQ-g_pstCatchBanPath->ssEndQ)<4 && (stAction_M2.emPathState == PATH_END))
						{
							g_ucTaskCnt++;
						}

							
						// 在此判定是否跳转
						if(g_ucTaskCnt>=3)
						{
							g_ucJumpRunFlag = 1;
						}
						
					}

				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					/******************************控制水平电机前后指令***************************************/
						 if(KEY_F5(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
						        
						       	stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
			 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid, g_fpActionM1PotFB + g_fpM1DeltaPos2);
							}	

						if(KEY_F4(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
							
								stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathMid, g_fpActionM1PotFB - g_fpM1DeltaPos2);
							 }
					
				/********************************控制竖直电机上下指令*******************************************************/
						if(KEY_F1(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB + g_fpM2DeltaPos2);
									
							}

						if(KEY_F8(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB - g_fpM2DeltaPos2);
								
							}

					// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                        			g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//进行跳转的操作
						stRobot.emRobotTask = TASK10;
					}
				}
				
				break;
			case TASK10://夹取中层馒头的相关操作

#if 1
			     if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//在此添加只执行一次的代码,  要确认各个电机的状态
						if((stRobot.emRobotTactic==RED_FIELD)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
						||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
						||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
							{
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
								SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_CATCHMIDDLEBAN_C*M1COFF_POT);//20,此高度也是取馒头的高度
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						else
							{
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
								SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_CATCHMIDDLEBAN_C*M1COFF_POT);//20,此高度也是取馒头的高度
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						
						OPEN_VALVE(TOP_CATCH_VALVE);
						OPEN_VALVE(MID_CATCH_VALVE);
						g_uiTaskTime1 = GetCurTime();
						
					}

				if(g_ucLoopRunFlag == 1)
					{
						// 在此添加循环执行代码

						// 在此判定是否跳转
						if((GetCurTime() - g_uiTaskTime1) >= 300000)
						{
							g_ucJumpRunFlag = 1;
						}
		
					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						/******************************控制水平电机前后指令***************************************/
						 if(KEY_F5(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
						        
						       	stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
			 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid, g_fpActionM1PotFB + g_fpM1DeltaPos2);
							}	

						if(KEY_F4(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
							
								stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathMid, g_fpActionM1PotFB - g_fpM1DeltaPos2);
							 }
					
				/********************************控制竖直电机上下指令*******************************************************/
						if(KEY_F1(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB + g_fpM2DeltaPos2);
									
							}

						if(KEY_F8(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB - g_fpM2DeltaPos2);
								
							}

						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//进行跳转的操作
							stRobot.emRobotTask = TASK11;
						}
					}
				
#endif

				break;
			case TASK11://夹取馒头之后M1M2电机开始运动
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;

					#if 1
					/*******************夹完馒头之后M2电机后撤一段距离********************************************/
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
								   
					//在此添加只执行一次的代码,  要确认各个电机的状态
					if((stRobot.emRobotTactic==RED_FIELD)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
						||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
						||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
						||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
						{
							
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
								 SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_LIFT_BAN*M1COFF_POT);//此高度是放馒头的高度
							       stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								 //OPEN_VALVE(MID_LIFTUP_VALVE);//因为气缸会卡住包山中层，故将此处后移
						}
					else 
						{
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
								 SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_LIFT_BAN*M1COFF_POT);//此高度是放馒头的高度
							       stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
						}
					
				}

				if(g_ucLoopRunFlag == 1)
				{
					// 在此添加循环执行代码


					// 在此判定是否跳转
					if(stAction_M1.fpPotFB>(115*M1COFF_POT))//只要M1电机将馒头提升到离开包山，C车就可以移动
						{
							g_ucJumpRunFlag = 1;
						}
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					/******************************控制水平电机前后指令***************************************/
						 if(KEY_F5(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
						        
						       	stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
			 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid, g_fpActionM1PotFB + g_fpM1DeltaPos2);
							}	

						if(KEY_F4(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
							
								stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathMid, g_fpActionM1PotFB - g_fpM1DeltaPos2);
							 }
					
				/********************************控制竖直电机上下指令*******************************************************/
						if(KEY_F1(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB + g_fpM2DeltaPos2);
									
							}

						if(KEY_F8(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB - g_fpM2DeltaPos2);
								
							}

					// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                                                g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//进行跳转的操作
						stRobot.emRobotTask =  TASK_INSERT_5;
					}
				}
				
				break;
			case TASK_INSERT_5: //用于是否是重启的路径判断，主要是重启方式2的时候使用
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//在此添加只执行一次的代码,  要确认各个电机的状态
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
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_DROPBAN_C*M2COFF_POT);	//在仍馒头之前收回M2电机
						}
					else
						{
						
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
							SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_DROPBAN_C*M2COFF_POT);	//在仍馒头之前收回M2电机
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
					// 在此添加循环执行代码
					
					
							// 在此判定是否跳转
							g_ucJumpRunFlag =1;
						
						
						
				}


				if(g_ucJumpRunFlag == 1)
				{
                                    g_ucLoopRunFlag = 0;
					// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
					if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
					{

						g_ucOnceRunFlag = 0;
						g_ucLoopRunFlag = 1;
						g_ucJumpRunFlag = 0;
                                                g_ucTaskCnt = 0;
						g_uiTaskTime1 = 0;
						g_uiTaskTime2 = 0;
						g_siTaskDeltaTime = 0;
						//进行跳转的操作
						stRobot.emRobotTask = TASK12;
					}
				}
				break;
			
			case TASK12://因为重启路径不一样，需要做一些更改

			#if 1
				 if(g_ucOnceRunFlag == 0)
				{
                                   g_ucOnceRunFlag = 1;
					//在此添加只执行一次的代码,  要确认各个电机的状态
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
						// 在此添加循环执行代码
					
						if((abs(stRobot.stPot.ssPosX)<150)&&(g_CFlag6==0))//将此处的打开阀的操作放到这个地方，使得中层手不会碰到中层包山
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
										 SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_WAIT_M_C*M1COFF_POT);//此高度是放馒头的高度
									       stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
										 //OPEN_VALVE(MID_LIFTUP_VALVE);//因为气缸会卡住包山中层，故将此处后移
								}
							else 
								{
										stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
										 SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_WAIT_M_C*M1COFF_POT);//此高度是放馒头的高度
									       stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								}
							g_CFlag6=1;
					
							}
						if(abs(stRobot.stPot.ssPosX -g_pstReleaseBanPath->ssEndX)<3 && abs(stRobot.stPot.ssPosY -g_pstReleaseBanPath->ssEndY) <4 && abs(stRobot.stPot.ssPosQ-g_pstReleaseBanPath->ssEndQ)<4)
							{
								g_ucTaskCnt++;
							}

						// 在此判定是否跳转
						if(g_ucTaskCnt>=3)
							{
								g_ucJumpRunFlag = 1;
							}

					}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
							{

								g_ucOnceRunFlag = 0;
								g_ucLoopRunFlag = 1;
								g_ucJumpRunFlag = 0;
		                                                g_ucTaskCnt = 0;
								g_uiTaskTime1 = 0;
								g_uiTaskTime2 = 0;
								g_siTaskDeltaTime = 0;
								//进行跳转的操作
								stRobot.emRobotTask = TASK13;
							}
					}
				
				#endif
				break;
			case TASK13://等待M车触发扔馒头
				 if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//在此添加只执行一次的代码,  要确认各个电机的状态
						g_uiTaskTime1=GetCurTime();
						g_siRobotQDes = stRobot.stPot.ssPosQ;
						g_siRobotXDes = stRobot.stPot.ssPosX;
						g_siRobotYDes = stRobot.stPot.ssPosY;					
					}

				if(g_ucLoopRunFlag == 1)
				{

					if(GetCurTime()-g_uiTaskTime1>200000)
						{
					// 在此添加循环执行代码

						
						

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
										// 在此判定是否跳转

										g_ucJumpRunFlag =1;

										
										/********************************改过************************************************************/
										 g_ucAllContinueFlag = 1;
										/******************************************************************************************/
										 
									}
							}
						else 
							{
								if(IS_SWITCH_ON(LEFT_LIGHT_SWITCH)||(JS_R1(JOYSTICK_RESERVED) ))
									{
										// 在此判定是否跳转

										g_ucJumpRunFlag =1;

										
										/********************************改过************************************************************/
										 g_ucAllContinueFlag = 1;
										/******************************************************************************************/
										 
									}
							}
						}

				}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//进行跳转的操作
							stRobot.emRobotTask = TASK14;
							
						}
					}
				
				
				break;
			

	/*******************************my code**********************************************************/
			case TASK14://释放馒头的操作，为后面的初始化动作做准备
				
               		if(g_ucOnceRunFlag == 0)
					{
					
	                                   g_ucOnceRunFlag = 1;
						
						//在此添加只执行一次的代码,  要确认各个电机的状态	
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
								SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_DROPBAN_C*M1COFF_POT);//在仍馒头的时候降低高度，使馒头尽可能的有向下的速度
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						else
							{
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
								SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_DROPBAN_C*M1COFF_POT);//在仍馒头的时候降低高度，使馒头尽可能的有向下的速度
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
						// 在此添加循环执行代码
						if(GetCurTime()-g_uiTaskTime1>300000)
							{
								// 在此判定是否跳转		
								g_ucJumpRunFlag = 1;
							}
								
					}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
							//进行跳转的操作
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
						//在此添加只执行一次的代码,  要确认各个电机的状态
					}

				if(g_ucLoopRunFlag == 1)
					{
						// 在此添加循环执行代码
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

						// 在此判定是否跳转

						if(g_ucMC_InitCnt>g_ucMC_CntTime)
							{
								g_ucJumpRunFlag = 1;
								g_ucAllContinueFlag =1;
							}
					/////////////手动初始化使用//////////////////////////
						if(JS_L1(JOYSTICK_RESERVED))
						{

							g_ucJumpRunFlag = 1;
							g_ucAllContinueFlag =1;
							//进行跳转的操作
						}
						
					}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
							//进行跳转的操作
						}
					}
			
				break;
			#endif
			case TASK_INSERT_6://重启方式4直接去M取C处的入口
				  if(g_ucOnceRunFlag == 0)
					{
	                                   g_ucOnceRunFlag = 1;
						//在此添加只执行一次的代码,  要确认各个电机的状态
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
						// 在此添加循环执行代码
						if(abs(stRobot.stPot.ssPosX -g_pstReleaseBanPath->ssEndX)<3 
							&& abs(stRobot.stPot.ssPosY -g_pstReleaseBanPath->ssEndY) <4 
							&& abs(stRobot.stPot.ssPosQ-g_pstReleaseBanPath->ssEndQ)<4)
							{
								g_ucTaskCnt++;
							}

						// 在此判定是否跳转
						if(g_ucTaskCnt>=3)
							{
								g_ucJumpRunFlag = 1;
							}

						
					}


				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//进行跳转的操作
						}
					}
				
				break;
			case TASK16://打开上面旋转手臂,水平竖直电机开始运动
				if(g_ucOnceRunFlag == 0)
					{
									
						 g_ucOnceRunFlag = 1;
						//在此添加只执行一次的代码,  要确认各个电机的状态
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
				 				SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid,M1_RED_CATCHTOPBAN_C*M1COFF_POT);//竖直电机伸到取顶层馒头的高度
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
										 
								stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
				 				SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_PICKUPTOPBAN_C*M2COFF_POT);//水平电机伸到取顶层馒头的长度
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;	
							}
						else

							{
						
						 		stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
				 				SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid,M1_BLUE_CATCHTOPBAN_C*M1COFF_POT);//竖直电机伸到取顶层馒头的高度
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
										 
								stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
				 				SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_PICKUPTOPBAN_C*M2COFF_POT);//水平电机伸到取顶层馒头的长度
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;	
							}
					}
				

				if(g_ucLoopRunFlag == 1)
					{

						// 在此添加循环执行代码
							/*if((stAction_M2.emPathState==PATH_END)||(GetCurTime()-g_uiTaskTime1>4000000))//防止M2电机出现卡死，故将此处加延时
								{
									stRobot.emNavState=NAV_NULL;
									stRobot.emBaseState=BASE_STOP_QUICKLY;
									
									
									// 在此判定是否跳转
									g_ucJumpRunFlag = 1;
								}*/
						if((IS_SWITCH_OFF(TOP_ROTAIRPOS_SWITCH)||(GetCurTime()-g_uiTaskTime1>1500000)))
							{
								// 在此判定是否跳转
								g_ucJumpRunFlag = 1;
								CLOSE_VALVE(ROT_VALVE);
							}
						

					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//进行跳转的操作
							stRobot.emRobotTask=TASK_INSERT_9;
						
						}
					}
				
				break;
			case TASK_INSERT_9:
				if(g_ucOnceRunFlag == 0)
					{
						g_ucOnceRunFlag = 1;
					//在此添加只执行一次的代码,  要确认各个电机的状态
						g_uiTaskTime1=GetCurTime();
						
					}

				if(g_ucLoopRunFlag == 1)
					{

						// 在此添加循环执行代码
						if(GetCurTime()-g_uiTaskTime1>200000)
							{
								// 在此判定是否跳转
								g_ucJumpRunFlag = 1;
								OPEN_VALVE(ROT_VALVE);
							}

					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{

							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							//进行跳转的操作
							stRobot.emRobotTask=TASK_INSERT_8;
						
						}
					}
				break;
			case TASK_INSERT_8:
				if(g_ucOnceRunFlag == 0)
					{
						g_ucOnceRunFlag = 1;
					//在此添加只执行一次的代码,  要确认各个电机的状态
						g_uiTaskTime1=GetCurTime();
						while(IrComm1IsDataInRxBuf())
						{
							IrComm1GetRxBufDat();
						}
						
					}
				
				if(g_ucLoopRunFlag == 1)
					{
						// 在此添加循环执行代码
						
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
						
								if((abs(stAction_M2.fpPotFB-M2_RED_PICKUPTOPBAN_C*M2COFF_POT)<M2_RED_CATCHTOPBAN_COM_ON_C*M2COFF_POT)||(GetCurTime()-g_uiTaskTime1>4000000))//防止M2电机出现卡死，故将此处加延时
										{
											stRobot.emNavState=NAV_NULL;
											stRobot.emBaseState=BASE_STOP_QUICKLY;
											
											
											// 在此判定是否跳转
											g_ucJumpRunFlag = 1;
										}
							}
						else
							{
						
								if((abs(stAction_M2.fpPotFB-M2_BLUE_PICKUPTOPBAN_C*M2COFF_POT)<M2_BLUE_CATCHTOPBAN_COM_ON_C*M2COFF_POT)||(GetCurTime()-g_uiTaskTime1>4000000))//防止M2电机出现卡死，故将此处加延时
										{
											stRobot.emNavState=NAV_NULL;
											stRobot.emBaseState=BASE_STOP_QUICKLY;
											
											
											// 在此判定是否跳转
											g_ucJumpRunFlag = 1;
										}
							}
											
					}


					if(g_ucJumpRunFlag == 1)
						{
		                                    g_ucLoopRunFlag = 0;
							// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
								//进行跳转的操作
							}
						}
				break;
				
			case TASK17:
				
				if(g_ucOnceRunFlag == 0)
					{
						g_ucOnceRunFlag = 1;
					//在此添加只执行一次的代码,  要确认各个电机的状态

						/*while(IrComm1IsDataInRxBuf())
						{
							IrComm1GetRxBufDat();
						}*/
						
					}
				
				if(g_ucLoopRunFlag == 1)
					{
						// 在此添加循环执行代码
					/******************************控制水平电机前后指令***************************************/
						 if(KEY_F5(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
						        
						       	stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
			 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathMid, g_fpActionM1PotFB + g_fpM1DeltaPos2);
							}	

						if(KEY_F4(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
							
								stAction_M1.pstCurPath = &g_stActionM1DymPathMid;
								g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathMid, g_fpActionM1PotFB - g_fpM1DeltaPos2);
							 }
					
				/********************************控制竖直电机上下指令*******************************************************/
						if(KEY_F1(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
								
								g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
								SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathTop, g_fpActionM2PotFB + g_fpM2DeltaPos2);
									
							}

						if(KEY_F8(usKeyValue))
							{
								 //在此等待按键释放
										        
								while(usKeyValue!= 0xffff)
								{
									 usKeyValue= PSKEY;
								}
								 //在此处理按键后的响应
								
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

				/******************************控制水平电机前后指令***************************************/
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
				/********************************控制竖直电机上下指令*******************************************************/
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

				/*****************************控制抓取馒头指令********************************************/
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

					/********************自动跳转指令*******************************/
						if(g_ucMC_Rot_ValveCnt >= g_ucMC_CntTime)
						{
							g_ucMC_Rot_ValveCnt = 0;
							if(IS_VALVE_OPEN(TOP_CATCH_VALVE))
								{
									// 在此判定是否跳转	
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
							// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
								//进行跳转的操作
							}
						}
				
				break;
			
			case TASK18://取完馒头之后,M1电机上升，将馒头拿出包山
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
 						SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,g_fpActionM2PotFB -40*M2COFF_POT);//水平电机缩短一段距离，防止馒头碰包山
					       stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

						
						g_ucOnceRunFlag = 1;
						
					}

				if(g_ucLoopRunFlag == 1)
					{
						// 在此添加循环执行代码
						//g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
						if(stAction_M1.emPathState==PATH_END)
						//(g_fpActionM2PotFB>200.4043f)
							{
								
							g_ucJumpRunFlag =1;
								
							}
						// 在此判定是否跳转

					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
							//进行跳转的操作
						}
					}
				break;

			case TASK19://暂时不进行任何操作
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
						// 在此添加循环执行代码
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

				/******************************控制水平电机前后指令***************************************/
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
				/********************************控制竖直电机上下指令*******************************************************/
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

				/*****************************控制抓取馒头指令********************************************/
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

					/********************自动跳转指令*******************************/
						if(g_ucMC_AutoCnt >= g_ucMC_CntTime)
						{
							g_ucMC_AutoCnt = 0;
							if(IS_VALVE_OPEN(TOP_CATCH_VALVE))
								{
									// 在此判定是否跳转	
									g_ucJumpRunFlag =1;
								}
							
						}								
						// 在此判定是否跳转
					}

				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
							//进行跳转的操作
						}
					}
				break;
			
				case TASK20://取完馒头，手臂回转到取馒头的位置，并且水平电机前伸
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
						// 在此添加循环执行代码
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
								if(((GetCurTime()-g_uiTaskTime1)>10000000)||(g_ucTaskFlag==1))//此处时间做了更改，防止
								//if((GetCurTime()-g_uiTaskTime1)>1000000)
									{
										stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
		 								SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_DROPTOPBAN_C*M2COFF_POT);//水平电机伸到仍馒头的位置
										stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
										// 在此判定是否跳转
										
										g_ucJumpRunFlag =1;
									}
							}
						else
							{
								if(IS_SWITCH_OFF(TOP_ROTAIRPOS_SWITCH))
									{
										g_ucTaskFlag=1;
										
									}
								if(((GetCurTime()-g_uiTaskTime1)>10000000)||(g_ucTaskFlag==1))//此处时间做了更改，防止
								//if((GetCurTime()-g_uiTaskTime1)>1000000)
									{
										stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
		 								SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_BLUE_DROPTOPBAN_C*M2COFF_POT);//水平电机伸到仍馒头的位置
										stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
										// 在此判定是否跳转
										
										g_ucJumpRunFlag =1;
									}
							}
					}

				if(g_ucJumpRunFlag == 1)
					{
						//进行跳转的操作
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
			case TASK21://延时一段时间之后竖直电机下降
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
		 						SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_RED_DROPTOPBAN_C*M1COFF_POT);//竖直电机下降到仍馒头的位置
								stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						else
							{
								stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
		 						SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,M1_BLUE_DROPTOPBAN_C*M1COFF_POT);//竖直电机下降到仍馒头的位置
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
								// 在此添加循环执行代码
								if(GetCurTime()-g_uiTaskTime1>35000)
									{
										CLOSE_VALVE(ROT_VALVE);
									
									}

								
								if(IS_SWITCH_OFF(TOP_ROTAIRREVERSEPOS_SWITCH))
									{
										g_ucTaskFlag=1;
										OPEN_VALVE(ROT_VALVE);
									}
								if(((GetCurTime()-g_uiTaskTime1)>1500000)||(g_ucTaskFlag==1))//此处时间做了更改，防止
								
									{
										// 在此判定是否跳转
										g_ucJumpRunFlag =1;
									}
								
							}
						else
							{
								// 在此添加循环执行代码
								if(GetCurTime()-g_uiTaskTime1>35000)
									{
										CLOSE_VALVE(ROT_VALVE);
										
									
									}
								if(IS_SWITCH_OFF(TOP_ROTAIRREVERSEPOS_SWITCH))
									{
										g_ucTaskFlag=1;
										OPEN_VALVE(ROT_VALVE);
										
									}
								if(((GetCurTime()-g_uiTaskTime1)>1500000)||(g_ucTaskFlag==1))//此处时间做了更改，防止
								
									{
										
										// 在此判定是否跳转
										g_ucJumpRunFlag =1;
									}
								
							}
								
						
					}
				
				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{
							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							//g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							
							//进行跳转的操作
							stRobot.emRobotTask=TASK_INSERT_10;
						}
					}
				break;
				case TASK_INSERT_10:
					if(g_ucOnceRunFlag == 0)
					{
						g_uiTaskTime1=GetCurTime();
						OPEN_VALVE(ROT_VALVE);
						//进行跳转的操作
						g_ucOnceRunFlag = 1;
						g_ucTaskFlag=0;
						
						
					}
					if(g_ucLoopRunFlag == 1)
					{
						// 在此添加循环执行代码
						if((stRobot.emRobotTactic==RED_FIELD)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
							||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4)
							||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
							||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))

							{
								// 在此添加循环执行代码
								
								if((GetCurTime()-g_uiTaskTime1)>((GetCurTime()-g_uiTaskTime2)*g_fpValveCoff/100.0))//此处时间做了更改，防止
								
									{
										CLOSE_VALVE(ROT_VALVE);
										// 在此判定是否跳转
										g_ucJumpRunFlag =1;
									}
							}
						else
							{
								// 在此添加循环执行代码
								
								if((GetCurTime()-g_uiTaskTime1)>((GetCurTime()-g_uiTaskTime2)*g_fpValveCoff/100.0))//此处时间做了更改，防止
								
									{
										CLOSE_VALVE(ROT_VALVE);
										// 在此判定是否跳转
										g_ucJumpRunFlag =1;
									}
								
							}
							
						
						
					}
				
				if(g_ucJumpRunFlag == 1)
					{
	                                    g_ucLoopRunFlag = 0;
						// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
						if(JS_L1(JOYSTICK_RESERVED) || g_ucAllContinueFlag == 1)
						{
							g_ucOnceRunFlag = 0;
							g_ucLoopRunFlag = 1;
							g_ucJumpRunFlag = 0;
	                                                g_ucTaskCnt = 0;
							g_uiTaskTime1 = 0;
							g_uiTaskTime2 = 0;
							g_siTaskDeltaTime = 0;
							
							//进行跳转的操作
							stRobot.emRobotTask=TASK22;
						}
					}
					break;

				case TASK22:

				if(g_ucOnceRunFlag == 0)
					{
						//进行跳转的操作
						g_ucOnceRunFlag = 1;
						while(IrComm1IsDataInRxBuf())
						{
						     IrComm1GetRxBufDat();
						}
						g_ucMC_ReleaseCnt = 0;
						
					}

				if(g_ucLoopRunFlag == 1)
					{
						// 在此添加循环执行代码

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

						// 在此判定是否跳转
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
							// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
								//进行跳转的操作
							}
						}
					break;
				case TASK23:
					
				if(g_ucOnceRunFlag == 0)
					{
						//进行跳转的操作
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
		 						SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,(M2_RED_DROPTOPBAN_C-50)*M2COFF_POT);//水平电机伸到仍馒头的位置后回撤一段距离防止电机卡死
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						else
							{
								stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
		 						SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,(M2_BLUE_DROPTOPBAN_C-50)*M2COFF_POT);//水平电机伸到仍馒头的位置后回撤一段距离防止电机卡死
								stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
							}
						
						
						
					}

				if(g_ucLoopRunFlag == 1)
					{
						// 在此添加循环执行代码

						if((GetCurTime()-g_uiTaskTime1)>500000)
							{
								// 在此判定是否跳转
							
								g_ucJumpRunFlag =1;
							}
					
					}


					if(g_ucJumpRunFlag == 1)
						{
							
		                                    g_ucLoopRunFlag = 0;
							// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
								//进行跳转的操作
							}
						}
					break;	
				case TASK24:
					if(g_ucOnceRunFlag == 0)
						{
							//进行跳转的操作
							g_ucOnceRunFlag = 1;
							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
	 						SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,15*M1COFF_POT);//
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

						}

					if(g_ucLoopRunFlag == 1)
						{
							// 在此添加循环执行代码

							
							// 在此判定是否跳转
						
							g_ucJumpRunFlag =1;
						
						}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
									//进行跳转的操作
								}
							}
					break;
				case TASK25:
					if(g_ucOnceRunFlag == 0)
					{
						g_ucOnceRunFlag = 1;
					//在此添加只执行一次的代码,  要确认各个电机的状态
						while(IrComm1IsDataInRxBuf())
						{
							IrComm1GetRxBufDat();
						}
						
					}
				
				if(g_ucLoopRunFlag == 1)
					{
						// 在此添加循环执行代码
						
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

				/******************************控制水平电机前后指令***************************************/
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
				/********************************控制竖直电机上下指令*******************************************************/
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

				/*****************************控制抓取馒头指令********************************************/
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
							// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
								//进行跳转的操作
							}
						}
					break;
					case TASK_TOPAIR1:
						if(g_ucOnceRunFlag == 0)
						{
							//进行跳转的操作
							g_ucOnceRunFlag = 1;
							CLOSE_VALVE(ROT_VALVE);
							stAction_M2.pstCurPath = &g_stActionM2DymPathTop;
		 					SetPathEndPosEx(&stAction_M2,&g_stActionM2DymPathTop,M2_RED_PICKUPTOPBAN_C*M2COFF_POT);//水平电机伸到仍馒头的位置后回撤一段距离防止电机卡死
							stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;

							stAction_M1.pstCurPath = &g_stActionM1DymPathTop;
		 					SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathLow,M1_RED_CATCHTOPBAN_C*M1COFF_POT);
							stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
					
						}

						if(g_ucLoopRunFlag == 1)
							{
								// 在此添加循环执行代码

								if((stAction_M2.emPathState==PATH_END)&&(stAction_M1.emPathState==PATH_END))
									{
										// 在此判定是否跳转
									
										g_ucJumpRunFlag =1;
									}
							
							}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
									//进行跳转的操作
								}
							}
						break;
					case TASK_TOPAIR2:
						if(g_ucOnceRunFlag == 0)
							{
								//进行跳转的操作
								g_ucOnceRunFlag = 1;
								OPEN_VALVE(ROT_VALVE);
				
							}

						if(g_ucLoopRunFlag == 1)
							{
								// 在此添加循环执行代码

								if(IS_SWITCH_OFF(TOP_ROTAIRPOS_SWITCH))
									{
										// 在此判定是否跳转
									
										g_ucJumpRunFlag =1;
										g_ucAllContinueFlag = 1;
										CLOSE_VALVE(ROT_VALVE);
									}
							}

						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
									//进行跳转的操作
								}
							}
						break;
						case TASK_TOPAIR7:
							if(g_ucOnceRunFlag == 0)
							{
								//进行跳转的操作
								g_ucOnceRunFlag = 1;
								g_uiTaskTime1=GetCurTime();
								
				
							}

						if(g_ucLoopRunFlag == 1)
							{
								// 在此添加循环执行代码

								if(GetCurTime()-g_uiTaskTime1>200000)
									{
										// 在此判定是否跳转
										OPEN_VALVE(ROT_VALVE);
									
										g_ucJumpRunFlag =1;
										g_ucAllContinueFlag = 0;
									}
							
							}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
									//进行跳转的操作
								}
							}
							break;
						case TASK_TOPAIR3:
						if(g_ucOnceRunFlag == 0)
							{
								//进行跳转的操作
								g_ucOnceRunFlag = 1;
								CLOSE_VALVE(ROT_VALVE);

								g_uiTaskTime2 = GetCurTime();
							}

						if(g_ucLoopRunFlag == 1)
							{
								// 在此添加循环执行代码
								

								if(IS_SWITCH_OFF(TOP_ROTAIRPOS_SWITCH)) 
									{
										// 在此判定是否跳转
									
										g_ucJumpRunFlag =1;
										
										g_ucAllContinueFlag = 1;
									}
							
							}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
									//进行跳转的操作
								}
							}
							break;
						case TASK_TOPAIR4:
						if(g_ucOnceRunFlag == 0)
							{
								//进行跳转的操作
								g_ucOnceRunFlag = 1;
								g_uiTaskTime1=GetCurTime();
								//OPEN_VALVE(ROT_VALVE);
							}

						if(g_ucLoopRunFlag == 1)
							{
								// 在此添加循环执行代码

								//if(GetCurTime()-g_uiTaskTime1>25000)

									{
										CLOSE_VALVE(ROT_VALVE);
										// 在此判定是否跳转
										
										g_ucJumpRunFlag =1;
										g_ucAllContinueFlag =1;
									}
							}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
									//进行跳转的操作
								}
							}
							break;
						case TASK_TOPAIR5:
						if(g_ucOnceRunFlag == 0)
							{
								//进行跳转的操作
								g_ucOnceRunFlag = 1;
								CLOSE_VALVE(ROT_VALVE);
							}

						if(g_ucLoopRunFlag == 1)
							{
								// 在此添加循环执行代码

								if(IS_SWITCH_OFF(TOP_ROTAIRREVERSEPOS_SWITCH)) 
									{
										// 在此判定是否跳转
									
										g_ucJumpRunFlag =1;
										g_TimeTemp=GetCurTime()-g_uiTaskTime2;
										g_ucAllContinueFlag = 1;
									}
							
							}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
									//进行跳转的操作
								}
							}
							break;
						case TASK_TOPAIR6:
						if(g_ucOnceRunFlag == 0)
							{
								//进行跳转的操作
								g_ucOnceRunFlag = 1;
								g_uiTaskTime1=GetCurTime();
								OPEN_VALVE(ROT_VALVE);
							}

						if(g_ucLoopRunFlag == 1)
							{
								// 在此添加循环执行代码

								if(GetCurTime()-g_uiTaskTime1>(g_TimeTemp*g_fpValveCoff/100.0))//120

									{
										CLOSE_VALVE(ROT_VALVE);
										// 在此判定是否跳转
										
										g_ucJumpRunFlag =1;
										g_ucAllContinueFlag =0;
									}
							}


						if(g_ucJumpRunFlag == 1)
							{
								
			                                    g_ucLoopRunFlag = 0;
								// 在此添加跳转代码在此要清除RunFlag、清除计数和定时,并且作单步及连续跳转判定
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
									//进行跳转的操作
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
任务名称：NavTask ()
任务功能：导航任务
****************************************************************************************************/
void NavTask (void *pdata)
{
//	stRobot.pstCurPath = (ST_PATH*) &stNavPath[7][0];
	stRobot.emNavState = NAV_OFF;//手动控制

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
				//进入该该状态应当确定好路径，并将路径的状态设置为PATH_RUN
					NavLineEx1(&stRobot);
					stRobot.emBaseState = BASE_CLOSE_LOOP_CTRL;
					break;
				

				case NAV_BASE_DIRECT:
					// 什么也不用做，在该状态中，直接修改三个轮子的速度
					stRobot.emBaseState = BASE_CLOSE_LOOP_CTRL;
					break;
				case NAV_XYQ:
					// 在此状态中，如果使能为1，则修改g_siRobot*DesShadow,单次 清除调用的pid的值，
					//使能位为2时，在外部修改VeltDes,能够实现
					//为0时，预期速度为0
					if(g_ucRobotXEnable == 1)
						{
							CopyIntSlowly(&g_siRobotXDes, &g_siRobotXDesShadow, 1);
							stRobot.stNavPidLine.stPidTrvs.fpE = g_siRobotXDes - stRobot.stPot.ssPosX;
							CalPosPID(&stRobot.stNavPidLine.stPidTrvs);
							stRobot.stVeltDes.fpVx = stRobot.stNavPidLine.stPidTrvs.fpU;
						}
					else if(g_ucRobotXEnable == 2)
						{
							// 什么都不用写，在外部修改  X 方向的速度,实现速度分配，可以在此形成斜坡信号
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
							// 什么都不用写，在外部修改  Y 方向的速度，实现速度分配
						}
					else
						{
							stRobot.stVeltDes.fpVy = 0;
						}
						

					if(g_ucRobotQEnable == 1)
						{
							CopyIntSlowly(&g_siRobotQDes,&g_siRobotQDesShadow,1);//原来是2，现在改为1
							stRobot.stNavPidLine.stPidRot.fpE = g_siRobotQDes - stRobot.stPot.ssPosQ;
							CalPosPID(&stRobot.stNavPidLine.stPidRot);
							stRobot.stVeltDes.fpW= stRobot.stNavPidLine.stPidRot.fpU;
						}
					else if(g_ucRobotQEnable == 2)
						{
							// 什么都不用写，在外部修改  Y 方向的速度，实现速度分配
						}
					else
						{
							stRobot.stVeltDes.fpW = 0;
						}
						//在此分配速度
						AutoBaseVeltAllocate(&stRobot, &stRobot.stVeltDes, 0);

						stRobot.emBaseState = BASE_CLOSE_LOOP_CTRL;
					break;
			       case NAV_NULL:
				   	
				   	//什么也不做，避免修改地盘的状态，导航状态为空
				   	break;
				
				default:
					stRobot.emBaseState = BASE_BREAK;				
					break;
			}
	 
	#if 0
		DoubleVerticalOMNILocate(&stDoubleOMNI, &stRobot);
	//	DoubleVerticalOMNILocate(&stDoubleOMNI, &stRobot);

		if(stRobot.emState == ROBOT_MOVE)//机器人启动
		{
			switch(stRobot.emNavState)
			{
				case NAV_MANUAL:
					ManualCtrl(&stRobot, &stRobotDesV);//开环
					stRobot.emBaseState = BASE_CLOSE_LOOP_CTRL;
					break;
				case NAV_ERROR:
					stRobot.emBaseState = BASE_BREAK;
					break;
				case NAV_OFF:
					stRobot.emBaseState = BASE_STOP;
					break;
				default://自动导航
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
任务名称:对底盘的电机进行控制
任务
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
				//约在400ms内停止
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
任务名称：ActionMotorCtrlTask()
任务功能：执行电机PID控制
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
             //在位置闭环中，已经存在计算速度，连续算两次速度误差太大
             //只调用VeltLoopCtrl的函数需要调用计算速度函数CalVelt
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
	//刚进入该模式时，完成各项初始化操作
	switch(ucMode)
           {
                      case C_MODE_INIT_0:
/*---------------初始化模式，用于选择后续的模式-----------------------*/

                            stRobot.emRobotTask = ALL_BREAK;
                            g_ucCurLcdPage = 0;
				break;
			case C_MODE_TEST_SENSOR_1:
/*---------------测试陀螺、开关、气动、手柄-----------------------------*/
                             g_ucCurLcdPage = 1;
                            stRobot.emRobotTask = ALL_BREAK;
				break;
			case C_MODE_TEST_CODER_2:
/*---------------测试码盘---------------------------------------------------------*/
                           g_ucCurLcdPage = 2;
                            stRobot.emRobotTask = ALL_BREAK;

				break;
			case C_MODE_TEST_LINE_3:
/*---------------测试巡线---------------------------------------------------------*/
                            g_ucCurLcdPage = 3;
                            stRobot.emRobotTask = ALL_BREAK;

				break;
			case C_MODE_CAL_GYRO_4:
/*---------------标定陀螺---------------------------------------------------------*/
                       g_ucCurLcdPage = 4;
                       stRobot.emRobotTask =NO_ACTION;
			  g_siDirAngle = 0;
			  
			  stAction_M1.emState = MOTOR_BREAK;
			  stAction_M2.emState = MOTOR_BREAK;
			  stAction_M3.emState = MOTOR_BREAK;
			  
			  stRobot.emNavState = NAV_MANUAL;

				break;
			case C_MODE_MANUAL_5:
/*---------------手动控制---------------------------------------------------------*/

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
/*---------------自动低速模式---------------------------------------------------*/
			
                      g_ucCurLcdPage = 6;
/***************************************加载L2的路径**********************************************************/
			if((stRobot.emRobotTactic==RED_FIELD)
				||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
				
				{
					g_pstL2Path=&stNavPathRed[0];//从A车上岛正常路径
				}
			else if((stRobot.emRobotTactic==BLUE_FIELD)
				||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART))
				
				{
					g_pstL2Path=&stNavPathBlue[0];//从A车上岛正常路径
				}
			
			else if((stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4))
				{
					g_pstL2Path=&stNavPathRed[9];//重启路径
				}

			else if((stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART4))
				{
					g_pstL2Path=&stNavPathBlue[9];//重启路径
				}
/*************************************************************************************************************/


/**********************************加载岛上的一些路径，包括抓取和释放馒头***************************************************************************/			

		/*********************************红场路径的加载****************************************************/
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
		/**********************************蓝场路径的加载**************************************************************************/
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
/*---------------自动中速模式---------------------------------------------------*/
                      g_ucCurLcdPage = 7;

/***************************************加载L2的路径**********************************************************/
			if((stRobot.emRobotTactic==RED_FIELD)
				||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
				{
					g_pstL2Path=&stNavPathRed[3];//从A车上岛正常路径
				}
			else if((stRobot.emRobotTactic==BLUE_FIELD)
				||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART))
				{
					g_pstL2Path=&stNavPathBlue[3];//从A车上岛正常路径
				}

			else if((stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4))
				{
					g_pstL2Path=&stNavPathRed[9];//重启路径
				}
			else if((stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART4))
				{
					g_pstL2Path=&stNavPathBlue[9];//重启路径
				}
/****************************************************************************************************************/

/**********************************加载岛上的一些路径，包括抓取和释放馒头***************************************************************************/			

		/*********************************红场路径的加载****************************************************/		
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


/**********************************蓝场路径的加载**************************************************************************/
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
/*---------------自动高速模式---------------------------------------------------*/
                   /***************************************加载L2的路径**********************************************************/
			if((stRobot.emRobotTactic==RED_FIELD)
				||(stRobot.emRobotTactic==RED_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==RED_FIELD_A_RESTART))
				{
					g_pstL2Path=&stNavPathRed[6];//从A车上岛正常路径
				}
			else if((stRobot.emRobotTactic==BLUE_FIELD)
				||(stRobot.emRobotTactic==BLUE_FIELD_S2_RESTART)
				||(stRobot.emRobotTactic==BLUE_FIELD_A_RESTART))
				{
					g_pstL2Path=&stNavPathBlue[6];//从A车上岛正常路径
				}

			else if((stRobot.emRobotTactic==RED_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART2)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART3)
				||(stRobot.emRobotTactic==RED_FIELD_L2_RESTART4))
				{
					g_pstL2Path=&stNavPathRed[9];//重启路径
				}
			else if((stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART1)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART2)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART3)
				||(stRobot.emRobotTactic==BLUE_FIELD_L2_RESTART4))
				{
					g_pstL2Path=&stNavPathBlue[9];//重启路径
				}
/****************************************************************************************************************/

/**********************************加载岛上的一些路径，包括抓取和释放馒头***************************************************************************/			

		/*********************************红场路径的加载****************************************************/		
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

/**********************************蓝场路径的加载**************************************************************************/
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
任务名称：ReadKeyTask()
任务功能：读取手柄/键盘键值以及一些开关量值
****************************************************************************************************/
void ReadKeyTask (void *pdata)
{
       stJSValue.usJsLeft = 0x8080;
	stJSValue.usJsRight = 0x8080;
	OSTimeDly(500); //延迟，消除上电瞬间的不确定状态
	StopMEMSGryo();
	StartMEMSGryo();
	while(1)
	{     
		usKeyValue=PSKEY;
		stJSValue.usReserved = JOYSTICK_RESERVED;
		stJSValue.usJsState = JOYSTICK_STATE;

		if(stJSValue.usJsState == 0x73) //模拟键按下才更新手柄模拟值
		{
		     stJSValue.usJsLeft = JOYSTICK_LEFT;
		     stJSValue.usJsRight = JOYSTICK_RIGTH;
		}
		else
		{
			stJSValue.usJsLeft = 0x8080;
	              stJSValue.usJsRight = 0x8080;
		}


		/*****************************用于在走场时控制上下手的气缸*******************************/
		  if((stJSValue.usJsState == 0x73)&&JS_R2(stJSValue.usReserved))
		  	{
		  		 //在此等待按键释放
				while(stJSValue.usReserved != 0xffff)
					{
						stJSValue.usReserved = JOYSTICK_RESERVED;
					}
				//在此处理按键后的响应
						         
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
		  
//     翻页键，系统保留，与模式无关
//     急停键，与模式无关
		 if(stJSValue.usReserved != 0xffff)
		 {
		 	 if(JS_L2(stJSValue.usReserved))
			   {
				   //在此等待按键释放
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
				   //在此等待按键释放
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

/************************************底盘退出闭环**********************************************************/

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

/************************************底盘进行闭环**********************************************************/
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
					 //在此等待按键释放
							        
					while(usKeyValue!= 0xffff)
					{
						 usKeyValue= PSKEY;
					}
					g_SelectState_Electric_On=1;	
				}	

 /***********************************加载flash键***************************************************************************/
  		if(KEY_N7(usKeyValue))
				{
					//在此等待按键释放
						        
					 while(usKeyValue!= 0xffff)
						{
							usKeyValue= PSKEY;
						}
					 g_ucFlashLoad=1;
			  	}
/***************************************************************************************************************/
/***********************************红蓝场选择键***************************************************************************/
#if 1
  		if(KEY_F2(usKeyValue))
				{
					//在此等待按键释放
						        
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
/*---------------初始化模式，用于选择后续的模式--手柄的模式---------------------*/
                     if((stJSValue.usReserved != 0xffff)||(usKeyValue!=0xffff))
		          {
			        //在此添加对应按键的处理
			         if(JS_UP(stJSValue.usReserved))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
						
					//在此处理按键后的响应
					         g_ucModeCnt++;
					         if(g_ucModeCnt >= g_ucTotalMode)
					         {
							 	 g_ucModeCnt = 0;
					         }
					        
				         }

					 if(KEY_F5(usKeyValue))
					         {
						        //在此等待按键释放
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//在此处理按键后的响应
						         g_ucModeCnt++;
						         if(g_ucModeCnt >= g_ucTotalMode)
						         {
								 	 g_ucModeCnt = 7;
						         }
						        
					         }
				 if(JS_DOWN(stJSValue.usReserved))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					
					//在此处理按键后的响应
					        if(g_ucModeCnt == 0)
					        {
							g_ucModeCnt = g_ucTotalMode-1;	
					        }
						 else
						 {
						 	g_ucModeCnt--;
						 }
					         
				         }

				 if(g_ucKeyFlag==0)//启动红场按键
				 	{

						  if(JS_B1(stJSValue.usReserved)||KEY_N1(usKeyValue))
						         {
							        //在此等待按键释放
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
								while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							      //在此处理按键后的响应
							      stRobot.emRobotTactic = RED_FIELD_L2_RESTART1;
							      
							         
						         }
						  

						  if(JS_B2(stJSValue.usReserved)||KEY_N2(usKeyValue))
						         {
							        //在此等待按键释放
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
								while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							      //在此处理按键后的响应
							       stRobot.emRobotTactic = RED_FIELD_L2_RESTART2;
							      
							         
						         }



						  if(JS_B3(stJSValue.usReserved)||KEY_N3(usKeyValue))
						         {
							        //在此等待按键释放
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
								while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							      //在此处理按键后的响应
							       stRobot.emRobotTactic =RED_FIELD_L2_RESTART3;
							      
							         
						         }
						  


						  if(JS_B4(stJSValue.usReserved)||KEY_N4(usKeyValue))
						         {
							        //在此等待按键释放
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
								while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							      //在此处理按键后的响应
							      stRobot.emRobotTactic =RED_FIELD_L2_RESTART4;
							         
						         }

						   if(JS_LEFT(stJSValue.usReserved)||KEY_N0(usKeyValue))
						         {
							        //在此等待按键释放
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //在此处理按键后的响应
							      stRobot.emRobotTactic =RED_FIELD;
							         
						         }
						     if(JS_RIGHT(stJSValue.usReserved)||KEY_N5(usKeyValue))
						         {
							        //在此等待按键释放
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //在此处理按键后的响应
							      stRobot.emRobotTactic =RED_FIELD_S2_RESTART;
							         
						         }

							 if(JS_L1(stJSValue.usReserved)||KEY_N6(usKeyValue))
						         {
							        //在此等待按键释放
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //在此处理按键后的响应
							      stRobot.emRobotTactic =RED_FIELD_A_RESTART;
							         
						         }
							  if(JS_R1(stJSValue.usReserved))
						         {
							        //在此等待按键释放
							     
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //在此处理按键后的响应
							      stRobot.emRobotTactic =TEST_TOPAIR;
							         
						         }
						     
				 	}



			
				else  if(g_ucKeyFlag==1)//启动蓝场按键
				 	{

						  if(JS_B1(stJSValue.usReserved)||KEY_N1(usKeyValue))
						         {
							        //在此等待按键释放
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //在此处理按键后的响应
							      stRobot.emRobotTactic = BLUE_FIELD_L2_RESTART1;
							      
							         
						         }
						  

						  if(JS_B2(stJSValue.usReserved)||KEY_N2(usKeyValue))
						         {
							        //在此等待按键释放
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //在此处理按键后的响应
							       stRobot.emRobotTactic =BLUE_FIELD_L2_RESTART2;
							      
							         
						         }



						  if(JS_B3(stJSValue.usReserved)||KEY_N3(usKeyValue))
						         {
							        //在此等待按键释放
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //在此处理按键后的响应
							       stRobot.emRobotTactic =BLUE_FIELD_L2_RESTART3;
							      
							         
						         }
						  


						  if(JS_B4(stJSValue.usReserved)||KEY_N4(usKeyValue))
						         {
							        //在此等待按键释放
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //在此处理按键后的响应
							      stRobot.emRobotTactic =BLUE_FIELD_L2_RESTART4;
							         
						         }

						   if(JS_LEFT(stJSValue.usReserved)||KEY_N0(usKeyValue))
						         {
							        //在此等待按键释放
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //在此处理按键后的响应
							      stRobot.emRobotTactic =BLUE_FIELD;
							         
						         }
						     if(JS_RIGHT(stJSValue.usReserved)||KEY_N5(usKeyValue))
						         {
							        //在此等待按键释放
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //在此处理按键后的响应
							      stRobot.emRobotTactic =BLUE_FIELD_S2_RESTART;
							         
						         }

							 if(JS_L1(stJSValue.usReserved)||KEY_N6(usKeyValue))
						         {
							        //在此等待按键释放
							        while(usKeyValue!= 0xffff)
							        {
								        usKeyValue= PSKEY;
							        }
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //在此处理按键后的响应
							      stRobot.emRobotTactic =BLUE_FIELD_A_RESTART;
							         
						         }
							   if(JS_R1(stJSValue.usReserved))
						         {
							        //在此等待按键释放
							     
							        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							      //在此处理按键后的响应
							      stRobot.emRobotTactic =TEST_TOPAIR;
							         
						         }
						     
				 	}

				 

				  if(JS_SELECT(stJSValue.usReserved)||KEY_F6(usKeyValue))
				         {
					        //在此等待按键释放
					        while(usKeyValue!= 0xffff)
						{
							usKeyValue= PSKEY;
						 }
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//在此处理按键后的响应
					       g_ucCurModeSel = g_ucModeCnt;
					       InitMode(g_ucCurModeSel);
	     
				         }
				 
				
			 
                         	}	 
                       
				break;
			case C_MODE_TEST_SENSOR_1:
/*---------------测试陀螺、开关、气动、手柄-----------------------------*/
                         if(stJSValue.usReserved != 0xffff)
			          {
				        //在此添加对应按键的处理
				         if(JS_L1(stJSValue.usReserved))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
						StartMEMSGryo();
				         }



					 if(JS_R1(stJSValue.usReserved))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
						StopMEMSGryo();
				         }
				         if(JS_UP(stJSValue.usReserved))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//在此处理按键后的响应
					         
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
						        //在此等待按键释放
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//在此处理按键后的响应
						         
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
						        //在此等待按键释放
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//在此处理按键后的响应
						         
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
						        //在此等待按键释放
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//在此处理按键后的响应
						         
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
						        //在此等待按键释放
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//在此处理按键后的响应
						         
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
						        //在此等待按键释放
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//在此处理按键后的响应
						         
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
						        //在此等待按键释放
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//在此处理按键后的响应
						         
						        if(IS_VALVE_CLOSE(MID_LIFTUP_VALVE))
						        {
								OPEN_VALVE(MID_LIFTUP_VALVE);
						        }
							else
							 {
							       CLOSE_VALVE(MID_LIFTUP_VALVE);
							 }
					         }

					
                        
	/************************************测试与A车通信*********************************************/

				           if(JS_B4(stJSValue.usReserved))
					         {
						        //在此等待按键释放
						       
						        //在此处理按键后的响应
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

	/************************************测试气缸动作频率*************************************************************/
					  if(JS_B2(stJSValue.usReserved))
					         {
						        //在此等待按键释放
						     while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//在此处理按键后的响应
						        SaveOneWord32(stRobot.stPot.ssPosX, 0);
						  
					         }
					     
		/**********************************************************************************************************/
					
                         	}
				
				
				break;
			case C_MODE_TEST_CODER_2:
/*---------------测试码盘---------------------------------------------------------*/

				break;
			case C_MODE_TEST_LINE_3:
/*---------------测试巡线---------------------------------------------------------*/
                        if(stJSValue.usReserved != 0xffff)
		          {
			        //在此添加对应按键的处理
			         if(JS_UP(stJSValue.usReserved))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//在此处理按键后的响应
					         
					        CALIBRATE_WHITE_LINE(LINE_CHAN_0);
					        PutCur(1,0);
						 PrintChar("CalWhite        ");
				         }

			         //在此添加对应按键的处理
			         if(JS_DOWN(stJSValue.usReserved))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//在此处理按键后的响应
					         
					        CALIBRATE_BLACK_LINE(LINE_CHAN_0);
					        PutCur(1,0);
						 PrintChar("CalBlack        ");
				         }

					  //在此添加对应按键的处理
			         if(JS_RIGHT(stJSValue.usReserved))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//在此处理按键后的响应
					         
					        CALIBRATE_RED_LINE(LINE_CHAN_0);
					        PutCur(1,0);
						 PrintChar("CalRed        ");
				         }


			         //在此添加对应按键的处理
			         if(JS_LEFT(stJSValue.usReserved))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//在此处理按键后的响应
					         
					        CALIBRATE_BLUE_LINE(LINE_CHAN_0);
					        PutCur(1,0);
						 PrintChar("CalBlue        ");
				         }


				 	  //在此添加对应按键的处理
			         if(JS_B1(stJSValue.usReserved))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					        //在此处理按键后的响应
					         
					        USE_RED_FIELD_CFG(LINE_CHAN_0);
						  PutCur(1,0);
						 PrintChar("UseRed          ");
				         }
				  if(JS_B2(stJSValue.usReserved))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					        //在此处理按键后的响应
					         
					        USE_BLUE_FIELD_CFG(LINE_CHAN_0);
						  PutCur(1,0);
						 PrintChar("UseBlue         ");
				         }

				    if(JS_B3(stJSValue.usReserved))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					        //在此处理按键后的响应
					         
					        USE_BLACK_FIELD_CFG(LINE_CHAN_0);
						 PutCur(1,0);
						 PrintChar("UseBlack         ");
				         }

				   if(JS_B4(stJSValue.usReserved))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					        //在此处理按键后的响应
					         
					        CONFIG_TO_LINE(LINE_CHAN_0);
						 PutCur(1,0);
						 PrintChar("ConfigToLine     ");
				         }

			         if(JS_L1(stJSValue.usReserved))
			         {
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //在此处理按键后的响应
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
/*---------------标定陀螺---------------------------------------------------------*/
                        if(stJSValue.usReserved != 0xffff)
		          {
			        //在此添加对应按键的处理
			         if(JS_UP(stJSValue.usReserved))
			         {
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				//在此处理按键后的响应
				         g_siDirAngle += 50;
			         }

			         //在此添加对应按键的处理
			         if(JS_DOWN(stJSValue.usReserved))
			         {
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				//在此处理按键后的响应
				         g_siDirAngle -= 50;
			         }


				 //在此添加对应按键的处理
			         if(JS_LEFT(stJSValue.usReserved))
			         {
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				//在此处理按键后的响应
				         g_siDirAngle -= 10;
			         }


                             //在此添加对应按键的处理
			         if(JS_RIGHT(stJSValue.usReserved))
			         {
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				//在此处理按键后的响应
				         g_siDirAngle += 10;
			         }

			          //在此添加对应按键的处理
			         if(JS_B1(stJSValue.usReserved))
			         {
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				//在此处理按键后的响应
				         g_siDirAngle += 1;
			         }

			        //在此添加对应按键的处理
			         if(JS_B3(stJSValue.usReserved))
			         {
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				//在此处理按键后的响应
				         g_siDirAngle  -= 1 ;
			         }
					 
                        }

				break;
			case C_MODE_MANUAL_5:
/*---------------手动控制---------------------------------------------------------*/
                
                       if(stJSValue.usReserved != 0xffff)
		          {
			        //在此添加对应按键的处理
			         if(JS_UP(stJSValue.usReserved))
			         {
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //在此处理按键后的响应
				        stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
				        g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
					 SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathLow, g_fpActionM1PotFB + g_fpM1DeltaPos);
				       
			         }

				  //在此添加对应按键的处理
			         if(JS_DOWN(stJSValue.usReserved))
			         {
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //在此处理按键后的响应
				        stAction_M1.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;//竖直提升电机
				        g_fpActionM1PotFB = stAction_M1.fpCoffPot * READ_CODER(stAction_M1.ucCoderChan);
					 SetPathEndPosEx(&stAction_M1, &g_stActionM1DymPathLow, g_fpActionM1PotFB -g_fpM1DeltaPos);
				       
			         }

			        //在此添加对应按键的处理
			         if(JS_B3(stJSValue.usReserved))
			         {
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //在此处理按键后的响应
				        stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;//顶部水平电机
				        g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
					 SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathLow, g_fpActionM2PotFB - g_fpM2DeltaPos);
				       
			         }

				 //在此添加对应按键的处理
			         if(JS_B1(stJSValue.usReserved))
			         {
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //在此处理按键后的响应
				        stAction_M2.emState = MOTOR_PATH_CLOSE_LOOP_CTRL;
				        g_fpActionM2PotFB = stAction_M2.fpCoffPot * READ_CODER(stAction_M2.ucCoderChan);
					 SetPathEndPosEx(&stAction_M2, &g_stActionM2DymPathLow, g_fpActionM2PotFB + g_fpM2DeltaPos);
				       
			         }

				   //在此添加对应按键的处理
			         if(JS_LEFT(stJSValue.usReserved))
			         {
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //在此处理按键后的响应
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
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //在此处理按键后的响应
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
				        //在此等待按键释放
				        while(stJSValue.usReserved != 0xffff)
				        {
					        stJSValue.usReserved = JOYSTICK_RESERVED;
				        }
				        //在此处理按键后的响应
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
/*---------------自动低速模式---------------------------------------------------*/
                    
				
				 if(stJSValue.usReserved != 0xffff)
		          	{
				        //在此添加对应按键的处理
				         if(JS_START(stJSValue.usReserved)&&(g_FirstRun==0))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }

						g_FirstRun=1;
					//在此处理按键后的响应
					         
					     switch(stRobot.emRobotTactic)
							{
								case  RED_FIELD:
									 stRobot.emRobotTask =  INIT;
									break;
								/*****状态还未加*************/	
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


								/*****状态还未加*************/	
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
					   //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					       g_fpValveCoff+=1;
			 	 }

				 if(JS_DOWN(stJSValue.usReserved))
					   {
						   //在此等待按键释放
						        while(stJSValue.usReserved != 0xffff)
							        {
								        stJSValue.usReserved = JOYSTICK_RESERVED;
							        }
							
						  g_fpValveCoff-=1;
				 	 }


/*************************************选择自动和非自动模式*****************************************************/
				  if(stJSValue.usReserved != 0xffff)
			          	{
					        //在此添加对应按键的处理
					         if(JS_B1(stJSValue.usReserved))
					         {
						        //在此等待按键释放
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//在此处理按键后的响应
						         
						     g_ucAllContinueFlag =1;
					         }
	                     	}

				    if(stJSValue.usReserved != 0xffff)
			          	{
					        //在此添加对应按键的处理
					         if(JS_B3(stJSValue.usReserved))
					         {
						        //在此等待按键释放
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//在此处理按键后的响应
						         
						     g_ucAllContinueFlag =0;
					         }
	                     	}

					if(usKeyValue!= 0xffff)
			          	{
					        //在此添加对应按键的处理
					         if(KEY_F7(PSKEY))
					         {
						        //在此等待按键释放
						        while(usKeyValue!= 0xffff)
						        {
							        usKeyValue = PSKEY;
						        }
						//在此处理按键后的响应
						
						    	g_ucAllContinueFlag =1;
							g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标

							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;
							stRobot.stPot.ssPosX=0;
							stRobot.stPot.ssPosY=0;
							stRobot.stPot.ssPosQ=0;
							
					         }
	                     	}
	

				break;
			case C_MODE_MID_SPEED_7:
/*---------------自动中速模式---------------------------------------------------*/
			  if((stJSValue.usReserved != 0xffff)||(usKeyValue!= 0xffff))
		          	{
				        //在此添加对应按键的处理
				         if((JS_START(stJSValue.usReserved)||KEY_F6(usKeyValue))&&(g_FirstRun==0))
				         {
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
						 while(usKeyValue!= 0xffff)
						  {
							 usKeyValue= PSKEY;
						  }
						g_FirstRun=1;
					//在此处理按键后的响应
					         
					    switch(stRobot.emRobotTactic)
							{
								case  RED_FIELD:
									 stRobot.emRobotTask =  INIT;
									break;

								
								/*****状态还未加*************/	
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
								
								/*****状态还未加*************/	
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


/*******************************选择自动和非自动模式***************/
				  if(stJSValue.usReserved != 0xffff)
			          	{
					        //在此添加对应按键的处理
					         if(JS_B1(stJSValue.usReserved))
					         {
						        //在此等待按键释放
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//在此处理按键后的响应
						         
						     g_ucAllContinueFlag =1;
					         }
							 
	                     	}
				  
					if(usKeyValue!= 0xffff)
			          	{
					        //在此添加对应按键的处理
					         if(KEY_F7(PSKEY))
					         {
						        //在此等待按键释放
						        while(usKeyValue!= 0xffff)
						        {
							        usKeyValue = PSKEY;
						        }
						//在此处理按键后的响应
						
						    	g_ucAllContinueFlag =1;
							g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标

							stDoubleOMNI.fpLastPosX=0;
							stDoubleOMNI.fpLastPosY=0;
							stRobot.stPot.ssPosX=0;
							stRobot.stPot.ssPosY=0;
							stRobot.stPot.ssPosQ=0;
							
					         }
	                     	}

				    if(stJSValue.usReserved != 0xffff)
			          	{
					        //在此添加对应按键的处理
					         if(JS_B3(stJSValue.usReserved))
					         {
						        //在此等待按键释放
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//在此处理按键后的响应
						         
						     	g_ucAllContinueFlag =0;
							
					         }
	                     	}

				break;
			case C_MODE_TOP_SPEED_8:
/*---------------自动高速模式---------------------------------------------------*/
  			if((stJSValue.usReserved != 0xffff)||(usKeyValue!= 0xffff))
		          	{
				        //在此添加对应按键的处理
				         if((JS_START(stJSValue.usReserved)||KEY_F6(usKeyValue))&&(g_FirstRun==0))
				         {
				         	g_FirstRun=1;
					        //在此等待按键释放
					        while(stJSValue.usReserved != 0xffff)
					        {
						        stJSValue.usReserved = JOYSTICK_RESERVED;
					        }
					//在此处理按键后的响应

						switch(stRobot.emRobotTactic)
							{
								case  RED_FIELD:
									 stRobot.emRobotTask =  INIT;
									break;
								
								/*****状态还未加*************/	
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


/*******************************选择自动和非自动模式***************/
				  if(stJSValue.usReserved != 0xffff)
			          	{
					        //在此添加对应按键的处理
					         if(JS_B1(stJSValue.usReserved))
					         {
						        //在此等待按键释放
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//在此处理按键后的响应
						         
						     g_ucAllContinueFlag =1;
					         }
	                     	}

				    if(stJSValue.usReserved != 0xffff)
			          	{
					        //在此添加对应按键的处理
					         if(JS_B3(stJSValue.usReserved))
					         {
						        //在此等待按键释放
						        while(stJSValue.usReserved != 0xffff)
						        {
							        stJSValue.usReserved = JOYSTICK_RESERVED;
						        }
						//在此处理按键后的响应
						         
						     g_ucAllContinueFlag =0;
					         }
	                     	}

					if(usKeyValue!= 0xffff)
			          	{
					        //在此添加对应按键的处理
					         if(KEY_F7(PSKEY))
					         {
						        //在此等待按键释放
						        while(usKeyValue!= 0xffff)
						        {
							        usKeyValue = PSKEY;
						        }
						//在此处理按键后的响应
						
						    	g_ucAllContinueFlag =1;
							g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标

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

             if(CAN1IsDataInTxBuf())//队列里有数就发送
              {
			  	CAN1BeginSend();
              }

	      if(CAN2IsDataInTxBuf())//队列里有数就发送
              {
			  	CAN2BeginSend();
              }
		  
		while(CAN2IsDataInRxBuf())//如果CAN2 接受队列里有数据进行处理,并全部处理完成
		{
			RxMessage = *CAN2GetRxBufDat();
			switch(RxMessage.StdId)
			{
				case CAN_SWITCH_ID:
					UPDATE_SWITCH(RxMessage);
					break;
				case CAN_LINE0_DATA_ID:
					UPDATE_LINE(RxMessage);
					//更新巡线值
					       if((GET_LINE_VALUE(LINE_CHAN_0) >= g_ucLineMinValue)  &&( GET_LINE_VALUE(LINE_CHAN_0) <= g_ucLineMaxValue))
					       {
					       	
					       	//g_ucLineValue = GET_LINE_VALUE(LINE_CHAN_0);
					       	//不用修改-------
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

/*****************打印自动和非自动模式****************************************/

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
任务名称：LcdDispTask()
任务功能：液晶刷新显示任务
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
		/*检测的思路为，超限值持续一段时间，若超限超时，则自动停止*/
		
			/**********************************机器人的自动复位************************/
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
						SetPathEndPosEx(&stAction_M1,&g_stActionM1DymPathTop,15*3.4043f);//此高度也是取馒头的高度
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

					 if(KEY_N8(usKeyValue))//是否更换陀螺键
					         {
						        //在此等待按键释放
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//在此处理按键后的响应
							
						 	g_ucUseMemsGryo=1;
							g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标	
							stRobot.stPot.ssPosQ=0;
							OPEN_VALVE(TOP_CATCH_VALVE);

					         }
					 //正常重启
					 if(KEY_N0(usKeyValue))
					         {
						        //在此等待按键释放
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//在此处理按键后的响应
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
							g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标

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

				//进入重启1
					 if(KEY_N1(usKeyValue))
					         {
						        //在此等待按键释放
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//在此处理按键后的响应
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
							g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标

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
				//进入重启2

					  if(KEY_N2(usKeyValue))
					         {
						        //在此等待按键释放
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//在此处理按键后的响应
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
							g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标

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
				//进入重启3
					  
					  if(KEY_N3(usKeyValue))
					         {
						        //在此等待按键释放
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//在此处理按键后的响应
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
							g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标

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
				//进入重启4
					  if(KEY_N4(usKeyValue))
					         {
						        //在此等待按键释放
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//在此处理按键后的响应
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
							g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标

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
				//进入重启
					   if(KEY_N5(usKeyValue))
					         {
						        //在此等待按键释放
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//在此处理按键后的响应
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
							g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标

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

				//进入重启
					     if(KEY_N6(usKeyValue))
					         {
						        //在此等待按键释放
						        
							 while(usKeyValue!= 0xffff)
						        {
							        usKeyValue= PSKEY;
						        }
						//在此处理按键后的响应
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
							g_fpModifyAngle = g_fpGyroAngle; //纠正当前陀螺坐标

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

			#if 0//暂时未用

			/******************************************初始化自检操作***************************************************************************************/

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

/***************************************************检测电机安全位置的一些操作***************************************************************************/


					if(stAction_M2.pstCurPath->siEndCode<0)
						{
							stAction_M2.pstCurPath->siEndCode=10;
						}
					if(stAction_M2.pstCurPath->siEndCode>1445)
						{
							stAction_M2.pstCurPath->siEndCode=1445;
						}




/***************************************************保存坐标值的一些操作******************************************************************/
			
			
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
					
			/*********************红场路径的Flash存储****************************************************************/		
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

					/********************************蓝场路径的Flash存储**************************************************/
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
				////////////////////////************红场路径加载*******************/////////////////////////////////////////////


						/*************************重启方式1(正常模式)***************************************/
						//////////////////慢速取馒头//////////////////
						stNavPathRed[1].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_X];
						stNavPathRed[1].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Y];
						stNavPathRed[1].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Q];

						//////////////////慢速扔馒头//////////////////
						stNavPathRed[2].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_X];
						stNavPathRed[2].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Y];
						stNavPathRed[2].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Q];

						//////////////////中速取馒头//////////////////
						stNavPathRed[4].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_X];
						stNavPathRed[4].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Y];
						stNavPathRed[4].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Q];

						//////////////////中速扔馒头//////////////////
						stNavPathRed[5].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_X];
						stNavPathRed[5].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Y];
						stNavPathRed[5].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Q];

						//////////////////高速取馒头//////////////////
						stNavPathRed[7].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_X];
						stNavPathRed[7].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Y];
						stNavPathRed[7].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES1_Q];

						//////////////////高速扔馒头//////////////////
						stNavPathRed[8].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_X];
						stNavPathRed[8].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Y];
						stNavPathRed[8].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES1_Q];
						


						/*************************重启方式2***************************************/
						//////////////////慢速取馒头//////////////////
						stNavPathRed[10].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_X];
						stNavPathRed[10].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Y];
						stNavPathRed[10].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Q];
						//////////////////慢速扔馒头//////////////////
						stNavPathRed[11].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_X];
						stNavPathRed[11].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Y];
						stNavPathRed[11].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Q];

						//////////////////中速取馒头//////////////////
						stNavPathRed[12].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_X];
						stNavPathRed[12].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Y];
						stNavPathRed[12].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Q];
						//////////////////中速扔馒头//////////////////
						stNavPathRed[13].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_X];
						stNavPathRed[13].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Y];
						stNavPathRed[13].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Q];

						//////////////////高速取馒头//////////////////
						stNavPathRed[14].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_X];
						stNavPathRed[14].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Y];
						stNavPathRed[14].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES2_Q];
						//////////////////高速扔馒头//////////////////
						stNavPathRed[15].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_X];
						stNavPathRed[15].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Y];
						stNavPathRed[15].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES2_Q];

						/*************************重启方式3***************************************/	
						//////////////////慢速取馒头//////////////////
						stNavPathRed[17].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_X];
						stNavPathRed[17].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Y];
						stNavPathRed[17].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Q];
						//////////////////慢速扔馒头//////////////////
						stNavPathRed[18].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_X];
						stNavPathRed[18].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Y];
						stNavPathRed[18].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Q];

						//////////////////中速取馒头//////////////////
						stNavPathRed[19].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_X];
						stNavPathRed[19].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Y];
						stNavPathRed[19].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Q];
						//////////////////中速扔馒头//////////////////
						stNavPathRed[20].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_X];
						stNavPathRed[20].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Y];
						stNavPathRed[20].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Q];

						
						//////////////////高速取馒头//////////////////
						stNavPathRed[21].ssEndX=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_X];
						stNavPathRed[21].ssEndY=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Y];
						stNavPathRed[21].ssEndQ=g_siFlashValue[OFST_RED_CATCHBANPATH_RES3_Q];
						//////////////////高速扔馒头//////////////////
						stNavPathRed[22].ssEndX=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_X];
						stNavPathRed[22].ssEndY=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Y];
						stNavPathRed[22].ssEndQ=g_siFlashValue[OFST_RED_RELEASEBANPATH_RES3_Q];
					
					#endif


#if 1
					#if 1
						////////////////////////************蓝场路径加载*******************/////////////////////////////////////////////


						/*************************重启方式1(正常模式)***************************************/
						//////////////////慢速取馒头//////////////////
						stNavPathBlue[1].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_X];
						stNavPathBlue[1].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Y];
						stNavPathBlue[1].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Q];

						//////////////////慢速扔馒头//////////////////
						stNavPathBlue[2].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_X];
						stNavPathBlue[2].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Y];
						stNavPathBlue[2].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Q];
					#endif
#if 1
						//////////////////中速取馒头//////////////////
						stNavPathBlue[4].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_X];
						stNavPathBlue[4].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Y];
						stNavPathBlue[4].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Q];

						//////////////////中速扔馒头//////////////////
						stNavPathBlue[5].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_X];
						stNavPathBlue[5].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Y];
						stNavPathBlue[5].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Q];

						//////////////////高速取馒头//////////////////
						stNavPathBlue[7].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_X];
						stNavPathBlue[7].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Y];
						stNavPathBlue[7].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES1_Q];

						//////////////////高速扔馒头//////////////////
						stNavPathBlue[8].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_X];
						stNavPathBlue[8].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Y];
						stNavPathBlue[8].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES1_Q];
#endif

#if 1


						/*************************重启方式2***************************************/
						//////////////////慢速取馒头//////////////////
						stNavPathBlue[10].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_X];
						stNavPathBlue[10].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Y];
						stNavPathBlue[10].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Q];
						//////////////////慢速扔馒头//////////////////

						#if 1
						stNavPathBlue[11].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_X];
						stNavPathBlue[11].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Y];
						stNavPathBlue[11].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Q];
						#endif

						#if 1

						//////////////////中速取馒头//////////////////
						stNavPathBlue[12].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_X];
						stNavPathBlue[12].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Y];
						stNavPathBlue[12].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Q];

						
						//////////////////中速扔馒头//////////////////
						stNavPathBlue[13].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_X];
						stNavPathBlue[13].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Y];
						stNavPathBlue[13].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Q];

						//////////////////高速取馒头//////////////////
						stNavPathBlue[14].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_X];
						stNavPathBlue[14].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Y];
						stNavPathBlue[14].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES2_Q];
						//////////////////高速扔馒头//////////////////
						stNavPathBlue[15].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_X];
						stNavPathBlue[15].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Y];
						stNavPathBlue[15].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES2_Q];
						#endif
#endif

#if 1
						/*************************重启方式3***************************************/	
						//////////////////慢速取馒头//////////////////
						stNavPathBlue[17].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_X];
						stNavPathBlue[17].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Y];
						stNavPathBlue[17].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Q];
						//////////////////慢速扔馒头//////////////////
						stNavPathBlue[18].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_X];
						stNavPathBlue[18].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_Y];
						stNavPathBlue[18].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_Q];

						//////////////////中速取馒头//////////////////
						stNavPathBlue[19].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_X];
						stNavPathBlue[19].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Y];
						stNavPathBlue[19].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Q];
						//////////////////中速扔馒头//////////////////
						stNavPathBlue[20].ssEndX=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_X];
						stNavPathBlue[20].ssEndY=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_Y];
						stNavPathBlue[20].ssEndQ=g_siFlashValue[OFST_BLUE_RELEASEBANPATH_RES3_Q];

						
						//////////////////高速取馒头//////////////////
						stNavPathBlue[21].ssEndX=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_X];
						stNavPathBlue[21].ssEndY=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Y];
						stNavPathBlue[21].ssEndQ=g_siFlashValue[OFST_BLUE_CATCHBANPATH_RES3_Q];
						//////////////////高速扔馒头//////////////////
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
 

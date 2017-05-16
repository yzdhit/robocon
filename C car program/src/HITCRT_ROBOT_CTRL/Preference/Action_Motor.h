#ifndef	__ACTION_MOTOR_H
#define __ACTION_MOTOR_H
#include "MotorConfig.h"
#include "math.h"

#define M1COFF_POT	3.341549f						

#define M2COFF_POT	3.745501f

/*****************M1电机的运动尺寸*************************/
#define M1M_PICKUP_C		-138
#define M1M_PUTDOWN_C		24
#define M1A_DOWN_C			-150
#define M1A_UP_C			57
#define M1_L2_UP_C			24
#define M1_CLIMBSTEP_C		-221
#define M1_DOUBLEPOTLOOP_C   -188

/***************************蓝场电机尺寸************************************/
#define M1_BLUE_PICKUPBAN_TEMP_C	7
#define M1_BLUE_PICKUPBAN_LAST_C	20
#define M1_BLUE_WAIT_M_C			141//C抓完馒头之后等待M时的高度
#define M1_BLUE_DROPBAN_C			135
#define M1_BLUE_LIFT_BAN			135//C抓完馒头之后提起的高度
#define M1_BLUE_CATCHTOPBAN_C		145//抓取顶层馒头时，伸出的距离
#define M1_BLUE_PICKUPTOPBAN_C	240//274
#define M1_BLUE_DROPTOPBAN_C		-190//扔顶层馒头的距离
/***************************红场电机尺寸*******************************************************/
#define M1_RED_PICKUPBAN_TEMP_C	7
#define M1_RED_PICKUPBAN_LAST_C	20
#define M1_RED_LIFT_BAN			135//C抓完馒头之后提起的高度
#define M1_RED_WAIT_M_C			141//C抓完馒头之后等待M时的高度
#define M1_RED_DROPBAN_C			135
#define M1_RED_CATCHTOPBAN_C		67//120//抓取顶层馒头时，伸出的距离
#define M1_RED_PICKUPTOPBAN_C		240//274
#define M1_RED_DROPTOPBAN_C		-190//扔顶层馒头的距离
/***********************************************************************/

#define M1_BLUE_CATCHMIDDLEBAN_C	20//此处是C车取馒头时，降的高度很低，以便夹在馒头底间

#define M1_RED_CATCHMIDDLEBAN_C	20//此处是C车取馒头时，降的高度很低，以便夹在馒头底间
/******************M2电机的运动尺寸***********************************************/

/***********************蓝场电机尺寸*******************************/
#define M2_BLUE_PICKUPBANRES1_LAST_C	35//重启路径1时，取中层馒头M2电机伸出的最终位移
#define M2_BLUE_PICKUPBANRES2_LAST_C	80  //重启路径2时，取中层馒头M2电机伸出的最终位移
#define M2_BLUE_PICKUPBANRES3_TEMP_C	80
#define M2_BLUE_PICKUPBANRES3_LAST_C	110//重启路径3时，取中层馒头M2电机伸出的最终位移
#define M2_BLUE_DROPBAN_C				40//扔中层和底层馒头时M2电机伸出的距离
#define M2_BLUE_PICKUPTOPBAN_C		370//夹取顶层馒头时M2电机伸出的距离
#define M2_BLUE_DROPTOPBAN_C			390//扔顶层馒头时M2电机伸出来的距离
#define M2_BLUE_CATCHTOPBAN_COM_ON_C	10//开启通信时，M2电机的阈度

/***********************红场电机尺寸*******************************/
#define M2_RED_PICKUPBANRES1_LAST_C	35//重启路径1时，取中层馒头M2电机伸出的最终位移
#define M2_RED_PICKUPBANRES2_LAST_C	60  //重启路径2时，取中层馒头M2电机伸出的最终位移
#define M2_RED_PICKUPBANRES3_TEMP_C	80
#define M2_RED_PICKUPBANRES3_LAST_C	110//重启路径3时，取中层馒头M2电机伸出的最终位移
#define M2_RED_DROPBAN_C				40//扔中层和底层馒头时M2电机伸出的距离
#define M2_RED_PICKUPTOPBAN_C			370//345//夹取顶层馒头时M2电机伸出的距离
#define M2_RED_DROPTOPBAN_C			390//扔顶层馒头时M2电机伸出来的距离
#define M2_RED_CATCHTOPBAN_COM_ON_C	10//开启通信时，M2电机的阈度
 
 
 
 
 
 
 
 
 
 

/*--------------------配置执行机构电机-----------------------*/
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
#define Action_M2_COFF_POT_INIT		-0.042531710964f//-0.042531710964f大线轮
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





//执行电机PID参数

/*定义PID参数数据结构*/
/**************************************************************
typedef struct 
{
	FP32 fpP;		      	//比例系数KP
	FP32 fpI;		      	//积分系数KI
	FP32 fpD;		      	//微分系数KD
	FP32 fpE;			  	//偏差fpE(i)
	FP32 fpPreE;		  	//偏差fpE(i-1)
	FP32 fpPrePreE;			//偏差的累加
	FP32 fpU;	  			//本次PID运算结果
	FP32 fpELimit;  		//做积分分离运算时的极限偏差
	FP32 ssULimit;			//做遇限削弱时的上限值
	FP32 fpEDead;			//死区间隔
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
	UCHAR8   ucSeries;        //路段类型
	SSHORT16 scOver;		//结束类型  1<<0--码盘计数增加，0<<0--码盘计数减少，1<<4--结束时位置闭环，1<<5--结束时PWMDuty=0,1<<6--结束时开环PWMDuty =?,1<<7 结束时零速pid
	SINT32   siStartCode;     //StartCode
	SINT32   siEndCode;       //EndCode
	SSHORT16 ssStartV;      //StartVelocity (mm/s)
	SSHORT16 ssEndV;        //EndVelocity (mm/s)
}ST_ACTION_PATH;*/
/*执行电机控制结构体  2010 12 28*/
#if 0
const ST_ACTION_PATH astActionPath[AMQ][20] =   
{
		/*SerialNumber		EndTypes	StartCode\EndCode			StartV\EndV   MaxV   AccUp/AccDown   EndPwmDuty*/ 
		//提升机构的路径
	{
		//横移
		{0,				 0x41,			0,28824,					20,20,		      70,        2,2,		1200}, //提升机构匀加速运动
		{1,				 0x10,			28824,-21000,					20,20,		      70,        2,2,		-1500}, //提升机构匀加速运动
	},
	{
		//收包
		{0,				 0x10,			0,-57500,					30,50,		      200,        3,6,		0}, //提升机构匀加速运动
		{1,				 0x10,			60000,0,					10,10,		      80,        0.8,0.8,		-200}, //提升机构匀加速运动
	},
	{
		//旋转
		{0,				 0x10,			0,-1300,					15,15,		      25,        0.5,0.5,		0}, //提升机构匀加速运动
//		{1,				 0x00,			-1250,-1350,					5,5,		      5,        0.5,0.5,		0}, //提升机构匀加速运动
//		{2,				 0x01,			-1350,-1150,					5,5,		      5,        0.5,0.5,		0}, //提升机构匀加速运动		
		{1,				 0x10,			-1300,-2000,					15,15,		      25,        0.8,0.8,		-500}, //提升机构匀加速运动
	},

		

};
#endif
                                                            /*SerialNumber		EndTypes	StartCode\EndCode			StartV\EndV\MaxV   AccUp/AccDown \EndPwmDuty*/
// 低速
ST_ACTION_PATH g_stActionM1DymPathLow = {0,                          0x11,  0 , 10 ,                                            10, 10, 100,            100, 100 , 0};
ST_ACTION_PATH g_stActionM2DymPathLow = {0,                          0x11,  0 , 10 ,                                            10, 10, 100,            20, 20 , 0};

// 中速
ST_ACTION_PATH g_stActionM1DymPathMid = {0,                          0x11,  0 , 10 ,                                            10, 10, 350,            105, 105 , 0};
ST_ACTION_PATH g_stActionM2DymPathMid = {0,                          0x11,  0 , 10 ,                                            10, 10, 100,            20, 20 , 0};

//高速
ST_ACTION_PATH g_stActionM1DymPathTop = {0,                          0x11,  0 , 10 ,                                            10, 10, 350,            150, 150 , 0};
ST_ACTION_PATH g_stActionM2DymPathTop = {0,                          0x11,  0 , 10 ,                                            10, 10, 250,            35, 35 , 0};


//实例化执行机构电机

INSTANCE_MOTOR(Action_M1, stAction_M1, Action_M1_MOTOR_CHANNEL, Action_M1_CODER_CHANNEL, Action_M1_MOTOR_DIR_SEL);
INSTANCE_MOTOR(Action_M2, stAction_M2, Action_M2_MOTOR_CHANNEL, Action_M2_CODER_CHANNEL, Action_M2_MOTOR_DIR_SEL);
INSTANCE_MOTOR(Action_M3, stAction_M3, Action_M3_MOTOR_CHANNEL, Action_M3_CODER_CHANNEL, Action_M3_MOTOR_DIR_SEL);


#endif

#ifndef	__HITCRT_ROBOTTYPES_H
#define	__HITCRT_ROBOTTYPES_H

#include "HITCRT_Types.h"
#include "Select.h"

/*运行状态*/
typedef enum
{
	FIRST,		//首次运行
	RUN			//已经运行
}EM_RUN_STATE;

/*向量结构体*/
typedef struct 
{
	SSHORT16 ssVx;
	SSHORT16 ssVy;
}ST_VECTOR;

/*定义PID参数数据结构*/
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
	FP32 fpULimit;			//做遇限削弱时的上限值
	FP32 fpEDead;			//死区间隔
	FP32 fpEthreshold;           //限制输入误差的最大值，当不用时，请把该值设为较大的一个数
}ST_PID;

/*机器人位姿结构体*/
typedef struct
{
	SSHORT16 ssPosX;	//横坐标(单位：mm)
	SSHORT16 ssPosY;	//纵坐标(单位：mm)
	SSHORT16 ssPosQ;	//航向角(单位：0.1°)
}ST_ROBOT_POT;

/*定义路径结构体*/
typedef struct 				
{
	UCHAR8   ucSeries;	//路段编号
	UCHAR8   ucType;	//路段类型
	SCHAR8   scOver;	//结束类型
	SSHORT16 ssStartX;	//StartX
	SSHORT16 ssStartY;	//StartY
	SSHORT16 ssEndX;	//EndX
	SSHORT16 ssEndY;	//EndY
	UCHAR8 	 ucMode;	//速度加速类型
	USHORT16 usStartV;	//路段起始速度
	USHORT16 usEndV;	//路段终止速度
	SSHORT16 ssCenterX;	//CenterX
	SSHORT16 ssCenterY;	//CenterY
	SSHORT16 ssRadius;	//Radius
	SSHORT16 ssStartQ;  //起点的航向角
	SSHORT16 ssEndQ;    //终点的航向角(0.1°)
	USHORT16 usMaxV;	//路段最高速
	FP32 fpAccUp;	//路段加速度
	FP32 fpAccDown;	//路段减速度
	FP32 fpDisRatio;//路段的旋转路段占总路段的比例，按照余弦分配
}ST_PATH;				//定义路径结构体

/*执行电机控制结构体（新加的）*/
typedef struct
{
	UCHAR8   ucSeries;        //路段类型
	SSHORT16 scOver;		//结束类型
	SINT32   siStartCode;     //StartCode
	SINT32   siEndCode;       //EndCode
	SSHORT16 ssStartV;      //StartVelocity (mm/s)
	SSHORT16 ssEndV;        //EndVelocity (mm/s)
	SSHORT16 ssMaxV;	//路段最高速
	FP32 fpAccUp;	//路段加速度
	FP32 fpAccDown;	//路段减速度
	SSHORT16 EndPwmDuty;
//	SINT32   siUpCode;     //加速转到匀速的码盘值
//	SINT32   siDownCode;       //匀速转到加速的码盘值
}ST_ACTION_PATH;


/*时间间隔结构体*/
typedef struct
{
	UINT32 uiPreTime;		//前一时刻
	UINT32 uiCurTime;		//当前时间
	UINT32 uiIntlTime;		//间隔时间
	EM_RUN_STATE emRunState;//
}ST_INTL_TIME;

/*码盘过线速度结构体*/
typedef struct
{
	SINT32	siPreCodeNum;	//上一次码盘读数
	SINT32 	siCurCodeNum;	//当前码盘读数
	SINT32	siDetCodeNum;	//码盘读数差值
	ST_INTL_TIME	stIntlTime; 	//时间间隔
	EM_RUN_STATE  emRunState;
}ST_VELT_CODER;
//定义双随动全向轮距离反馈结构体
typedef struct
{
	SINT32 siCoderA;		//随动轮A码盘脉冲数计数
	SINT32 siLastCoderA;	//随动轮A码盘脉冲数上一次计数
	SINT32 siCoderB;		//随动轮B码盘脉冲数计数
	SINT32 siLastCoderB;	//随动轮B码盘脉冲数上一次计数
	SINT32 siPosQ;			//本次航向角
	SINT32 siLastPosQ;		//上一次航向角
	FP32 fpPosQFix;			//局部坐标系航向角纠偏
	FP32 fpKCodeToDegA;		//随动轮A码盘脉冲数到弧度数转换系数
	FP32 fpKCodeToDegB;		//随动轮B码盘脉冲数到弧度数转换系数
	FP32 fpOMNI_WHEEL_R;	//随动轮AB的半径
	FP32 fpOMNI_Position_L;	//随动轮安装位置相对与全向轮底盘的中点位置
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
	/***********机器人复位时使用*****************/
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
	/***********机器人复位时使用*****************/
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
       /*预留三个任务，供插入*/
	TASK_START_1,
	TASK_START_2,
	TASK_START_3,

	TASK_INSERT_1,
	TASK_INSERT_2,
	TASK_INSERT_3,

	TASK_INSERT_4,

	TASK_INSERT_5,//用于重启的时候使用
	TASK_INSERT_6,//用于重启方式4的时候使用
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
	
	/*预留三个任务收尾*/
	TASK_END_1,
	TASK_END_2,
	TASK_END_3,
	
	OVER
}EM_ROBOT_TASK;
typedef struct
{
	SSHORT16		ssPwmDuty;			//电机PWM占空比
	USHORT16		usMaxPwmDuty;		//电机控制最大占空比
	SINT32			siCoderNum;			//码盘线数
	FP32			fpPotFB;			//反馈位置Position，feedback
	FP32			fpPrePotFB;			//上一次反馈位置
	FP32			fpPotDes;			//期望位置，desired
	FP32            fpPotSen;           //结束位置偏差
	FP32			fpCoffPot;			//实际位置与码盘线数转换系数
	FP32			fpVeltFB;			//反馈速度Velocity
	FP32			fpVeltDes;			//期望速度
	FP32            fpMaxVelt;			//电机控制的最大速度
	FP32			fpCoffVelt;			//实际速度与码盘过线速度的转换系数					
	ST_VELT_CODER	stVeltCoder;		//码盘过线速度结构体
	ST_PID			stVeltPid;				//电机速度闭环控制PID结构体
	ST_PID			stPotPid;				//电机位置闭环控制PID结构体
	void			(*TurnMotor)(SSHORT16 ssPwmDuty);		//电机开环控制函数
	ST_ACTION_PATH*	pstCurPath;
	ST_ACTION_PATH*	pstPrePath;
	UCHAR8 			ucName[12];			//电机名称
	UCHAR8			ucMotorChan;		//电机通道
	UCHAR8			ucCoderChan;		//码盘通道
	EM_MOTOR_STATE	emState;			//电机工作状态
	EM_PATH_STATE	emPathState;
}ST_MOTOR_CTRL;

//机器人速度结构体
typedef struct
{
	FP32 fpVx;		//X方向速度
	FP32 fpVy;		//Y方向速度
	FP32 fpW;		//旋转角速度
}ST_ROBOT_VELT;

#if	BASE_TYPE
typedef struct
{
	ST_PID	stPidRot;				//旋转PID，Rotation
	ST_PID	stPidTrvs;				//横向PID，Transverse
	ST_PID	stPidVtc;				//纵向PID，Vertical
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
	SINT32    siCoderA;		//随动轮A码盘脉冲数计数
	SINT32    siLastCoderA;	//随动轮A码盘脉冲数上一次计数
	SINT32    siCoderB;		//随动轮B码盘脉冲数计数
	SINT32    siLastCoderB;	//随动轮B码盘脉冲数上一次计数
	FP32      fpPosQ;	    //本次航向角(单位: 0.1°)
	FP32      fpLastPosQ;	//上一次航向角
	FP32      fpPosX;	    //横坐标(单位：mm)
	FP32      fpLastPosX;   //上一次横坐标
	FP32      fpPosY;	    //纵坐标(单位：mm)
	FP32      fpLastPosY;   //上一次纵坐标
	
}ST_DOUBLE_OMNI_LOCATION;	    //定义双随动全向轮距离反馈结构体
//自己定义的跟踪变量用的结构体
typedef struct
{
	SSHORT16              ssPotX;
	SSHORT16              ssPotY;
//	SSHORT16              ssPotQ;
	FP32                  fpE;//切向的偏差
	FP32                  fpDis;//总距离
	FP32                  fpK;//系数
}ST_VAR_WATCH;


/*---------------PS2手柄------------------*/
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


//雷达相关的定义
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
	SINT32 NumLines;  //  直线段数码
	FP32 Rho1;	   //  第一条线倾角
	FP32 Rho2;	   //  第二条直线倾角
	FP32 Dis1;    //  第一条直线到原点距离 ， 直线 y = a * x + b 中 b 的符号是dis的符号
	FP32 Dis2;    //  第二条直线到原点距离
}ST_VDL;

											
#endif


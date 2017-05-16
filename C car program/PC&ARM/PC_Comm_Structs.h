
#ifndef ROBOCON_COMM_STRUCTS
#define ROBOCON_COMM_STRUCTS
#include "HITCRT_Types.h"
#include "HITCRT_RobotTypes.h"
/********************************************************
该文件存放与PC通信所使用的结构体定义
*********************************************************/
/*分配通信对象的ID*/

/****************例程*********************/

#define  COMMU_FLAG  1	    // 上位机通信使能
#define   stMyTest_ID    1		//分配stMyTest的ID为1	
#define stSendVelt_ID   2
#define stGryo_ID  9                //陀螺id
#define  VISIONDETECLINE_ID  0x72

ST_VDL stVisionDetecLine = {0,-1,-1,-1,-1};  
ST_GRYO stGryoData = {0,0};

typedef struct
{
	FP32 fpWaMotorVelt;
	FP32 fpWbMotorVelt;
}stSendVelt;

typedef struct
{
	UINT32 uiFir;
	UCHAR8 ucSec;
	FP32  fpThir;
	SCHAR8 scFour;	
}stTestStruct;
/*定义通信变量*/


stTestStruct  stMyTest;  //定义通信变量
/****************例程*********************/

#define   stPV_ID    2	

typedef struct
{
	
	FP32  fpPotDes;
	FP32  fpVeltFB;	
}stPotVelt;
/*定义通信变量*/

//stPotVelt   stPV;

#define stDiffRobot_ID 3
typedef struct
{
	SINT32 siPosX;
    SINT32 siPosY;
}ST_POS;

//ST_POS stDiffRobotPos;


#define stTestAcc_ID 4
typedef struct
{
    SINT32 siPosX;
    SINT32 siPosY;
    SINT32 siPosQ;
    FP32   fpActualV;
    FP32   fpDesV;
}ST_TEST_ACC;

#define stSensorData_ID  5
typedef struct
{
    SINT32 siPosX;
    SINT32 siPosY;
    SINT32 siPosQ;
    SINT32 siCoder;
    SINT32 siTimer;
    SINT32 siCnt;
}ST_SENSORDATA;
ST_SENSORDATA stSensorData;

#endif


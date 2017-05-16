
#ifndef ROBOCON_COMM_STRUCTS
#define ROBOCON_COMM_STRUCTS
#include "HITCRT_Types.h"
#include "HITCRT_RobotTypes.h"
/********************************************************
���ļ������PCͨ����ʹ�õĽṹ�嶨��
*********************************************************/
/*����ͨ�Ŷ����ID*/

/****************����*********************/

#define  COMMU_FLAG  1	    // ��λ��ͨ��ʹ��
#define   stMyTest_ID    1		//����stMyTest��IDΪ1	
#define stSendVelt_ID   2
#define stGryo_ID  9                //����id
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
/*����ͨ�ű���*/


stTestStruct  stMyTest;  //����ͨ�ű���
/****************����*********************/

#define   stPV_ID    2	

typedef struct
{
	
	FP32  fpPotDes;
	FP32  fpVeltFB;	
}stPotVelt;
/*����ͨ�ű���*/

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


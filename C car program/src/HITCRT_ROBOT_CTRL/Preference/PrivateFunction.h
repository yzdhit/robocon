#ifndef PRIVATE_FUNCTION_H
#define PRIVATE_FUNCTION_H

#include "HITCRT_RobotTypes.h"

// 从线速度 mm/s 到转速 转每分 ， 从0.1度每秒到 转每分
#define L_X   0.187978282f
#define L_Y   0.187978282f
#define L_W  0.0785761144f

extern void DesRobotV(ST_JS_FPGA *pstJoyStickValue, ST_ROBOT_VELT *pstV);
extern void ManualBaseVeltAllocate(ST_OMNI_MOBILE_ROBOT *pstR,ST_ROBOT_VELT *pstV, SINT32 siDirAngle);
extern void AutoBaseVeltAllocate(ST_OMNI_MOBILE_ROBOT *pstR,ST_ROBOT_VELT *pstV, SINT32 siDirAngle);
extern void ZerosPID(ST_PID * pPid);
extern UINT32 GetCurTime(void);


extern void RcvMEMSGryo(UCHAR8 RcvData,SSHORT16 * pssAngle);
extern void StopMEMSGryo(void);
extern void StartMEMSGryo(void);
extern void SetValvePwm(UINT32 uiOpenTime,UINT32 uiTotalTime,UCHAR8 ucValveChan  );
extern UCHAR8 FindUCHAR8Min(UCHAR8 * pucData, UCHAR8 ucLen);
extern UCHAR8 FindUCHAR8Max(UCHAR8 * pucData, UCHAR8 ucLen);
extern SINT32 RetVisionLineValue(ST_VDL * pstVDL);
extern FP32  ReadRobotFpVy(ST_OMNI_MOBILE_ROBOT *pstR);

#endif



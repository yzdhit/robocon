#ifndef _CAN_API_H_
#define _CAN_API_H_
#include "HITCRT_Types.h"
#include "stm32f4xx.h"
#include "HITCRT_RobotTypes.h"

//id的分配


//气动板上的id
#define  CAN_SWITCH_ID   0x20
#define  CAN_AIR_ID          0x30
#define  CAN_SERVO_ID      0x40

//开关相关的函数,开关的通道0-15
#define UPDATE_SWITCH(RxMsg)   (UpdateSwitchValue(&g_usSwitchCan, &RxMsg))
#define IS_SWITCH_ON(channel)   (g_usSwitchCan&(0x0001 << channel))
#define IS_SWITCH_OFF(channel)  (!(g_usSwitchCan&(0x0001 << channel)))
#define GET_SWITCH_VALUE()        (g_usSwitchCan)
#define SET_SWITCH_VALUE(value)   (g_usSwitchCan = value)

//气动相关的函数，气动的通道0-7
#define OPEN_VALVE(channel)          (g_ucAirValveCan |= (0x01<<channel));SendAirMsgByCan2(&g_ucAirValveCan)
#define CLOSE_VALVE(channel)        (g_ucAirValveCan &= (~((UCHAR8)(0x01<<channel))));SendAirMsgByCan2(&g_ucAirValveCan)
#define IS_VALVE_OPEN(channel)    (g_ucAirValveCan &(0x01<<channel))
#define IS_VALVE_CLOSE(channel)   (!(g_ucAirValveCan &(0x01<<channel)))
#define GET_VALVE_VALUE()             (g_ucAirValveCan)
#define SET_VALVE_VALUE(value)    (g_ucAirValveCan = value);SendAirMsgByCan2(&g_ucAirValveCan)


//舵机相关的函数，舵机通道 0 - 3,占空比从50-250
#define SET_SERVO_PWM(channel,value)  g_ucServoCan[channel]= value;SendServoMsgByCan2(channel,  value)
#define GET_SERVO_PWM(channel)   (g_ucServoCan[channel])

//巡线部分-最多支持四个巡线板
//配置id
#define CAN_CONFIG_LINE_ID 0xc0
//数据接收id
#define CAN_LINE0_DATA_ID  0x70
#define CAN_LINE1_DATA_ID  0x71
#define CAN_LINE2_DATA_ID  0x72
#define CAN_LINE3_DATA_ID  0x73
//命令发送id
#define CAN_LINE0_CMD_ID   0x78
#define CAN_LINE1_CMD_ID  0x79
#define CAN_LINE2_CMD_ID  0x7a
#define CAN_LINE3_CMD_ID  0x7b







//巡线相关的函数
#define UPDATE_LINE(RxMsg)  (UpdateLineValue(&RxMsg))
#define GET_LINE_VALUE(chan)      (g_ucLineCan[chan])

#define CONFIG_TO_LINE(chan)   (SendLineCfgCmdMsgByCan2(chan))

#define CLOSE_LINE_TRACK(chan)  (SendLineCmdMsgByCan2(chan,0x20))
#define OPEN_LINE_TRACK(chan)  (SendLineCmdMsgByCan2(chan,0x01))

//#define CALIBRATE_NOT_WHITE_LINE(chan)  (SendLineCmdMsgByCan2(chan,0x04))/*标定非白线扩展成以下四种颜色的值*/
#define CALIBRATE_WHITE_LINE(chan)  (SendLineCmdMsgByCan2(chan,0x08))
#define CALIBRATE_RED_LINE(chan)     (SendLineCmdMsgByCan2(chan,0x33))
#define CALIBRATE_GREEN_LINE(chan)  (SendLineCmdMsgByCan2(chan,0x31))
#define CALIBRATE_BLUE_LINE(chan)    (SendLineCmdMsgByCan2(chan,0x32))
#define CALIBRATE_BLACK_LINE(chan)   (SendLineCmdMsgByCan2(chan,0x34))  

/*新增的调用某个场地的标定系数命令*/
#define  USE_GREEN_FIELD_CFG(chan)  (SendLineCmdMsgByCan2(chan,0x35)) 
#define  USE_BLUE_FIELD_CFG(chan)     (SendLineCmdMsgByCan2(chan,0x36)) 
#define  USE_RED_FIELD_CFG(chan)   (SendLineCmdMsgByCan2(chan,0x37)) 
#define  USE_BLACK_FIELD_CFG(chan)   (SendLineCmdMsgByCan2(chan,0x38)) 

#define SET_LINE_SEND_SPEED(chan,value)   (SendLineCmdDataMsgByCan2(chan,value,0x10))
#define SET_LINE_LIGHT_FORCE(chan,value)   (SendLineCmdDataMsgByCan2(chan,value,0x02))

//调试的时候使用
#define GET_SEND_ID_CMD(chan)   (SendLineCmdMsgByCan2(chan,0x14))
#define GET_RCV_ID_CMD(chan)    (SendLineCmdMsgByCan2(chan,0x15))
#define GET_CALIBRATE_CMD(chan)  (SendLineCmdMsgByCan2(chan,0x12))
#define GET_LIGHT_FORCE_CMD(chan) (SendLineCmdMsgByCan2(chan,0x16))
#define GET_SPEED_CMD(chan)   (SendLineCmdMsgByCan2(chan,0x13))


//返回灯的状态的的命令
#define GET_LIGHT_STATE_CMD(chan)   (SendLineCmdMsgByCan2(chan,0x40))
//#define GET_FIELD_INFO_CMD(chan)   (SendLineCmdMsgByCan2(chan,0x39))



#define GET_SPEED_VALUE(chan)    (g_ucRetSpeedCan[chan])
#define GET_LIGHT_FORCE_VALUE(chan)  (g_ucRetLightForceCan[chan])
#define GET_CALIBRATE_VALUE(chan)   (g_ucRetCalibrateCan[chan])
#define GET_SEND_ID_VALUE(chan)   (g_ucRetSendIDCan[chan])
#define GET_RCV_ID_VALUE(chan)    (g_ucRetRcvIDCan[chan])
//返回场地信息值
#define GET_FIELD_INFO_VALUE(chan)  (g_ucRetFieldInfoCan[chan])  
//返回巡线板灯的状态
#define GET_LIGHT_STATE_VALUE(chan)  (g_uiLightStateCan[chan])



// 雷达相关的函数
#define GET_RADAR_VALUE()  (g_stRadarPosCan)
#define GET_CALIBRATE_POS()  (g_stCalPosCan)
#define UPDATE_RADAR(RxMsg)    (UpdateRadarValue(&RxMsg))
//雷达的接受id
#define CAN_RADAR_RCV_ID  0x60

//开关
extern USHORT16 g_usSwitchCan;
//气动
extern UCHAR8 g_ucAirValveCan;
//舵机
extern UCHAR8 g_ucServoCan[4];
// 巡线
extern UCHAR8 g_ucLineCan[4];
extern UCHAR8 g_ucRetSpeedCan[4];
extern UCHAR8 g_ucRetSendIDCan[4];
extern UCHAR8 g_ucRetRcvIDCan[4];
extern UCHAR8 g_ucRetLightForceCan[4];
extern UCHAR8 g_ucRetCalibrateCan[4];
extern UCHAR8 g_ucRetFieldInfoCan[4];
extern UINT32 g_uiLightStateCan[4];

//雷达相关的定义
extern ST_POS_INFO g_stRadarPosCan;
extern ST_CALIBRATE_POS g_stCalPosCan;


extern void  UpdateSwitchValue(USHORT16*pusSwitch,const CanRxMsg * pRxMsg);
extern void SendAirMsgByCan2(const UCHAR8 * pAir);
extern void SendServoMsgByCan2(UCHAR8 chan,UCHAR8 value);
extern void SendLineCfgCmdMsgByCan2(UCHAR8 ucType);
extern void SendLineCmdDataMsgByCan2(UCHAR8 ucChan,UCHAR8 ucData,UCHAR8 ucType);
extern void SendLineCmdMsgByCan2(UCHAR8 ucChan,UCHAR8 ucType);
extern void  UpdateLineValue(const CanRxMsg * pRxMsg);

extern void  UpdateRadarValue(const CanRxMsg * pRxMsg);






#endif

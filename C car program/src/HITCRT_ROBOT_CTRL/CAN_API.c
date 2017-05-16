#include "CAN_API.h"
#include "CAN.h"
#include "stm32f4xx.h"
#include "HITCRT_RobotTypes.h"

//相关变量的定义,为了防止命名冲突，所有变量以Can结尾
//开关
USHORT16 g_usSwitchCan = 0;
//气动
UCHAR8 g_ucAirValveCan = 0;
//舵机
UCHAR8 g_ucServoCan[4] = {0};
//巡线
UCHAR8 g_ucLineCan[4] = {0};
UCHAR8 g_ucRetSpeedCan[4] = {0};
UCHAR8 g_ucRetSendIDCan[4]= {0};
UCHAR8 g_ucRetRcvIDCan[4]= {0};
UCHAR8 g_ucRetLightForceCan[4]= {0};
UCHAR8 g_ucRetCalibrateCan[4]= {0};


//巡线的场地信息
UCHAR8 g_ucRetFieldInfoCan[4] = {0};
// 巡线板的灯的好坏检测
UINT32 g_uiLightStateCan[4] = {0};



// 雷达位置的定义
ST_POS_INFO g_stRadarPosCan= {{0,0},{-1,-1}};
ST_CALIBRATE_POS g_stCalPosCan = {0};


/**************************************************************
** 函数名:UpdateSwitchValue
** 功能: 更新开关的值
** 参数 :开关变量的指针，消息的指针
** 注意事项:
***************************************************************/
void  UpdateSwitchValue(USHORT16*pusSwitch,const CanRxMsg * pRxMsg)
{
	*pusSwitch =*((USHORT16*) &(pRxMsg->Data[0]));
}

/**************************************************************
** 函数名:SendAirMsgByCan2
** 功能: 通过    CAN2发送气动的变量
** 参数 :气动变量的指针
** 注意事项:
***************************************************************/
void SendAirMsgByCan2(const UCHAR8 * pAir)
{
	static CanTxMsg TxMessage = {CAN_AIR_ID, 0x00, CAN_ID_STD, CAN_RTR_DATA, 3, 0,0,0,0,0,0,0,0};
	static UCHAR8 s_ucLastAir = 0;
	if(s_ucLastAir != *pAir )//值改变时才发送
	{
		s_ucLastAir = *pAir;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = 0;
		TxMessage.Data[2] = *pAir;
		CAN2PutDatatoTxBuf(&TxMessage);
		CAN2BeginSend();
	}
	
}


/**************************************************************
** 函数名:SendServoMsgByCan2
** 功能: 通过    CAN2发送舵机的变量
** 参数 :通道号0-3，pwm值
** 注意事项:
***************************************************************/
void SendServoMsgByCan2(UCHAR8 chan,UCHAR8 value)
{
	static CanTxMsg TxMessage = {CAN_SERVO_ID, 0x00, CAN_ID_STD, CAN_RTR_DATA, 3, 0,0,0,0,0,0,0,0};
	static UCHAR8 s_aucLastPwm[4] = {150,150,150,150};
	if(s_aucLastPwm[chan] != value)//值改变时才发送
	{
		s_aucLastPwm[chan] = value;
		TxMessage.Data[0] = 0;
		TxMessage.Data[1] = chan;
		TxMessage.Data[2] = value;
		CAN2PutDatatoTxBuf(&TxMessage);
		CAN2BeginSend();
	}
}


/**************************************************************
** 函数名:SendLineCfgCmdMsgByCan2
** 功能: 根据配置命令类型，设置相关的参数
** 参数 :类型0-3
** 注意事项:
***************************************************************/
void SendLineCfgCmdMsgByCan2(UCHAR8 ucType)
{
	static CanTxMsg TxMessage = {CAN_CONFIG_LINE_ID, 0x00, CAN_ID_STD, CAN_RTR_DATA, 2, 0,0,0,0,0,0,0,0};
	switch(ucType)
	{
		case 0 : 
			TxMessage.Data[1] = CAN_LINE0_DATA_ID;
			break;
		case 1 : 
			TxMessage.Data[1] = CAN_LINE1_DATA_ID;
			break;
		case 2 : 
			TxMessage.Data[1] = CAN_LINE2_DATA_ID;
			break;
		case 3 :
			TxMessage.Data[1] = CAN_LINE3_DATA_ID;
			break;
		default:
			break;
		
	}
	TxMessage.Data[0] = 0x80;	
	CAN2PutDatatoTxBuf(&TxMessage);
	CAN2BeginSend();
	switch(ucType)
	{
		case 0 : 
			TxMessage.Data[1] = CAN_LINE0_CMD_ID;
			break;
		case 1 : 
			TxMessage.Data[1] = CAN_LINE1_CMD_ID;
			break;
		case 2 : 
			TxMessage.Data[1] = CAN_LINE2_CMD_ID;
			break;
		case 3 :
			TxMessage.Data[1] = CAN_LINE3_CMD_ID;
			break;
		default:
			break;
		
	}
	TxMessage.Data[0] = 0x40;	
	CAN2PutDatatoTxBuf(&TxMessage);
	CAN2BeginSend();
	
}


/**************************************************************
** 函数名:SendLineCmdMsgByCan2
** 功能: 发送相关的命令
** 参数 :
** 注意事项:
***************************************************************/
void SendLineCmdMsgByCan2(UCHAR8 ucChan,UCHAR8 ucType)
{
	static CanTxMsg TxMessage = {0x00, 0x00, CAN_ID_STD, CAN_RTR_DATA, 1, 0,0,0,0,0,0,0,0};

	switch(ucChan)
	{
		case 0 : 
			TxMessage.StdId= CAN_LINE0_CMD_ID;
			break;
		case 1 : 
			TxMessage.StdId= CAN_LINE1_CMD_ID;
			break;
		case 2 : 
			TxMessage.StdId= CAN_LINE2_CMD_ID;
			break;
		case 3 :
			TxMessage.StdId= CAN_LINE3_CMD_ID;
			break;
		default:
			break;
		
	}
	
	TxMessage.Data[0] = ucType;
	CAN2PutDatatoTxBuf(&TxMessage);
	CAN2BeginSend();
	
}


/**************************************************************
** 函数名:SendLineCmdDataMsgByCan2
** 功能: 发送相关的命令及数据
** 参数 :
** 注意事项:
***************************************************************/
void SendLineCmdDataMsgByCan2(UCHAR8 ucChan,UCHAR8 ucData,UCHAR8 ucType)
{
	static CanTxMsg TxMessage = {0x00, 0x00, CAN_ID_STD, CAN_RTR_DATA, 2, 0,0,0,0,0,0,0,0};

	switch(ucChan)
	{
		case 0 : 
			TxMessage.StdId= CAN_LINE0_CMD_ID;
			break;
		case 1 : 
			TxMessage.StdId= CAN_LINE1_CMD_ID;
			break;
		case 2 : 
			TxMessage.StdId= CAN_LINE2_CMD_ID;
			break;
		case 3 :
			TxMessage.StdId= CAN_LINE3_CMD_ID;
			break;
		default:
			break;
		
	}
	
	TxMessage.Data[0] = ucType;
	TxMessage.Data[1] = ucData;
	CAN2PutDatatoTxBuf(&TxMessage);
	CAN2BeginSend();
	
}

/**************************************************************
** 函数名:UpdateLineValue
** 功能: 更新巡线相关的值
** 参数 :消息的指针
** 注意事项:
***************************************************************/
void  UpdateLineValue(const CanRxMsg * pRxMsg)
{
          UCHAR8 ucIndex = 0;
	   UCHAR8 ucTmp =0;
	   UINT32 uiTmp = 0;
	   UINT32 uiValue = 0;
		  
		switch(pRxMsg->StdId)
		{
			case CAN_LINE0_DATA_ID: 
				ucIndex = 0;
				break;
	              case CAN_LINE1_DATA_ID: 
				ucIndex = 1;
				break;
	             case CAN_LINE2_DATA_ID: 
				ucIndex = 2;
				break;
	             case CAN_LINE3_DATA_ID: 
				ucIndex = 3;
				break;
			default: break;
		}
		
	if(pRxMsg->DLC == 1)
	{
             g_ucLineCan[ucIndex] = pRxMsg->Data[0];
	}
	else if(pRxMsg->DLC == 2)
	{
               switch(pRxMsg->Data[0])
               {
			   	case 0x12:
					g_ucRetCalibrateCan[ucIndex] = pRxMsg->Data[1];
					break;
				case 0x13:
					g_ucRetSpeedCan[ucIndex] = pRxMsg->Data[1];
					break;
				case 0x14:
					g_ucRetSendIDCan[ucIndex] = pRxMsg->Data[1];
					break;
				case 0x15:
					g_ucRetRcvIDCan[ucIndex] = pRxMsg->Data[1];
					break;
				case 0x16:
					g_ucRetLightForceCan[ucIndex] = pRxMsg->Data[1];
					break;
				case 0x39:
					g_ucRetFieldInfoCan[ucIndex] = pRxMsg->Data[1];
					break;
				case 0x40:
					ucTmp = pRxMsg->Data[1];
					uiTmp = (UINT32) ucTmp;
					uiValue = g_uiLightStateCan[ucIndex] ;
					uiValue &= 0xFFFFFF00;
					uiValue = uiValue | uiTmp;
					g_uiLightStateCan[ucIndex] = uiValue;
					break;
				case 0x41:
					ucTmp = pRxMsg->Data[1];
					uiTmp = (UINT32) ucTmp;
					uiValue = g_uiLightStateCan[ucIndex] ;
					uiValue &= 0xFFFF00FF;
					uiValue = uiValue | (uiTmp<<8);
					g_uiLightStateCan[ucIndex] = uiValue;
					break;
				case 0x42:
					ucTmp = pRxMsg->Data[1];
					uiTmp = (UINT32) ucTmp;
					uiValue = g_uiLightStateCan[ucIndex] ;
					uiValue &= 0xFF00FFFF;
					uiValue = uiValue | (uiTmp<<16);
					g_uiLightStateCan[ucIndex] = uiValue;
					break;
				case 0x43:
					ucTmp = pRxMsg->Data[1];
					uiTmp = (UINT32) ucTmp;
					uiValue = g_uiLightStateCan[ucIndex] ;
					uiValue &= 0x00FFFFFF;
					uiValue = uiValue | (uiTmp<<24);
					g_uiLightStateCan[ucIndex] = uiValue;
					break;
				default:break;
               }
	}
	else
	{
	}
}


/**************************************************************
** 函数名:UpdateRadarValue
** 功能: 更新雷达变量的值
** 参数 :消息的指针
** 注意事项:
***************************************************************/
void  UpdateRadarValue(const CanRxMsg * pRxMsg)
{
    switch(pRxMsg->Data[0])
    {
		case 0x00:
			g_stRadarPosCan.Angle[0]=*((FP32*) &(pRxMsg->Data[1]));
	        g_stRadarPosCan.Distance[0]=*((SSHORT16*) &(pRxMsg->Data[5]));
			break;
		case 0x01:
			g_stRadarPosCan.Angle[1]=*((FP32*) &(pRxMsg->Data[1]));
	        g_stRadarPosCan.Distance[1]=*((SSHORT16*) &(pRxMsg->Data[5]));
			break;
		case 0xaa:
			g_stCalPosCan.usLeftDis = *((USHORT16*) &(pRxMsg->Data[1]));
			g_stCalPosCan.usRightDis = *((USHORT16*) &(pRxMsg->Data[3]));
			break;
		default:
			break;
    }
	

	
}



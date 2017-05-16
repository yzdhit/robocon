#include "IrComm.h"
#include "stdlib.h"
#include "stm32f4xx.h"
#include "HITCRT_RobotTypes.h"

static UCHAR8 IrComm1RxBuf[IR_COMM1_RX_BUF_MAX]; //队列
static UCHAR8 IrComm1TxBuf[IR_COMM1_TX_BUF_MAX];//队列

static UCHAR8 IrComm2RxBuf[IR_COMM2_RX_BUF_MAX];//队列
static UCHAR8 IrComm2TxBuf[IR_COMM2_TX_BUF_MAX];//队列

static ST_IR_MESSAGE Ir1MsgTxBuf[IR1_MSG_TX_BUF_MAX];
static ST_IR_MESSAGE Ir2MsgTxBuf[IR2_MSG_TX_BUF_MAX];

volatile UINT32 Ir1MsgTxHead = 0;
volatile UINT32 Ir1MsgTxTail = 0;

volatile UINT32 Ir2MsgTxHead = 0;
volatile UINT32 Ir2MsgTxTail = 0;

volatile UINT32 IrComm1RxHead = 0;//队列头-接收的时候移动
volatile UINT32 IrComm1RxTail= 0;//队列尾-读取的时候移动


volatile UINT32 IrComm1TxHead = 0;//队列头-接收的时候移动
volatile UINT32 IrComm1TxTail= 0;//队列尾-读取的时候移动


volatile UINT32 IrComm2TxHead = 0;//队列头-接收的时候移动
volatile UINT32 IrComm2TxTail= 0;//队列尾-读取的时候移动


volatile UINT32 IrComm2RxHead = 0;//队列头-接收的时候移动
volatile UINT32 IrComm2RxTail= 0;//队列尾-读取的时候移动


//通信相关的变量

ST_IR_COMM_OBJ * pstIr1Obj = NULL;
ST_IR_COMM_OBJ * pstIr2Obj = NULL;

ST_IR_OBJ_CENTER stIr1ObjCenter = {NULL,0};
ST_IR_OBJ_CENTER stIr2ObjCenter = {NULL,0};

UCHAR8 ucIr1Status = 0;
UCHAR8 ucIr2Status = 0;

UCHAR8 ucIr1CommRecCount = 0;
UCHAR8 ucIr2CommRecCount = 0;

UCHAR8 Ir1CommBuf[IR_COMM_BUF_LENGTH];
UCHAR8 Ir2CommBuf[IR_COMM_BUF_LENGTH];

//实际数据存放相关的变量
ST_IR_MSG_DATA stIR_M_TO_A = {0};
ST_IR_MSG_DATA stIR_M_TO_C = {0};
ST_IR_MSG_DATA stIR_A_TO_M = {0};
ST_IR_MSG_DATA stIR_A_TO_C = {0};
ST_IR_MSG_DATA stIR_C_TO_M = {0};
ST_IR_MSG_DATA stIR_C_TO_A = {0};

volatile UCHAR8 ucAck_M_TO_A_FLAG = 0;
volatile UCHAR8 ucAck_M_TO_C_FLAG = 0;
volatile UCHAR8 ucAck_A_TO_M_FLAG = 0;
volatile UCHAR8 ucAck_A_TO_C_FLAG = 0;
volatile UCHAR8 ucAck_C_TO_M_FLAG = 0;
volatile UCHAR8 ucAck_C_TO_A_FLAG = 0;


volatile UCHAR8 ucAck_M_TO_A_CNT = 0;
volatile UCHAR8 ucAck_M_TO_C_CNT = 0;
volatile UCHAR8 ucAck_A_TO_M_CNT = 0;
volatile UCHAR8 ucAck_A_TO_C_CNT = 0;
volatile UCHAR8 ucAck_C_TO_M_CNT = 0;
volatile UCHAR8 ucAck_C_TO_A_CNT = 0;

UCHAR8 ucAck_M_TO_A_SEND_DATA = IR_M_TO_A_ID;
UCHAR8 ucAck_M_TO_C_SEND_DATA = IR_M_TO_C_ID;
UCHAR8 ucAck_A_TO_M_SEND_DATA = IR_A_TO_M_ID;
UCHAR8 ucAck_A_TO_C_SEND_DATA = IR_A_TO_C_ID;
UCHAR8 ucAck_C_TO_M_SEND_DATA = IR_C_TO_M_ID;
UCHAR8 ucAck_C_TO_A_SEND_DATA  = IR_C_TO_A_ID;


UCHAR8 ucAck_M_TO_A_RCV_DATA = 0;
UCHAR8 ucAck_M_TO_C_RCV_DATA = 0;
UCHAR8 ucAck_A_TO_M_RCV_DATA =0;
UCHAR8 ucAck_A_TO_C_RCV_DATA = 0;
UCHAR8 ucAck_C_TO_M_RCV_DATA =0;
UCHAR8 ucAck_C_TO_A_RCV_DATA  = 0;

//////////////////////////////////////////////////////
//	以下函数为接收相关
/////////////////////////////////////////////////////
void IrComm1PutDataToRxBuf(UCHAR8 Data)
{
	UINT32 tmpHead;
	tmpHead = (IrComm1RxHead + 1)&IR_COMM1_RX_BUF_MARK;
	IrComm1RxHead = tmpHead;
	IrComm1RxBuf[tmpHead] = Data;
}


void IrComm2PutDataToRxBuf(UCHAR8 Data)
{
	UINT32 tmpHead;
	tmpHead = (IrComm2RxHead + 1)&IR_COMM2_RX_BUF_MARK;
	IrComm2RxHead = tmpHead;
	IrComm2RxBuf[tmpHead] = Data;
}

UCHAR8 IrComm1IsDataInRxBuf(void)
{
	if(IrComm1RxHead == IrComm1RxTail)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

UCHAR8 IrComm2IsDataInRxBuf(void)
{
	if(IrComm2RxHead == IrComm2RxTail)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


UINT32 IrComm1GetRxBufLen(void)
{
	if(IrComm1RxHead >= IrComm1RxTail)
	{
		return (IrComm1RxHead - IrComm1RxTail);
	}
	else
	{
		return  ( IR_COMM1_RX_BUF_MAX+IrComm1RxHead - IrComm1RxTail);
	}
}

UINT32 IrComm2GetRxBufLen(void)
{
	if(IrComm2RxHead >= IrComm2RxTail)
	{
		return (IrComm2RxHead - IrComm2RxTail);
	}
	else
	{
		return  ( IR_COMM2_RX_BUF_MAX+IrComm2RxHead - IrComm2RxTail);
	}
}


UCHAR8 IrComm1GetRxBufDat(void)
{
	UINT32 tmpTail;
	while ( IrComm1RxHead==  IrComm1RxTail);//为防止数据混乱而弄上的硬等待
	tmpTail = (IrComm1RxTail+1)&IR_COMM1_RX_BUF_MARK;
	IrComm1RxTail = tmpTail;
	return IrComm1RxBuf[tmpTail];
}


UCHAR8 IrComm2GetRxBufDat(void)
{
	UINT32 tmpTail;
	while ( IrComm2RxHead==  IrComm2RxTail);//为防止数据混乱而弄上的硬等待
	tmpTail = (IrComm2RxTail+1)&IR_COMM2_RX_BUF_MARK;
	IrComm2RxTail = tmpTail;
	return IrComm2RxBuf[tmpTail];
}

//////////////////////////////////////////
//	以下函数为发送相关
//////////////////////////////////////////

UCHAR8 IrComm1PutDataToTxBuf(UCHAR8 Data)
{
	UINT32 tmpHead;
	tmpHead = (IrComm1TxHead + 1)&IR_COMM1_TX_BUF_MARK;
	if(tmpHead == IrComm1TxTail)
	{
		return 0;
	}
      IrComm1TxHead  = tmpHead;
	IrComm1TxBuf[tmpHead] = Data;
	return 1;
	
}





UCHAR8 IrComm2PutDataToTxBuf(UCHAR8 Data)
{
	UINT32 tmpHead;
	tmpHead = (IrComm2TxHead + 1)&IR_COMM2_TX_BUF_MARK;
	if(tmpHead == IrComm2TxTail)
	{
		return 0;
	}
      IrComm2TxHead  = tmpHead;
	IrComm2TxBuf[tmpHead] = Data;
	return 1;
	
}

UCHAR8 IrComm1IsDataInTxBuf(void)
{
	if(IrComm1TxHead == IrComm1TxTail)
	{
		return 0;
	}
	else
	{
		return 1;
	}
		
}


UCHAR8 IrComm2IsDataInTxBuf(void)
{
	if(IrComm2TxHead == IrComm2TxTail)
	{
		return 0;
	}
	else
	{
		return 1;
	}
		
}

UINT32 IrComm1GetTxBufLen(void)
{
	if(IrComm1TxHead >= IrComm1TxTail)
	{
		return (IR_COMM1_TX_BUF_MAX - (IrComm1TxHead -IrComm1TxTail));
	}
	else
	{
		 return  (IrComm1TxTail - IrComm1TxHead);
	}
}


UINT32 IrComm2GetTxBufLen(void)
{
	if(IrComm2TxHead >= IrComm2TxTail)
	{
		return (IR_COMM2_TX_BUF_MAX - (IrComm2TxHead -IrComm2TxTail));
	}
	else
	{
		 return  (IrComm2TxTail - IrComm2TxHead);
	}
}

UCHAR8 IrComm1GetTxBufDat(void)
{
	UINT32 tmpTail;
	while(IrComm1TxHead == IrComm1TxTail);
	tmpTail = (IrComm1TxTail+1)&IR_COMM1_TX_BUF_MARK;
	IrComm1TxTail = tmpTail;
	return IrComm1TxBuf[tmpTail];
}


UCHAR8 IrComm2GetTxBufDat(void)
{
	UINT32 tmpTail;
	while(IrComm2TxHead == IrComm2TxTail);
	tmpTail = (IrComm2TxTail+1)&IR_COMM2_TX_BUF_MARK;
	IrComm2TxTail = tmpTail;
	return IrComm2TxBuf[tmpTail];
}



void SendHeadByIrComm1(UCHAR8 ucID)
{
	SEND_BYTE_BY_IR_COMM1(HEAD_BYTE);
	SEND_BYTE_BY_IR_COMM1(ucID);
}


void SendHeadByIrComm2(UCHAR8 ucID)
{
	SEND_BYTE_BY_IR_COMM2(HEAD_BYTE);
	SEND_BYTE_BY_IR_COMM2(ucID);
}

void SendTailByIrComm1(UCHAR8 ucSum)
{
	SEND_BYTE_BY_IR_COMM1(ucSum);
	SEND_BYTE_BY_IR_COMM1(TAIL_BYTE);
}

void SendTailByIrComm2(UCHAR8 ucSum)
{
	SEND_BYTE_BY_IR_COMM2(ucSum);
	SEND_BYTE_BY_IR_COMM2(TAIL_BYTE);
}

void SendVariableByIrComm1(UCHAR8 * pVar,UINT32 uiLength)
{
	int i = 0;
	for(i = 0;i<uiLength;i++)
	{
		SEND_BYTE_BY_IR_COMM1((*(pVar+i)));
	}
}


void SendVariableByIrComm2(UCHAR8 * pVar,UINT32 uiLength)
{
	int i = 0;
	for(i = 0;i<uiLength;i++)
	{
		SEND_BYTE_BY_IR_COMM2((*(pVar+i)));
	}
}

void SendStructByIrComm1(UCHAR8 ucID,void* pVarAdd,UINT32 uiLength)
{
	UINT32  i = 0;
	UCHAR8 ucSum = 0;
	UCHAR8 ucTmp;
	SEND_BYTE_BY_IR_COMM1(HEAD_BYTE);
	SEND_BYTE_BY_IR_COMM1(ucID);
	for(i = 0;i<uiLength;i++)
	{
		ucTmp = *(((UCHAR8 *)pVarAdd)+i);
		SEND_BYTE_BY_IR_COMM1(ucTmp);
		ucSum +=ucTmp;//和校验
	}
	SEND_BYTE_BY_IR_COMM1(ucSum);
       SEND_BYTE_BY_IR_COMM1(TAIL_BYTE);
	
}

void SendStructByIrComm2(UCHAR8 ucID,void* pVarAdd,UINT32 uiLength)
{
	UINT32  i = 0;
	UCHAR8 ucSum = 0;
	UCHAR8 ucTmp;
	SEND_BYTE_BY_IR_COMM2(HEAD_BYTE);
	SEND_BYTE_BY_IR_COMM2(ucID);
	for(i = 0;i<uiLength;i++)
	{
		ucTmp = *(((UCHAR8 *)pVarAdd)+i);
		SEND_BYTE_BY_IR_COMM2(ucTmp);
		ucSum +=ucTmp;//和校验
	}
	SEND_BYTE_BY_IR_COMM2(ucSum);
       SEND_BYTE_BY_IR_COMM2(TAIL_BYTE);
	
}



void CreateIr1CommObj(UCHAR8 ucID,void * pAdd, UINT32 uiLength)
{
	ST_IR_COMM_OBJ * pObj = stIr1ObjCenter.pCommObjHeader;
	if(stIr1ObjCenter.pCommObjHeader == NULL)
	{
		pObj = (ST_IR_COMM_OBJ *)malloc(sizeof(ST_IR_COMM_OBJ));
		stIr1ObjCenter.pCommObjHeader = pObj;
		pObj->ucID = ucID;
		pObj->pVarAddress = pAdd;
		pObj->uiLength = uiLength;
		pObj->pNextVar = NULL;
		pObj->ucSum = 0;
		stIr1ObjCenter.uiObjCount = 1;
		return;
	}

	while(pObj->pNextVar !=NULL)
	{
		pObj = (ST_IR_COMM_OBJ *) (pObj->pNextVar);
	}
	pObj->pNextVar = (ST_IR_COMM_OBJ *)malloc(sizeof(ST_IR_COMM_OBJ));
	pObj = (ST_IR_COMM_OBJ *) (pObj->pNextVar);
	pObj->ucID = ucID;
	pObj->pVarAddress = pAdd;
	pObj->uiLength = uiLength;
	pObj->pNextVar = NULL;
	pObj->ucSum = 0;
	stIr1ObjCenter.uiObjCount++;
	
}

void CreateIr2CommObj(UCHAR8 ucID,void * pAdd, UINT32 uiLength)
{
	ST_IR_COMM_OBJ * pObj = stIr2ObjCenter.pCommObjHeader;
	if(stIr2ObjCenter.pCommObjHeader == NULL)
	{
		pObj = (ST_IR_COMM_OBJ *)malloc(sizeof(ST_IR_COMM_OBJ));
		stIr2ObjCenter.pCommObjHeader = pObj;
		pObj->ucID = ucID;
		pObj->pVarAddress = pAdd;
		pObj->uiLength = uiLength;
		pObj->pNextVar = NULL;
		pObj->ucSum = 0;
		stIr2ObjCenter.uiObjCount = 1;
		return;
	}

	while(pObj->pNextVar !=NULL)
	{
		pObj = (ST_IR_COMM_OBJ *) (pObj->pNextVar);
	}
	pObj->pNextVar = (ST_IR_COMM_OBJ *)malloc(sizeof(ST_IR_COMM_OBJ));
	pObj = (ST_IR_COMM_OBJ *) (pObj->pNextVar);
	pObj->ucID = ucID;
	pObj->pVarAddress = pAdd;
	pObj->uiLength = uiLength;
	pObj->pNextVar = NULL;
	pObj->ucSum = 0;
	stIr2ObjCenter.uiObjCount++;
	
}



void Ir1CommuRcv_Handle(void)
{
     UINT32  i = 0;
     UCHAR8 ucTmp = 0;
     static UCHAR8 cnt = 0;
     UCHAR8 ucSum = 0;
     ST_IR_COMM_OBJ * pObj = NULL;
     while(IrComm1IsDataInRxBuf())
     {
	 	ucTmp = IrComm1GetRxBufDat();

		switch(ucIr1Status)
		{
			case IR_COMM_STATUS_IDLE:
				if(ucTmp == HEAD_BYTE)
				{
					ucIr1Status = IR_COMM_STATUS_HEADER_FOUND;
					Ir1CommBuf[0] = ucTmp;
					ucIr1CommRecCount = 1;
				}
				break;
			case IR_COMM_STATUS_HEADER_FOUND:
				pObj = stIr1ObjCenter.pCommObjHeader;
				while(pObj != NULL)
				{
					if(pObj->ucID == ucTmp)
					{
						Ir1CommBuf[1] = ucTmp;
						ucIr1CommRecCount = 2;
						ucIr1Status = IR_COMM_STATUS_ID_FOUND;
						pObj->ucSum = 0;
						pstIr1Obj = pObj;
						break;
					}
					pObj = pObj->pNextVar;
				}

				if(pObj == NULL)
				{
					ucIr1Status = IR_COMM_STATUS_IDLE;
					ucIr1CommRecCount = 0;
				}
				break;
			case IR_COMM_STATUS_ID_FOUND:
				Ir1CommBuf[ucIr1CommRecCount] = ucTmp;
				ucIr1CommRecCount++;
				if(ucIr1CommRecCount == pstIr1Obj->uiLength + 4)
				{
					//校验和的验证及计算
					for(i= 0;i<pstIr1Obj->uiLength;i++)
					{
						pstIr1Obj->ucSum += Ir1CommBuf[i+2];
					}

					if(pstIr1Obj->ucSum == Ir1CommBuf[pstIr1Obj->uiLength + 4 -2] &&Ir1CommBuf[pstIr1Obj->uiLength + 4 -1] == TAIL_BYTE)
					{
						for(i= 0;i<pstIr1Obj->uiLength;i++)
					       {
						      *(((UCHAR8 *)(pstIr1Obj->pVarAddress)) + i)= Ir1CommBuf[i+2];
					       }

						//如果前面发送的为数据帧，则发送应答帧
						switch(pstIr1Obj->ucID)
						{
							//在此发送应答帧
							case IR_A_TO_C_ID :
								SEND_STRUCT_BY_IR_COMM1(IR_C_TO_A_ACK_ID, ucAck_C_TO_A_SEND_DATA);
								break;
							case IR_A_TO_M_ID:
								SEND_STRUCT_BY_IR_COMM1(IR_M_TO_A_ACK_ID, ucAck_M_TO_A_SEND_DATA);
								break;
							case IR_M_TO_A_ID:
								SEND_STRUCT_BY_IR_COMM1(IR_A_TO_M_ACK_ID, ucAck_A_TO_M_SEND_DATA);
								break;
							case IR_M_TO_C_ID:
								SEND_STRUCT_BY_IR_COMM1(IR_C_TO_M_ACK_ID, ucAck_C_TO_M_SEND_DATA);
								break;
							case IR_C_TO_M_ID:
								SEND_STRUCT_BY_IR_COMM1(IR_M_TO_C_ACK_ID, ucAck_M_TO_C_SEND_DATA);
								break;
							case IR_C_TO_A_ID:
								SEND_STRUCT_BY_IR_COMM1(IR_A_TO_C_ACK_ID, ucAck_A_TO_C_SEND_DATA);
								break;
							default:
								break;
						}

						//如果前面发送的为应答帧，则改变接受标志量
						switch(pstIr1Obj->ucID)
						{
							case IR_A_TO_C_ACK_ID:
								if(ucAck_A_TO_C_RCV_DATA == ucAck_A_TO_C_SEND_DATA)
								{
									ucAck_C_TO_A_FLAG = 1;
								}
								break;
							case IR_A_TO_M_ACK_ID:
								if(ucAck_A_TO_M_RCV_DATA == ucAck_A_TO_M_SEND_DATA)
								{
									ucAck_M_TO_A_FLAG = 1;
								}
								break;
							case IR_M_TO_A_ACK_ID:
								if(ucAck_M_TO_A_RCV_DATA == ucAck_M_TO_A_SEND_DATA)
								{
									ucAck_A_TO_M_FLAG = 1;
								}
								break;
							case IR_M_TO_C_ACK_ID:
								if(ucAck_M_TO_C_RCV_DATA == ucAck_M_TO_C_SEND_DATA)
								{
									ucAck_C_TO_M_FLAG = 1;
								}
								break;
							case IR_C_TO_M_ACK_ID:
								if(ucAck_C_TO_M_RCV_DATA == ucAck_C_TO_M_SEND_DATA)
								{
									ucAck_M_TO_C_FLAG = 1;
								}
								break;
							case IR_C_TO_A_ACK_ID:
								if(ucAck_C_TO_A_RCV_DATA == ucAck_C_TO_A_SEND_DATA)
								{
									ucAck_A_TO_C_FLAG = 1;
								}
								break;
							default:
								break;
						}

						
						ucIr1Status = IR_COMM_STATUS_IDLE;
						ucIr1CommRecCount = 0;
						pstIr1Obj->ucSum = 0;
						pstIr1Obj = NULL;
						break;
						
					}
					else
					{
						ucIr1Status = IR_COMM_STATUS_IDLE;
						ucIr1CommRecCount = 0;
						pstIr1Obj->ucSum = 0;
						pstIr1Obj = NULL;
					}
				}


				if(ucIr1CommRecCount >= IR_COMM_BUF_LENGTH)
				{
					      ucIr1Status = IR_COMM_STATUS_IDLE;
						ucIr1CommRecCount = 0;
						pstIr1Obj->ucSum = 0;
						pstIr1Obj = NULL;
						break;
				}
				break;
			default:
				break;
			
		}
     }
}



void Ir2CommuRcv_Handle(void)
{
     UINT32  i = 0;
     UCHAR8 ucTmp = 0;
     static UCHAR8 cnt = 0;
     UCHAR8 ucSum = 0;
     ST_IR_COMM_OBJ * pObj = NULL;
     while(IrComm2IsDataInRxBuf())
     {
	 	ucTmp = IrComm2GetRxBufDat();

		switch(ucIr2Status)
		{
			case IR_COMM_STATUS_IDLE:
				if(ucTmp == HEAD_BYTE)
				{
					ucIr2Status = IR_COMM_STATUS_HEADER_FOUND;
					Ir2CommBuf[0] = ucTmp;
					ucIr2CommRecCount = 1;
				}
				break;
			case IR_COMM_STATUS_HEADER_FOUND:
				pObj = stIr2ObjCenter.pCommObjHeader;
				while(pObj != NULL)
				{
					if(pObj->ucID == ucTmp)
					{
						Ir2CommBuf[1] = ucTmp;
						ucIr2CommRecCount = 2;
						ucIr2Status = IR_COMM_STATUS_ID_FOUND;
						pObj->ucSum = 0;
						pstIr2Obj = pObj;
						break;
					}
					pObj = pObj->pNextVar;
				}

				if(pObj == NULL)
				{
					ucIr2Status = IR_COMM_STATUS_IDLE;
					ucIr2CommRecCount = 0;
				}
				break;
			case IR_COMM_STATUS_ID_FOUND:
				Ir2CommBuf[ucIr2CommRecCount] = ucTmp;
				ucIr2CommRecCount++;
				if(ucIr2CommRecCount == pstIr2Obj->uiLength + 4)
				{
					//校验和的验证及计算
					for(i= 0;i<pstIr2Obj->uiLength;i++)
					{
						pstIr2Obj->ucSum += Ir2CommBuf[i+2];
					}

					if(pstIr2Obj->ucSum == Ir2CommBuf[pstIr2Obj->uiLength + 4 -2] &&Ir2CommBuf[pstIr2Obj->uiLength + 4 -1] == TAIL_BYTE)
					{
						for(i= 0;i<pstIr2Obj->uiLength;i++)
					       {
						      *(((UCHAR8 *)(pstIr2Obj->pVarAddress)) + i)= Ir2CommBuf[i+2];
					       }
						//如果前面发送的为数据帧，则发送应答帧
						switch(pstIr2Obj->ucID)
						{
							//在此发送应答帧
							case IR_A_TO_C_ID :
								SEND_STRUCT_BY_IR_COMM2(IR_C_TO_A_ACK_ID, ucAck_C_TO_A_SEND_DATA);
								break;
							case IR_A_TO_M_ID:
								SEND_STRUCT_BY_IR_COMM2(IR_M_TO_A_ACK_ID, ucAck_M_TO_A_SEND_DATA);
								break;
							case IR_M_TO_A_ID:
								SEND_STRUCT_BY_IR_COMM2(IR_A_TO_M_ACK_ID, ucAck_A_TO_M_SEND_DATA);
								break;
							case IR_M_TO_C_ID:
								SEND_STRUCT_BY_IR_COMM2(IR_C_TO_M_ACK_ID, ucAck_C_TO_M_SEND_DATA);
								break;
							case IR_C_TO_M_ID:
								SEND_STRUCT_BY_IR_COMM2(IR_M_TO_C_ACK_ID, ucAck_M_TO_C_SEND_DATA);
								break;
							case IR_C_TO_A_ID:
								SEND_STRUCT_BY_IR_COMM2(IR_A_TO_C_ACK_ID, ucAck_A_TO_C_SEND_DATA);
								break;
							default:
								break;
						}

						//如果前面发送的为应答帧，则改变接受标志量
						switch(pstIr2Obj->ucID)
						{
							case IR_A_TO_C_ACK_ID:
								if(ucAck_A_TO_C_RCV_DATA == ucAck_A_TO_C_SEND_DATA)
								{
									ucAck_C_TO_A_FLAG = 1;
								}
								break;
							case IR_A_TO_M_ACK_ID:
								if(ucAck_A_TO_M_RCV_DATA == ucAck_A_TO_M_SEND_DATA)
								{
									ucAck_M_TO_A_FLAG = 1;
								}
								break;
							case IR_M_TO_A_ACK_ID:
								if(ucAck_M_TO_A_RCV_DATA == ucAck_M_TO_A_SEND_DATA)
								{
									ucAck_A_TO_M_FLAG = 1;
								}
								break;
							case IR_M_TO_C_ACK_ID:
								if(ucAck_M_TO_C_RCV_DATA == ucAck_M_TO_C_SEND_DATA)
								{
									ucAck_C_TO_M_FLAG = 1;
								}
								break;
							case IR_C_TO_M_ACK_ID:
								if(ucAck_C_TO_M_RCV_DATA == ucAck_C_TO_M_SEND_DATA)
								{
									ucAck_M_TO_C_FLAG = 1;
								}
								break;
							case IR_C_TO_A_ACK_ID:
								if(ucAck_C_TO_A_RCV_DATA == ucAck_C_TO_A_SEND_DATA)
								{
									ucAck_A_TO_C_FLAG = 1;
								}
								break;
							default:
								break;
						}
						ucIr2Status = IR_COMM_STATUS_IDLE;
						ucIr2CommRecCount = 0;
						pstIr2Obj->ucSum = 0;
						pstIr2Obj = NULL;
						break;
						
					}
					else
					{
						ucIr2Status = IR_COMM_STATUS_IDLE;
						ucIr2CommRecCount = 0;
						pstIr2Obj->ucSum = 0;
						pstIr2Obj = NULL;
					}
				}


				if(ucIr2CommRecCount >= IR_COMM_BUF_LENGTH)
				{
					      ucIr2Status = IR_COMM_STATUS_IDLE;
						ucIr2CommRecCount = 0;
						pstIr2Obj->ucSum = 0;
						pstIr2Obj = NULL;
						break;
				}
				break;
			default:
				break;
			
		}
     }
}


void InitIrComm1(void)
{
	stIr1ObjCenter.pCommObjHeader = NULL;
	stIr1ObjCenter.uiObjCount = 0;
}


void InitIrComm2(void)
{
	stIr2ObjCenter.pCommObjHeader = NULL;
	stIr2ObjCenter.uiObjCount = 0;
}


void Ir1MsgPutDataToTxBuf(ST_IR_MESSAGE * pIRMessage)
{
	UINT32 tmpHead;
	tmpHead = (Ir1MsgTxHead + 1)&IR1_MSG_TX_BUF_MARK;
	Ir1MsgTxHead = tmpHead;
	Ir1MsgTxBuf[tmpHead] = * pIRMessage;
}


void Ir2MsgPutDataToTxBuf(ST_IR_MESSAGE * pIRMessage)
{
	UINT32 tmpHead;
	tmpHead = (Ir2MsgTxHead + 1)&IR2_MSG_TX_BUF_MARK;
	Ir2MsgTxHead = tmpHead;
	Ir2MsgTxBuf[tmpHead] = * pIRMessage;
}


UCHAR8 Ir1MsgIsDataInTxBuf(void)
{
	if(Ir1MsgTxHead == Ir1MsgTxTail)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

UCHAR8 Ir2MsgIsDataInTxBuf(void)
{
	if(Ir2MsgTxHead == Ir2MsgTxTail)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

UINT32 Ir1MsgGetTxBufLen(void)
{
	if(Ir1MsgTxHead >= Ir1MsgTxTail)
	{
		return (IR1_MSG_TX_BUF_MAX - (Ir1MsgTxHead - Ir1MsgTxTail));
	}
	else
	{
		return (Ir1MsgTxTail - Ir1MsgTxHead);
	}
}

UINT32 Ir2MsgGetTxBufLen(void)
{
	if(Ir2MsgTxHead >= Ir2MsgTxTail)
	{
		return (IR2_MSG_TX_BUF_MAX - (Ir2MsgTxHead - Ir2MsgTxTail));
	}
	else
	{
		return (Ir2MsgTxTail - Ir2MsgTxHead);
	}
}


ST_IR_MESSAGE * Ir1MsgGetTxBufDat(void)
{
	UINT32 tmpTail;
	while(Ir1MsgTxHead == Ir1MsgTxTail);
	tmpTail = (Ir1MsgTxTail + 1) & IR1_MSG_TX_BUF_MARK;
	Ir1MsgTxTail = tmpTail;
	return &(Ir1MsgTxBuf[tmpTail]);
}


ST_IR_MESSAGE * Ir2MsgGetTxBufDat(void)
{
	UINT32 tmpTail;
	while(Ir2MsgTxHead == Ir2MsgTxTail);
	tmpTail = (Ir2MsgTxTail + 1) & IR2_MSG_TX_BUF_MARK;
	Ir2MsgTxTail = tmpTail;
	return &(Ir2MsgTxBuf[tmpTail]);
}

UINT32  IR1_TransmitData(UCHAR8 ucID, ST_IR_MSG_DATA stDat)
{
	ST_IR_MESSAGE stTmpMessage;
	stTmpMessage.ucID = ucID;
	stTmpMessage.stData = stDat;
	if(Ir1MsgGetTxBufLen() > 0)
	{
		Ir1MsgPutDataToTxBuf(&stTmpMessage);
		return 1;
	}
	else
	{
		return 0;
	}
}


UINT32  IR2_TransmitData(UCHAR8 ucID, ST_IR_MSG_DATA stDat)
{
	ST_IR_MESSAGE stTmpMessage;
	stTmpMessage.ucID = ucID;
	stTmpMessage.stData = stDat;
	if(Ir2MsgGetTxBufLen() > 0)
	{
		Ir2MsgPutDataToTxBuf(&stTmpMessage);
		return 1;
	}
	else
	{
		return 0;
	}
}

void SendIrByteByUart1(UCHAR8 Data)
{
	USART_SendData(USART1, Data);
       while(USART_GetFlagStatus(USART1 ,USART_FLAG_TC ) != SET);
	USART_ClearFlag(USART1 ,USART_FLAG_TC);
}


void SendIrByteByUart2(UCHAR8 Data)
{
	USART_SendData(USART2, Data);
       while(USART_GetFlagStatus(USART2 ,USART_FLAG_TC ) != SET);
	USART_ClearFlag(USART2 ,USART_FLAG_TC);
}

void IrTransmitByteByUart2(UCHAR8 Data)
{
    UINT32 i = 0;
	for(i = 0; i < 20; i++)
	{
	    USART_SendData(USART2, Data);
	    while(USART_GetFlagStatus(USART2 ,USART_FLAG_TC ) != SET);
		USART_ClearFlag(USART2 ,USART_FLAG_TC);
	}
}





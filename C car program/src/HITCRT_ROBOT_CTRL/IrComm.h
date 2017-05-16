#ifndef IR_COMM_H_
#define IR_COMM_H_

#include "stm32f4xx.h"
#include "HITCRT_RobotTypes.h"

//红外串口接收队列相关定义
#define IR_COMM1_RX_BUF_MAX   256
#define IR_COMM1_RX_BUF_MARK (IR_COMM1_RX_BUF_MAX -1)
#define IR_COMM2_RX_BUF_MAX   16
#define IR_COMM2_RX_BUF_MARK (IR_COMM2_RX_BUF_MAX -1)

//红外串口发送队列相关定义
#define IR_COMM1_TX_BUF_MAX   16
#define IR_COMM1_TX_BUF_MARK (IR_COMM1_TX_BUF_MAX -1)
#define IR_COMM2_TX_BUF_MAX   16
#define IR_COMM2_TX_BUF_MARK (IR_COMM2_TX_BUF_MAX -1)

#define IR1_MSG_TX_BUF_MAX  4
#define IR1_MSG_TX_BUF_MARK  (IR1_MSG_TX_BUF_MAX  - 1)
#define IR2_MSG_TX_BUF_MAX  4
#define IR2_MSG_TX_BUF_MARK  (IR2_MSG_TX_BUF_MAX  - 1)


//红外发送ID的定义
#define IR_M_TO_C_ID   0x46
#define IR_C_TO_M_ID   0x64

#define IR_A_TO_C_ID   0x56
#define IR_C_TO_A_ID   0x65

#define IR_M_TO_A_ID  0x45
#define IR_A_TO_M_ID  0x54


#define IR_M_TO_C_ACK_ID   0x79
#define IR_C_TO_M_ACK_ID   0x97

#define IR_A_TO_C_ACK_ID   0x89
#define IR_C_TO_A_ACK_ID   0x98

#define IR_M_TO_A_ACK_ID  0x78
#define IR_A_TO_M_ACK_ID  0x87



#define IR_ACK_MAX_CNT 3
#define IR_MAX_SEND_CNT 3




typedef struct
{
	UCHAR8 ucID;//改变了的id
	void * pVarAddress;//结构体变量的地址
	UINT32 uiLength;//对结构体变量求sizeof的大小
	UCHAR8 ucSum;
	void * pNextVar; //指向下一个变量的
}ST_IR_COMM_OBJ; //红外通信对象结构体

typedef struct
{
	ST_IR_COMM_OBJ *pCommObjHeader; //中心对象的头
	UINT32 uiObjCount; //通信对象的个数
}ST_IR_OBJ_CENTER;// 通信对象管理中心

typedef struct
{
	UCHAR8 ucData[8];
}ST_IR_MSG_DATA; //发送消息内容的结构体

typedef struct
{
	UCHAR8 ucID;
	ST_IR_MSG_DATA stData;  
}ST_IR_MESSAGE;//红外通信发送消息结构体




#define HEAD_BYTE 0xaa  //协议规定的头
#define TAIL_BYTE 0x55 //协议规定的尾

#define IR_COMM_STATUS_IDLE (0)
#define IR_COMM_STATUS_HEADER_FOUND (1)
#define IR_COMM_STATUS_ID_FOUND (2)

#define IR_COMM_BUF_LENGTH (128)


//发送相关的宏定义

#define SEND_BYTE_BY_IR_COMM1(Data)    (SendIrByteByUart1(Data))
#define SEND_BYTE_BY_IR_COMM2(Data)    (SendIrByteByUart2(Data))


#define SEND_STRUCT_BY_IR_COMM1(ucID,Var)  (SendStructByIrComm1(ucID, &(Var), sizeof(Var)))
#define SEND_STRUCT_BY_IR_COMM2(ucID,Var)   (SendStructByIrComm2(ucID, &(Var), sizeof(Var)))

//声明变量
#define DECLARE_IR_COMM1_OBJS(ucID, Data)  (CreateIr1CommObj(ucID, &Data, sizeof(Data)))
#define DECLARE_IR_COMM2_OBJS(ucID, Data)  (CreateIr2CommObj(ucID, &Data, sizeof(Data)))


//外部使用的变量
extern  ST_IR_MSG_DATA stIR_M_TO_A;
extern  ST_IR_MSG_DATA stIR_M_TO_C;
extern  ST_IR_MSG_DATA stIR_A_TO_M;
extern ST_IR_MSG_DATA stIR_A_TO_C;
extern ST_IR_MSG_DATA stIR_C_TO_M;
extern ST_IR_MSG_DATA stIR_C_TO_A;

extern volatile UCHAR8 ucAck_M_TO_A_FLAG ;
extern volatile UCHAR8 ucAck_M_TO_C_FLAG ;
extern volatile UCHAR8 ucAck_A_TO_M_FLAG;
extern volatile UCHAR8 ucAck_A_TO_C_FLAG ;
extern volatile UCHAR8 ucAck_C_TO_M_FLAG ;
extern volatile UCHAR8 ucAck_C_TO_A_FLAG;


extern volatile UCHAR8 ucAck_M_TO_A_CNT ;
extern volatile UCHAR8 ucAck_M_TO_C_CNT ;
extern volatile UCHAR8 ucAck_A_TO_M_CNT ;
extern volatile UCHAR8 ucAck_A_TO_C_CNT ;
extern volatile UCHAR8 ucAck_C_TO_M_CNT ;
extern volatile UCHAR8 ucAck_C_TO_A_CNT ;


extern UCHAR8 ucAck_M_TO_A_SEND_DATA;
extern UCHAR8 ucAck_M_TO_C_SEND_DATA;
extern UCHAR8 ucAck_A_TO_M_SEND_DATA;
extern UCHAR8 ucAck_A_TO_C_SEND_DATA;
extern UCHAR8 ucAck_C_TO_M_SEND_DATA;
extern UCHAR8 ucAck_C_TO_A_SEND_DATA;


extern UCHAR8 ucAck_M_TO_A_RCV_DATA;
extern UCHAR8 ucAck_M_TO_C_RCV_DATA;
extern UCHAR8 ucAck_A_TO_M_RCV_DATA;
extern UCHAR8 ucAck_A_TO_C_RCV_DATA;
extern  UCHAR8 ucAck_C_TO_M_RCV_DATA;
extern UCHAR8 ucAck_C_TO_A_RCV_DATA;

//队列操作的相关函数
extern void IrTransmitByteByUart2(UCHAR8 Data);
extern void IrComm1PutDataToRxBuf(UCHAR8 Data);
extern void IrComm2PutDataToRxBuf(UCHAR8 Data);
extern 	UCHAR8 IrComm1IsDataInRxBuf(void);
extern 	UCHAR8 IrComm2IsDataInRxBuf(void);
extern 	UINT32 IrComm1GetRxBufLen(void);
extern 	UINT32 IrComm2GetRxBufLen(void);
extern 	UCHAR8 IrComm1GetRxBufDat(void);
extern 	UCHAR8 IrComm2GetRxBufDat(void);
extern 	UCHAR8 IrComm1PutDataToTxBuf(UCHAR8 Data);
extern 	UCHAR8 IrComm2PutDataToTxBuf(UCHAR8 Data);
extern 	UCHAR8 IrComm1IsDataInTxBuf(void);
extern 	UCHAR8 IrComm2IsDataInTxBuf(void);
extern 	UINT32 IrComm1GetTxBufLen(void);
extern 	UINT32 IrComm2GetTxBufLen(void);
extern 	UCHAR8 IrComm1GetTxBufDat(void);
extern 	UCHAR8 IrComm2GetTxBufDat(void);
	
//接受相关的函数
extern void SendHeadByIrComm1(UCHAR8 ucID);
extern void SendHeadByIrComm2(UCHAR8 ucID);
extern void SendTailByIrComm1(UCHAR8 ucSum);
extern void SendTailByIrComm2(UCHAR8 ucSum);
extern void SendVariableByIrComm1(UCHAR8 * pVar,UINT32 uiLength);
extern void SendVariableByIrComm2(UCHAR8 * pVar,UINT32 uiLength);
extern void SendStructByIrComm1(UCHAR8 ucID,void* pVarAdd,UINT32 uiLength);
extern void SendStructByIrComm2(UCHAR8 ucID,void* pVarAdd,UINT32 uiLength);

extern void InitIrComm1(void);
extern void InitIrComm2(void);	
extern void CreateIr1CommObj(UCHAR8 ucID,void * pAdd, UINT32 uiLength);
extern void CreateIr2CommObj(UCHAR8 ucID,void * pAdd, UINT32 uiLength);
extern void Ir1CommuRcv_Handle(void);
extern void Ir2CommuRcv_Handle(void);


//消息队列相关的函数
extern void Ir1MsgPutDataToTxBuf(ST_IR_MESSAGE * pIRMessage);
extern void Ir2MsgPutDataToTxBuf(ST_IR_MESSAGE * pIRMessage);
extern UCHAR8 Ir1MsgIsDataInTxBuf(void);
extern UCHAR8 Ir2MsgIsDataInTxBuf(void);
extern UINT32 Ir1MsgGetTxBufLen(void);
extern UINT32 Ir2MsgGetTxBufLen(void);
extern ST_IR_MESSAGE * Ir1MsgGetTxBufDat(void);
extern ST_IR_MESSAGE * Ir2MsgGetTxBufDat(void);
extern UINT32  IR1_TransmitData(UCHAR8 ucID, ST_IR_MSG_DATA stDat);
extern UINT32  IR2_TransmitData(UCHAR8 ucID, ST_IR_MSG_DATA stDat);

// 串口发送的相关函数
extern void SendIrByteByUart1(UCHAR8 Data);
extern void SendIrByteByUart2(UCHAR8 Data);




	



#endif

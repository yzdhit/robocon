#ifndef CAN_H
#define CAN_H
#include "stm32f4xx.h"
#define CAN1_RX_BUF_MAX 64/*必须是2的幂*/
#define CAN1_TX_BUF_MAX 64/*必须是2的幂*/
#define CAN1_RX_BUF_MARK	(CAN1_RX_BUF_MAX-1)
#define CAN1_TX_BUF_MARK	(CAN1_TX_BUF_MAX-1)

#define CAN2_RX_BUF_MAX 64/*必须是2的幂*/
#define CAN2_TX_BUF_MAX 64/*必须是2的幂*/
#define CAN2_RX_BUF_MARK	(CAN2_RX_BUF_MAX-1)
#define CAN2_TX_BUF_MARK	(CAN2_TX_BUF_MAX-1)

//extern CanTxMsg TxMessage;//要发送的信息，全局变量

/**************************************************************
** 函数名:CAN1PutDatatoRxBuf
** 功能:把数据放进队列中,
** 注意事项:应该在串口接收中断中调用此函数
***************************************************************/
extern void CAN1PutDatatoRxBuf(CanRxMsg *RxMessage);
extern void CAN2PutDatatoRxBuf(CanRxMsg *RxMessage);
/*************************************************
**函数名:CAN1IsDataInRxBuf
**功能:获知缓冲中是否有数据
**注意事项:当队列的头和尾不相等的时候,就代表了有数据在缓冲中
*************************************************/
extern bool CAN1IsDataInRxBuf( void );
extern bool CAN2IsDataInRxBuf( void );
/*************************************************
**函数名:CAN1GetRxBufLen
**功能:获取缓冲中有效数据的长度
**注意事项:获取的值必然为最小值,因为真实长度会不断变化.由于是32位的ARM系统,获取32位数据不存在临界问题,所以可以不考虑关中断
**************************************************/
extern u32 CAN1GetRxBufLen(void);
extern u32 CAN2GetRxBufLen(void);
/**************************************************
**函数名:CAN1GetRxBufDat
**功能:从队列中获取数据
**注意事项:调用此函数前请先确保队列中有数据!!否则会陷入硬等待
**************************************************/
extern CanRxMsg *CAN1GetRxBufDat( void );
extern CanRxMsg *CAN2GetRxBufDat( void );
//////////////////////////////////////////
//	以下函数为发送相关
//////////////////////////////////////////
/**************************************************************
** 函数名:CAN1PutDatatoTxBuf
** 功能:把数据放进发送队列中,
** 注意事项:用户需要有数据发的时候使用
***************************************************************/
extern bool CAN1PutDatatoTxBuf(CanTxMsg *TxMessage);
extern bool CAN2PutDatatoTxBuf(CanTxMsg *TxMessage);
/*************************************************
**函数名:CAN1IsDataInTxBuf
**功能:获知缓冲中是否有数据
**注意事项:当队列的头和尾不相等的时候,就代表了有数据在缓冲中
*************************************************/
extern bool CAN1IsDataInTxBuf( void );
extern bool CAN2IsDataInTxBuf( void );
/*************************************************
**函数名:CAN1GetTxBufLen
**功能:获取缓冲中有效数据的长度
**注意事项:获取的值必然为最小值,因为真实长度会不断变化.由于是32位的ARM系统,获取32位数据不存在临界问题,所以可以不考虑关中断
**			所谓有效,是指剩余的可用长度
**************************************************/
extern u32 CAN1GetTxBufLen(void);
extern u32 CAN2GetTxBufLen(void);
/*******************************************************
**函数名:CAN1BeginSend/StopSend
**功能:启动发送
**注意事项:这里使用空中断方式发送,只要发送寄存器为空,则会进入发送空中断,系统再在中断中进行发送
********************************************************/
extern void CAN1BeginSend(void);
extern void CAN2BeginSend(void);
extern void CAN1StopSend(void);
extern void CAN2StopSend(void);
extern CanTxMsg *CAN1GetTxBufDat( void );
extern CanTxMsg *CAN2GetTxBufDat( void );
extern void CAN_Configuration(void);
//发送一个字节的数据
extern void SendUCHARByCan(CAN_TypeDef *CANx, UCHAR8 dat);
//发送一个2字节的数据
extern void SendSINT32ByCan(CAN_TypeDef *CANx, s32 dat);
//发送一个2字节的数据
extern void SendUSHORT16ByCan(CAN_TypeDef *CANx, u16 dat);
#endif

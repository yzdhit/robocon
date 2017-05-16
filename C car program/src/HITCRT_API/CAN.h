#ifndef CAN_H
#define CAN_H
#include "stm32f4xx.h"
#define CAN1_RX_BUF_MAX 64/*������2����*/
#define CAN1_TX_BUF_MAX 64/*������2����*/
#define CAN1_RX_BUF_MARK	(CAN1_RX_BUF_MAX-1)
#define CAN1_TX_BUF_MARK	(CAN1_TX_BUF_MAX-1)

#define CAN2_RX_BUF_MAX 64/*������2����*/
#define CAN2_TX_BUF_MAX 64/*������2����*/
#define CAN2_RX_BUF_MARK	(CAN2_RX_BUF_MAX-1)
#define CAN2_TX_BUF_MARK	(CAN2_TX_BUF_MAX-1)

//extern CanTxMsg TxMessage;//Ҫ���͵���Ϣ��ȫ�ֱ���

/**************************************************************
** ������:CAN1PutDatatoRxBuf
** ����:�����ݷŽ�������,
** ע������:Ӧ���ڴ��ڽ����ж��е��ô˺���
***************************************************************/
extern void CAN1PutDatatoRxBuf(CanRxMsg *RxMessage);
extern void CAN2PutDatatoRxBuf(CanRxMsg *RxMessage);
/*************************************************
**������:CAN1IsDataInRxBuf
**����:��֪�������Ƿ�������
**ע������:�����е�ͷ��β����ȵ�ʱ��,�ʹ������������ڻ�����
*************************************************/
extern bool CAN1IsDataInRxBuf( void );
extern bool CAN2IsDataInRxBuf( void );
/*************************************************
**������:CAN1GetRxBufLen
**����:��ȡ��������Ч���ݵĳ���
**ע������:��ȡ��ֵ��ȻΪ��Сֵ,��Ϊ��ʵ���Ȼ᲻�ϱ仯.������32λ��ARMϵͳ,��ȡ32λ���ݲ������ٽ�����,���Կ��Բ����ǹ��ж�
**************************************************/
extern u32 CAN1GetRxBufLen(void);
extern u32 CAN2GetRxBufLen(void);
/**************************************************
**������:CAN1GetRxBufDat
**����:�Ӷ����л�ȡ����
**ע������:���ô˺���ǰ����ȷ��������������!!���������Ӳ�ȴ�
**************************************************/
extern CanRxMsg *CAN1GetRxBufDat( void );
extern CanRxMsg *CAN2GetRxBufDat( void );
//////////////////////////////////////////
//	���º���Ϊ�������
//////////////////////////////////////////
/**************************************************************
** ������:CAN1PutDatatoTxBuf
** ����:�����ݷŽ����Ͷ�����,
** ע������:�û���Ҫ�����ݷ���ʱ��ʹ��
***************************************************************/
extern bool CAN1PutDatatoTxBuf(CanTxMsg *TxMessage);
extern bool CAN2PutDatatoTxBuf(CanTxMsg *TxMessage);
/*************************************************
**������:CAN1IsDataInTxBuf
**����:��֪�������Ƿ�������
**ע������:�����е�ͷ��β����ȵ�ʱ��,�ʹ������������ڻ�����
*************************************************/
extern bool CAN1IsDataInTxBuf( void );
extern bool CAN2IsDataInTxBuf( void );
/*************************************************
**������:CAN1GetTxBufLen
**����:��ȡ��������Ч���ݵĳ���
**ע������:��ȡ��ֵ��ȻΪ��Сֵ,��Ϊ��ʵ���Ȼ᲻�ϱ仯.������32λ��ARMϵͳ,��ȡ32λ���ݲ������ٽ�����,���Կ��Բ����ǹ��ж�
**			��ν��Ч,��ָʣ��Ŀ��ó���
**************************************************/
extern u32 CAN1GetTxBufLen(void);
extern u32 CAN2GetTxBufLen(void);
/*******************************************************
**������:CAN1BeginSend/StopSend
**����:��������
**ע������:����ʹ�ÿ��жϷ�ʽ����,ֻҪ���ͼĴ���Ϊ��,�����뷢�Ϳ��ж�,ϵͳ�����ж��н��з���
********************************************************/
extern void CAN1BeginSend(void);
extern void CAN2BeginSend(void);
extern void CAN1StopSend(void);
extern void CAN2StopSend(void);
extern CanTxMsg *CAN1GetTxBufDat( void );
extern CanTxMsg *CAN2GetTxBufDat( void );
extern void CAN_Configuration(void);
//����һ���ֽڵ�����
extern void SendUCHARByCan(CAN_TypeDef *CANx, UCHAR8 dat);
//����һ��2�ֽڵ�����
extern void SendSINT32ByCan(CAN_TypeDef *CANx, s32 dat);
//����һ��2�ֽڵ�����
extern void SendUSHORT16ByCan(CAN_TypeDef *CANx, u16 dat);
#endif

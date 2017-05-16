#include "stm32f4xx.h"
#include "CAN.h"

static CanRxMsg CAN1_RX_Buf[CAN1_RX_BUF_MAX];		//����
static CanTxMsg CAN1_TX_Buf[CAN1_TX_BUF_MAX];		//����

static CanRxMsg CAN2_RX_Buf[CAN1_RX_BUF_MAX];		//����
static CanTxMsg CAN2_TX_Buf[CAN1_TX_BUF_MAX];		//����

volatile u32 CAN1_Rx_Head = 0;					//����ͷ-���յ�ʱ���ƶ�
volatile u32 CAN1_Rx_Tail = 0;					//����β-��ȡ��ʱ���ƶ�

volatile u32 CAN1_Tx_Head = 0;					//����ͷ-���յ�ʱ���ƶ�
volatile u32 CAN1_Tx_Tail = 0;					//����β-��ȡ��ʱ���ƶ�

volatile u32 CAN2_Rx_Head = 0;					//����ͷ-���յ�ʱ���ƶ�
volatile u32 CAN2_Rx_Tail = 0;					//����β-��ȡ��ʱ���ƶ�

volatile u32 CAN2_Tx_Head = 0;					//����ͷ-���յ�ʱ���ƶ�
volatile u32 CAN2_Tx_Tail = 0;					//����β-��ȡ��ʱ���ƶ�
//////////////////////////////////////////////////////
//	���º���Ϊ�������
/////////////////////////////////////////////////////
/**************************************************************
** ������:CAN1PutDatatoRxBuf
** ����:�����ݷŽ�������,
** ע������:Ӧ���ڴ��ڽ����ж��е��ô˺���
***************************************************************/
void CAN1PutDatatoRxBuf(CanRxMsg *pRxMessage)
{
	u32 tmphead;
	tmphead = ( CAN1_Rx_Head + 1 ) & CAN1_RX_BUF_MARK;//����ͷ�����ֵ�ж�,�������,����0
	CAN1_Rx_Head = tmphead; 	// ÿ��һ������,����ͷ����1 
	CAN1_RX_Buf[tmphead] = *pRxMessage; 				
}
void CAN2PutDatatoRxBuf(CanRxMsg *pRxMessage)
{
	u32 tmphead;
	tmphead = ( CAN2_Rx_Head + 1 ) & CAN2_RX_BUF_MARK;//����ͷ�����ֵ�ж�,�������,����0
	CAN2_Rx_Head = tmphead; 	// ÿ��һ������,����ͷ����1 
	CAN2_RX_Buf[tmphead] = *pRxMessage; 				
}
/*************************************************
**������:CAN1IsDataInRxBuf
**����:��֪�������Ƿ�������
**ע������:�����е�ͷ��β����ȵ�ʱ��,�ʹ������������ڻ�����
*************************************************/
bool CAN1IsDataInRxBuf( void )
{
	if( CAN1_Rx_Head == CAN1_Rx_Tail )
		return FALSE;
	else 
		return TRUE; 
}
bool CAN2IsDataInRxBuf( void )
{
	if( CAN2_Rx_Head == CAN2_Rx_Tail )
		return FALSE;
	else 
		return TRUE; 
}

/*************************************************
**������:CAN1GetRxBufLen
**����:��ȡ��������Ч���ݵĳ���
**ע������:��ȡ��ֵ��ȻΪ��Сֵ,��Ϊ��ʵ���Ȼ᲻�ϱ仯.������32λ��ARMϵͳ,��ȡ32λ���ݲ������ٽ�����,���Կ��Բ����ǹ��ж�
**************************************************/
u32 CAN1GetRxBufLen(void)
{
	//__disalbe_irq();
	if(CAN1_Rx_Head >= CAN1_Rx_Tail)
	{
		//__enable_irq();	
		return(CAN1_Rx_Head - CAN1_Rx_Tail);
	}
	else
	{
		//__enable_irq();	
		return(CAN1_RX_BUF_MAX + CAN1_Rx_Head - CAN1_Rx_Tail);
	}
}
u32 CAN2GetRxBufLen(void)
{
	//__disalbe_irq();
	if(CAN2_Rx_Head >= CAN2_Rx_Tail)
	{
		//__enable_irq();	
		return(CAN2_Rx_Head - CAN2_Rx_Tail);
	}
	else
	{
		//__enable_irq();	
		return(CAN2_RX_BUF_MAX + CAN2_Rx_Head - CAN2_Rx_Tail);
	}
}
/**************************************************
**������:CAN1GetRxBufDat
**����:�Ӷ����л�ȡ����
**ע������:���ô˺���ǰ����ȷ��������������!!���������Ӳ�ȴ�
**************************************************/
CanRxMsg *CAN1GetRxBufDat( void )
{
	u32 tmptail;
	while ( CAN1_Rx_Head == CAN1_Rx_Tail );//Ϊ��ֹ���ݻ��Ҷ�Ū�ϵ�Ӳ�ȴ�
	tmptail = ( CAN1_Rx_Tail + 1 ) & CAN1_RX_BUF_MARK;
	CAN1_Rx_Tail = tmptail;
	return &CAN1_RX_Buf[tmptail];
}
CanRxMsg *CAN2GetRxBufDat( void )
{
	u32 tmptail;
	while ( CAN2_Rx_Head == CAN2_Rx_Tail );//Ϊ��ֹ���ݻ��Ҷ�Ū�ϵ�Ӳ�ȴ�
	tmptail = ( CAN2_Rx_Tail + 1 ) & CAN2_RX_BUF_MARK;
	CAN2_Rx_Tail = tmptail;
	return &CAN2_RX_Buf[tmptail];
}



//////////////////////////////////////////
//	���º���Ϊ�������
//////////////////////////////////////////

/**************************************************************
** ������:CAN1PutDatatoTxBuf
** ����:�����ݷŽ����Ͷ�����,
** ע������:�û���Ҫ�����ݷ���ʱ��ʹ��
***************************************************************/
bool CAN1PutDatatoTxBuf(CanTxMsg *pTxMessage)
{
	u32 tmphead;
	tmphead = ( CAN1_Tx_Head + 1 ) & CAN1_TX_BUF_MARK;//����ĩ���ж�,����ĩ��,����0
	if(tmphead == CAN1_Tx_Tail)
		return FALSE;
	
	CAN1_Tx_Head = tmphead; 	// ÿ����,����ͷ����1 
	CAN1_TX_Buf[tmphead] = *pTxMessage; 	
	return TRUE;			
}
bool CAN2PutDatatoTxBuf(CanTxMsg *pTxMessage)
{
	u32 tmphead;
	tmphead = ( CAN2_Tx_Head + 1 ) & CAN2_TX_BUF_MARK;//����ĩ���ж�,����ĩ��,����0
	if(tmphead == CAN2_Tx_Tail)
		return FALSE;
	
	CAN2_Tx_Head = tmphead; 	// ÿ����,����ͷ����1 
	CAN2_TX_Buf[tmphead] = *pTxMessage; 	
	return TRUE;			
}
/*************************************************
**������:CAN1IsDataInTxBuf
**����:��֪�������Ƿ�������
**ע������:�����е�ͷ��β����ȵ�ʱ��,�ʹ������������ڻ�����
*************************************************/
bool CAN1IsDataInTxBuf( void )
{
	if( CAN1_Tx_Head == CAN1_Tx_Tail )
		return FALSE;
	else 
		return TRUE; 
}
bool CAN2IsDataInTxBuf( void )
{
	if( CAN2_Tx_Head == CAN2_Tx_Tail )
		return FALSE;
	else 
		return TRUE; 
}
/*************************************************
**������:CAN1GetTxBufLen
**����:��ȡ��������Ч���ݵĳ���
**ע������:��ȡ��ֵ��ȻΪ��Сֵ,��Ϊ��ʵ���Ȼ᲻�ϱ仯.������32λ��ARMϵͳ,��ȡ32λ���ݲ������ٽ�����,���Կ��Բ����ǹ��ж�
**			��ν��Ч,��ָʣ��Ŀ��ó���
**************************************************/
u32 CAN1GetTxBufLen(void)
{
	//__disalbe_irq();
	if(CAN1_Tx_Head >= CAN1_Tx_Tail)
	{
		//__enable_irq();	
		return(CAN1_TX_BUF_MAX - (CAN1_Tx_Head-CAN1_Tx_Tail));
	}
	else
	{
		//__enable_irq();	
		return(CAN1_Tx_Tail - CAN1_Tx_Head);
	}
}
u32 CAN2GetTxBufLen(void)
{
	//__disalbe_irq();
	if(CAN2_Tx_Head >= CAN2_Tx_Tail)
	{
		//__enable_irq();	
		return(CAN2_TX_BUF_MAX - (CAN2_Tx_Head-CAN2_Tx_Tail));
	}
	else
	{
		//__enable_irq();	
		return(CAN2_Tx_Tail - CAN2_Tx_Head);
	}
}
/**************************************************
**������:CAN1GetTxBufDat
**����:�Ӷ����л�ȡ����
**ע������:���ô˺���ǰ����ȷ��������������!!
**************************************************/
CanTxMsg *CAN1GetTxBufDat( void )
{
	u32 tmptail;
	while ( CAN1_Tx_Head == CAN1_Tx_Tail );//Ϊ��ֹ���ݻ��Ҷ�Ū�ϵ�Ӳ�ȴ�
	tmptail = ( CAN1_Tx_Tail + 1 ) & CAN1_TX_BUF_MARK;
	CAN1_Tx_Tail = tmptail;
	return &CAN1_TX_Buf[tmptail];
}
CanTxMsg *CAN2GetTxBufDat( void )
{
	u32 tmptail;
	while ( CAN2_Tx_Head == CAN2_Tx_Tail );//Ϊ��ֹ���ݻ��Ҷ�Ū�ϵ�Ӳ�ȴ�
	tmptail = ( CAN2_Tx_Tail + 1 ) & CAN2_TX_BUF_MARK;
	CAN2_Tx_Tail = tmptail;
	return &CAN2_TX_Buf[tmptail];
}
/*******************************************************
**������:CAN1BeginSend/StopSend
**����:��������
**ע������:����ʹ�ÿ��жϷ�ʽ����,ֻҪ���ͼĴ���Ϊ��,�����뷢�Ϳ��ж�,ϵͳ�����ж��н��з���
********************************************************/
void CAN1BeginSend(void)
{
//	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);//����������ж�
	CAN1->IER |= CAN_IT_TME;
}
void CAN2BeginSend(void)
{
//	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);//����������ж�
	CAN2->IER |= CAN_IT_TME;
}
void CAN1StopSend(void)
{
	CAN1->IER &= ~CAN_IT_TME;
}
void CAN2StopSend(void)
{
	CAN2->IER &= ~CAN_IT_TME;
}
void CAN_Configuration(void)
{
//#if 0
	/* Private variables ---------------------------------------------------------*/
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	CanTxMsg TxMessage;
  
	/* CAN GPIOs configuration **************************************************/
	/* Connect CAN pins to AF9 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1); 

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2); 
  
	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* CAN configuration ********************************************************/  
	/* Enable CAN clock */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);
	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_DeInit(CAN2);


	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = ENABLE;//�¼�����
	CAN_InitStructure.CAN_ABOM = ENABLE;//�Զ����߹���
	CAN_InitStructure.CAN_AWUM = ENABLE;//�Զ�����
	CAN_InitStructure.CAN_NART = DISABLE;//�����ش�
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    
	/* CAN Baudrate = 1 MBps (CAN clocked at 30 MHz) */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
	CAN_InitStructure.CAN_Prescaler = 21;//��ƵΪ500k
	CAN_Init(CAN1, &CAN_InitStructure);
	CAN_Init(CAN2, &CAN_InitStructure);
	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_FilterInitStructure.CAN_FilterNumber = 14;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/* Transmit Structure preparation */
	TxMessage.StdId = 0x00;
	TxMessage.ExtId = 0x00;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 1;
	CAN_Transmit(CAN1,&TxMessage);
	CAN_Transmit(CAN2,&TxMessage);
  
	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);//�����ж�	
	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);//����������ж�
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);//�����ж�		
	CAN_ITConfig(CAN2,CAN_IT_TME, ENABLE);//����������ж�
//#endif
}
//����һ���ֽڵ�����
void SendUCHARByCan(CAN_TypeDef *CANx, UCHAR8 dat)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x11;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=1;
	TxMessage.Data[0] = dat;
	
	CAN_Transmit(CANx,&TxMessage);
}
//����һ��2�ֽڵ�����
void SendSINT32ByCan(CAN_TypeDef *CANx, s32 dat)
{
	CanTxMsg TxMessage;
	s32 *p;
	TxMessage.StdId = 0x11;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=4;
	p = (s32*)TxMessage.Data;
	*p = dat;
	
	CAN_Transmit(CANx,&TxMessage);
}
//����һ��2�ֽڵ�����
void SendUSHORT16ByCan(CAN_TypeDef *CANx, u16 dat)
{
	CanTxMsg TxMessage;
	u16 *p;
	TxMessage.StdId = 0x11;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=2;
	p = (u16*)TxMessage.Data;
	*p = dat;
	
	CAN_Transmit(CANx,&TxMessage);
}

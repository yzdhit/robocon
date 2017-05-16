#include "stm32f4xx.h"
#include "CAN.h"

static CanRxMsg CAN1_RX_Buf[CAN1_RX_BUF_MAX];		//队列
static CanTxMsg CAN1_TX_Buf[CAN1_TX_BUF_MAX];		//队列

static CanRxMsg CAN2_RX_Buf[CAN1_RX_BUF_MAX];		//队列
static CanTxMsg CAN2_TX_Buf[CAN1_TX_BUF_MAX];		//队列

volatile u32 CAN1_Rx_Head = 0;					//队列头-接收的时候移动
volatile u32 CAN1_Rx_Tail = 0;					//队列尾-读取的时候移动

volatile u32 CAN1_Tx_Head = 0;					//队列头-接收的时候移动
volatile u32 CAN1_Tx_Tail = 0;					//队列尾-读取的时候移动

volatile u32 CAN2_Rx_Head = 0;					//队列头-接收的时候移动
volatile u32 CAN2_Rx_Tail = 0;					//队列尾-读取的时候移动

volatile u32 CAN2_Tx_Head = 0;					//队列头-接收的时候移动
volatile u32 CAN2_Tx_Tail = 0;					//队列尾-读取的时候移动
//////////////////////////////////////////////////////
//	以下函数为接收相关
/////////////////////////////////////////////////////
/**************************************************************
** 函数名:CAN1PutDatatoRxBuf
** 功能:把数据放进队列中,
** 注意事项:应该在串口接收中断中调用此函数
***************************************************************/
void CAN1PutDatatoRxBuf(CanRxMsg *pRxMessage)
{
	u32 tmphead;
	tmphead = ( CAN1_Rx_Head + 1 ) & CAN1_RX_BUF_MARK;//队列头的最大值判断,到达最大,则变回0
	CAN1_Rx_Head = tmphead; 	// 每收一次数据,队列头增加1 
	CAN1_RX_Buf[tmphead] = *pRxMessage; 				
}
void CAN2PutDatatoRxBuf(CanRxMsg *pRxMessage)
{
	u32 tmphead;
	tmphead = ( CAN2_Rx_Head + 1 ) & CAN2_RX_BUF_MARK;//队列头的最大值判断,到达最大,则变回0
	CAN2_Rx_Head = tmphead; 	// 每收一次数据,队列头增加1 
	CAN2_RX_Buf[tmphead] = *pRxMessage; 				
}
/*************************************************
**函数名:CAN1IsDataInRxBuf
**功能:获知缓冲中是否有数据
**注意事项:当队列的头和尾不相等的时候,就代表了有数据在缓冲中
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
**函数名:CAN1GetRxBufLen
**功能:获取缓冲中有效数据的长度
**注意事项:获取的值必然为最小值,因为真实长度会不断变化.由于是32位的ARM系统,获取32位数据不存在临界问题,所以可以不考虑关中断
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
**函数名:CAN1GetRxBufDat
**功能:从队列中获取数据
**注意事项:调用此函数前请先确保队列中有数据!!否则会陷入硬等待
**************************************************/
CanRxMsg *CAN1GetRxBufDat( void )
{
	u32 tmptail;
	while ( CAN1_Rx_Head == CAN1_Rx_Tail );//为防止数据混乱而弄上的硬等待
	tmptail = ( CAN1_Rx_Tail + 1 ) & CAN1_RX_BUF_MARK;
	CAN1_Rx_Tail = tmptail;
	return &CAN1_RX_Buf[tmptail];
}
CanRxMsg *CAN2GetRxBufDat( void )
{
	u32 tmptail;
	while ( CAN2_Rx_Head == CAN2_Rx_Tail );//为防止数据混乱而弄上的硬等待
	tmptail = ( CAN2_Rx_Tail + 1 ) & CAN2_RX_BUF_MARK;
	CAN2_Rx_Tail = tmptail;
	return &CAN2_RX_Buf[tmptail];
}



//////////////////////////////////////////
//	以下函数为发送相关
//////////////////////////////////////////

/**************************************************************
** 函数名:CAN1PutDatatoTxBuf
** 功能:把数据放进发送队列中,
** 注意事项:用户需要有数据发的时候使用
***************************************************************/
bool CAN1PutDatatoTxBuf(CanTxMsg *pTxMessage)
{
	u32 tmphead;
	tmphead = ( CAN1_Tx_Head + 1 ) & CAN1_TX_BUF_MARK;//队列末端判断,到达末端,则变回0
	if(tmphead == CAN1_Tx_Tail)
		return FALSE;
	
	CAN1_Tx_Head = tmphead; 	// 每入列,队列头增加1 
	CAN1_TX_Buf[tmphead] = *pTxMessage; 	
	return TRUE;			
}
bool CAN2PutDatatoTxBuf(CanTxMsg *pTxMessage)
{
	u32 tmphead;
	tmphead = ( CAN2_Tx_Head + 1 ) & CAN2_TX_BUF_MARK;//队列末端判断,到达末端,则变回0
	if(tmphead == CAN2_Tx_Tail)
		return FALSE;
	
	CAN2_Tx_Head = tmphead; 	// 每入列,队列头增加1 
	CAN2_TX_Buf[tmphead] = *pTxMessage; 	
	return TRUE;			
}
/*************************************************
**函数名:CAN1IsDataInTxBuf
**功能:获知缓冲中是否有数据
**注意事项:当队列的头和尾不相等的时候,就代表了有数据在缓冲中
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
**函数名:CAN1GetTxBufLen
**功能:获取缓冲中有效数据的长度
**注意事项:获取的值必然为最小值,因为真实长度会不断变化.由于是32位的ARM系统,获取32位数据不存在临界问题,所以可以不考虑关中断
**			所谓有效,是指剩余的可用长度
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
**函数名:CAN1GetTxBufDat
**功能:从队列中获取数据
**注意事项:调用此函数前请先确保队列中有数据!!
**************************************************/
CanTxMsg *CAN1GetTxBufDat( void )
{
	u32 tmptail;
	while ( CAN1_Tx_Head == CAN1_Tx_Tail );//为防止数据混乱而弄上的硬等待
	tmptail = ( CAN1_Tx_Tail + 1 ) & CAN1_TX_BUF_MARK;
	CAN1_Tx_Tail = tmptail;
	return &CAN1_TX_Buf[tmptail];
}
CanTxMsg *CAN2GetTxBufDat( void )
{
	u32 tmptail;
	while ( CAN2_Tx_Head == CAN2_Tx_Tail );//为防止数据混乱而弄上的硬等待
	tmptail = ( CAN2_Tx_Tail + 1 ) & CAN2_TX_BUF_MARK;
	CAN2_Tx_Tail = tmptail;
	return &CAN2_TX_Buf[tmptail];
}
/*******************************************************
**函数名:CAN1BeginSend/StopSend
**功能:启动发送
**注意事项:这里使用空中断方式发送,只要发送寄存器为空,则会进入发送空中断,系统再在中断中进行发送
********************************************************/
void CAN1BeginSend(void)
{
//	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);//发送邮箱空中断
	CAN1->IER |= CAN_IT_TME;
}
void CAN2BeginSend(void)
{
//	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);//发送邮箱空中断
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
	CAN_InitStructure.CAN_TTCM = ENABLE;//事件触发
	CAN_InitStructure.CAN_ABOM = ENABLE;//自动离线管理
	CAN_InitStructure.CAN_AWUM = ENABLE;//自动唤醒
	CAN_InitStructure.CAN_NART = DISABLE;//错误重传
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    
	/* CAN Baudrate = 1 MBps (CAN clocked at 30 MHz) */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
	CAN_InitStructure.CAN_Prescaler = 21;//分频为500k
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
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);//接收中断	
	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);//发送邮箱空中断
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);//接收中断		
	CAN_ITConfig(CAN2,CAN_IT_TME, ENABLE);//发送邮箱空中断
//#endif
}
//发送一个字节的数据
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
//发送一个2字节的数据
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
//发送一个2字节的数据
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

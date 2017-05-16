/**
  ******************************************************************************
  * @file    ADC3_DMA/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "ucos_ii.h"
#include "rcc.h"
#include "TIM.h"
#include "CAN.h"
#include "UART.h"
#include "IrComm.h"
#include "PrivateFunction.h"
#include "RobotCtrl.h"


extern UCHAR8 g_ucWatchComTemp;
//#include "PC_Comm_Structs.h"

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup ADC_ADC3_DMA
  * @{
  */  

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
//  static int i = 0;
//  i++;
// return;

  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
/*
void PendSV_Handler(void)
{
}*/

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	
	CPU_SR         cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();
	if(Timer1)
		Timer1--;
	if(Timer2)
		Timer2--;
    OSTimeTick();                                /* Call uC/OS-II's OSTimeTick()                       */
//	TimingDelay_Decrement();
    OSIntExit();  
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
  /**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */

uint16_t capture = 0;
void TIM3_IRQHandler(void)
{
	CPU_SR         cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();
#if 0
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

//		GPIOD->ODR ^= GPIO_Pin_12;
		capture = TIM_GetCapture1(TIM3);
		TIM_SetCompare1(TIM3, capture + CCR1_Val);
	}
	else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

//		GPIOD->ODR ^= GPIO_Pin_13;
		capture = TIM_GetCapture2(TIM3);
		TIM_SetCompare2(TIM3, capture + CCR2_Val);
	}
	else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

//		GPIOD->ODR ^= GPIO_Pin_14;
		capture = TIM_GetCapture3(TIM3);
		TIM_SetCompare3(TIM3, capture + CCR3_Val);
	}
	else
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);

//		GPIOD->ODR ^= GPIO_Pin_15;
		capture = TIM_GetCapture4(TIM3);
		TIM_SetCompare4(TIM3, capture + CCR4_Val);
	}
	#endif
	OSIntExit(); 
}
//#ifdef USE_CAN1
/**
  * @brief  This function handles CAN1 RX0 request.
  * @param  None
  * @retval None
  */
 
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage; 
	CPU_SR         cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();
	
	
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	CAN1PutDatatoRxBuf(&RxMessage);

	OSIntExit();
}

void CAN1_TX_IRQHandler(void)
{
	CPU_SR         cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();
	
	if(CAN1IsDataInTxBuf() == TRUE)
	{
		CAN_Transmit(CAN1, CAN1GetTxBufDat());
	}
	else
	{
		CAN1StopSend();
	}
	OSIntExit();
}
/**
  * @brief  This function handles CAN2 RX0 request.
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage; 
	CPU_SR         cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();
	
	CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
	CAN2PutDatatoRxBuf(&RxMessage);

	OSIntExit();
}
void CAN2_TX_IRQHandler(void)
{
	CPU_SR         cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();
	
	if(CAN2IsDataInTxBuf() == TRUE)
	{
		CAN_Transmit(CAN2, CAN2GetTxBufDat());
	}
	else
	{
		CAN2StopSend();
	}

	OSIntExit();
}
void USART1_IRQHandler(void)
{
	CPU_SR         cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();

   
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
	/* Read one byte from the receive data register */
	   #if EN_UART1_PC_ARM
 	   //在此使用兰博士的接受中断，无需修改此项的内容
		  CommuInterrupt_Irq();
	         g_uiVisionUpdateTime = GetCurTime();
	   #else 
	   //如果不使用兰博士的接受中断，则在此添加自己的接受中断的代码
	   
	   #endif
	}

	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{   
		/* Write one byte to the transmit data register */
		//USART_SendData(USART1,'a');
	}

	OSIntExit();
}
void USART2_IRQHandler(void)
{
//	char scTmp; 
	CPU_SR         cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
    OSIntNesting++;
    OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{

	/* Read one byte from the receive data register */
//		scTmp = USART_ReceiveData(USART2);
//		USART_SendData(USART2, USART_ReceiveData(USART2));
		g_ucWatchComTemp=USART_ReceiveData(USART2);
		
              IrComm1PutDataToRxBuf(g_ucWatchComTemp);
            
	}

	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
	{   
		/* Write one byte to the transmit data register */
//		USART_SendData(USART2,'a');
	}

	OSIntExit();
}

void USART6_IRQHandler(void)
{
//	char scTmp; 
	CPU_SR         cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
       OSIntNesting++;
       OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{

	/* Read one byte from the receive data register */
//		scTmp = USART_ReceiveData(USART2);
//		USART_SendData(USART2, USART_ReceiveData(USART2));
              RcvMEMSGryo(USART_ReceiveData(USART6),&g_ssMemsGryo);
      

             
            
	}

	if(USART_GetITStatus(USART6, USART_IT_TXE) != RESET)
	{   
		/* Write one byte to the transmit data register */
//		USART_SendData(USART2,'a');
	}

	OSIntExit();
}

/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

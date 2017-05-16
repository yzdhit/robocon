#include "hal.h"
#include "Select.h"

/*******************************
**������:ChipHalInit()
**����:Ƭ��Ӳ����ʼ��
*******************************/
void  ChipHalInit(void)
{
	//��ʼ��ʱ��Դ
	RCC_Configuration();
	NVIC_Configuration();
	ARM_LED_Configuration();
	//��ʼ��GPIO
	GPIO_Configuration();
	TIM_Configuration();
	FSMC_Configuration();
	CAN_Configuration();
	#if EN_UART1
	UART1_Configuration();
	#endif
	#if EN_UART2
	UART2_Configuration();
	#endif
	#if EN_UART3
	#endif
	#if EN_UART4
	#endif
	#if EN_UART5
	#endif
	#if EN_UART6
	UART6_Configuration();
	#endif
	

}


/*********************************
**������:ChipOutHalInit()
**����:Ƭ��Ӳ����ʼ��
*********************************/
void  ChipOutHalInit(void)
{
	//��ʼ��SD
//	TurnToSD();
//	TestSD();
}

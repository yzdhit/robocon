#include "hal.h"
#include "Select.h"

/*******************************
**函数名:ChipHalInit()
**功能:片内硬件初始化
*******************************/
void  ChipHalInit(void)
{
	//初始化时钟源
	RCC_Configuration();
	NVIC_Configuration();
	ARM_LED_Configuration();
	//初始化GPIO
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
**函数名:ChipOutHalInit()
**功能:片外硬件初始化
*********************************/
void  ChipOutHalInit(void)
{
	//初始化SD
//	TurnToSD();
//	TestSD();
}

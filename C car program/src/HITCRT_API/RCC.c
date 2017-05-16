#include "stm32f4xx.h"


void RCC_Configuration(void)
{
	//SYSTICK分频--1ms的系统时钟中断
	if (SysTick_Config(SystemCoreClock / 1000))
	{ 
    /* Capture error */ 
		while (1);
	}
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD |RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF |   RCC_AHB1Periph_GPIOG, ENABLE);

	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
		
}
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
static __IO u32 TimingDelay; 
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}
/********************************************
**函数名:SysTickDelay
**功能:使用系统时钟的硬延迟
**注意事项:一般地,不要在中断中调用本函数,否则会存在重入问题.另外如果屏蔽了全局中断,则不要使用此函数
********************************************/
volatile u16 Timer1,Timer2;
void SysTickDelay(u16 dly_ms)
{
	Timer1=dly_ms;
	while(Timer1);
}


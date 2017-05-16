#include "stm32f4xx.h"


void RCC_Configuration(void)
{
	//SYSTICK��Ƶ--1ms��ϵͳʱ���ж�
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
**������:SysTickDelay
**����:ʹ��ϵͳʱ�ӵ�Ӳ�ӳ�
**ע������:һ���,��Ҫ���ж��е��ñ�����,����������������.�������������ȫ���ж�,��Ҫʹ�ô˺���
********************************************/
volatile u16 Timer1,Timer2;
void SysTickDelay(u16 dly_ms)
{
	Timer1=dly_ms;
	while(Timer1);
}


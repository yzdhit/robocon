#ifndef RCC_H
#define RCC_H
#include "stm32f4xx.h"
extern volatile u16 Timer1,Timer2;
extern void SysTickDelay(u16 dly_ms);
extern void RCC_Configuration(void);
extern void Delay(u32 nTime);
extern void TimingDelay_Decrement(void);

#endif

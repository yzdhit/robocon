#ifndef HAL_H
#define HAL_H
#include "GPIO.h"
#include "RCC.h"
#include "FSMC.h"
#include "TIM.h"
#include "UART.h"
#include "CAN.h"
#include "NVIC.h"
#include "SD.h"

extern void  ChipHalInit(void);
extern void  ChipOutHalInit(void);

#endif


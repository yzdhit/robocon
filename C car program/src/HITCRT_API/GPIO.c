#include "stm32f4xx.h"

void ARM_LED_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//ARM_LED配置
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF,&GPIO_InitStructure);	
}
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
/*	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);*/

//	GPIOD->MODER |= (u32) GPIO_Mode_OUT << (12 * 2) | GPIO_Mode_OUT << (13 * 2) | GPIO_Mode_OUT << (14 * 2) | GPIO_Mode_OUT << (15 * 2);//输出
///	GPIOx->OTYPER &= (u16);//默认推挽输出,不需要设置
//	GPIOD->OTYPER &= ~(GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15);//Open Drain模式
//	GPIOD->PUPDR &= ;//No pull up，pull down
//	GPIOD->OSPEEDR = (u32) GPIO_Speed_100MHz << (12 * 2) | GPIO_Speed_100MHz << (13 * 2) | GPIO_Speed_100MHz << (14 * 2) | GPIO_Speed_100MHz << (15 * 2);

/*	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);*/


	
/* Configure Motor_Control_1 in output pushpull mode */
/*	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_7;
	GPIO_Init(GPIOG, &GPIO_InitStructure);*/


//Motor_Control_1配置
	GPIOA->MODER |= (u32) GPIO_Mode_OUT << (8 * 2);//输出
	GPIOA->OTYPER &= ~GPIO_Pin_8;;//默认推挽输出,不需要设置
	GPIOA->PUPDR &= ~(0x11 << (8 * 2));//默认，No pull up，pull down
	GPIOA->OSPEEDR = (u32) GPIO_Speed_100MHz << (8 * 2);

	GPIOB->MODER |= (u32) GPIO_Mode_OUT << (10 * 2) | GPIO_Mode_OUT << (11 * 2) | GPIO_Mode_OUT << (12 * 2) | GPIO_Mode_OUT << (13 * 2) | GPIO_Mode_OUT << (14 * 2) | GPIO_Mode_OUT << (15 * 2);//输出
	GPIOB->OTYPER &= ~(GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);//默认推挽输出,不需要设置
//	GPIOB->PUPDR &= ;//默认，No pull up，pull down
	GPIOB->OSPEEDR = (u32) GPIO_Speed_100MHz << (10 * 2) | GPIO_Speed_100MHz << (11 * 2) | GPIO_Speed_100MHz << (12 * 2) | GPIO_Speed_100MHz << (13 * 2) | GPIO_Speed_100MHz << (14 * 2) | GPIO_Speed_100MHz << (15 * 2);//输出

	GPIOC->MODER |= (u32) GPIO_Mode_OUT << (8 * 2) | GPIO_Mode_OUT << (9 * 2);//输出
	GPIOC->OTYPER &= ~(GPIO_Pin_8 | GPIO_Pin_9);//默认推挽输出,不需要设置
//	GPIOC->PUPDR &= ;//默认，No pull up，pull down
	GPIOC->OSPEEDR = (u32) GPIO_Speed_100MHz << (8 * 2) | GPIO_Speed_100MHz << (9 * 2);//输出
	
	GPIOF->MODER |= (u32) GPIO_Mode_OUT << (14 * 2);//输出
	GPIOF->OTYPER &= ~GPIO_Pin_14;//默认推挽输出,不需要设置
//	GPIOF->PUPDR &= ;//默认，No pull up，pull down
	GPIOF->OSPEEDR = (u32) GPIO_Speed_100MHz << (14 * 2);

	GPIOG->MODER |= (u32) GPIO_Mode_OUT << (7 * 2) | GPIO_Mode_OUT << (8 * 2);//输出
	GPIOG->OTYPER &= ~(GPIO_Pin_7 | GPIO_Pin_8);//Open Drain模式
//	GPIOG->PUPDR &= ;//默认，No pull up，pull down
	GPIOG->OSPEEDR = (u32) GPIO_Speed_100MHz << (7 * 2) | GPIO_Speed_100MHz << (8 * 2);//输出



	//Motor_Control_2配置
	GPIOA->MODER |= (u32) GPIO_Mode_OUT << (11 * 2) | GPIO_Mode_OUT << (12 * 2);//输出
	GPIOA->OTYPER &= ~(GPIO_Pin_11 | GPIO_Pin_12);;//默认推挽输出,不需要设置
	GPIOA->PUPDR &= ~(0x11 << (11 * 2) | 0x11 << (12 * 2));//默认，No pull up，pull down
	GPIOA->OSPEEDR = (u32) GPIO_Speed_100MHz << (11 * 2) | GPIO_Speed_100MHz << (12 * 2);

	GPIOC->MODER |= (u32) GPIO_Mode_OUT << (10 * 2) | GPIO_Mode_OUT << (11 * 2) | GPIO_Mode_OUT << (12 * 2);//输出
	GPIOC->OTYPER &= ~(GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);//默认推挽输出,不需要设置
//	GPIOC->PUPDR &= ;//默认，No pull up，pull down
	GPIOC->OSPEEDR = (u32) GPIO_Speed_100MHz << (10 * 2) | GPIO_Speed_100MHz << (11 * 2) | GPIO_Speed_100MHz << (12 * 2);//输出

	GPIOD->MODER |= (u32) GPIO_Mode_OUT << (2 * 2);//输出
	GPIOD->OTYPER &= ~GPIO_Pin_2;//默认推挽输出,不需要设置
//	GPIOD->PUPDR &= ;//默认，No pull up，pull down
	GPIOD->OSPEEDR = (u32) GPIO_Speed_100MHz << (2 * 2);//输出
	


	GPIOG->MODER |= (u32) GPIO_Mode_OUT << (9 * 2) | GPIO_Mode_OUT << (10 * 2) | GPIO_Mode_OUT << (11 * 2) | GPIO_Mode_OUT << (12 * 2) | GPIO_Mode_OUT << (13 * 2) | GPIO_Mode_OUT << (14 * 2);//输出
	GPIOG->OTYPER &= ~(GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14);//Open Drain模式
//	GPIOG->PUPDR &= ;//默认，No pull up，pull down
	GPIOG->OSPEEDR = (u32) GPIO_Speed_100MHz << (9 * 2) | GPIO_Speed_100MHz << (10 * 2) | GPIO_Speed_100MHz << (11 * 2) | GPIO_Speed_100MHz << (12 * 2) | GPIO_Speed_100MHz << (13 * 2) | GPIO_Speed_100MHz << (14 * 2);//输出

}



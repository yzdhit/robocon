#include "stm32f4xx.h"

void ARM_LED_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//ARM_LED����
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

//	GPIOD->MODER |= (u32) GPIO_Mode_OUT << (12 * 2) | GPIO_Mode_OUT << (13 * 2) | GPIO_Mode_OUT << (14 * 2) | GPIO_Mode_OUT << (15 * 2);//���
///	GPIOx->OTYPER &= (u16);//Ĭ���������,����Ҫ����
//	GPIOD->OTYPER &= ~(GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15);//Open Drainģʽ
//	GPIOD->PUPDR &= ;//No pull up��pull down
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


//Motor_Control_1����
	GPIOA->MODER |= (u32) GPIO_Mode_OUT << (8 * 2);//���
	GPIOA->OTYPER &= ~GPIO_Pin_8;;//Ĭ���������,����Ҫ����
	GPIOA->PUPDR &= ~(0x11 << (8 * 2));//Ĭ�ϣ�No pull up��pull down
	GPIOA->OSPEEDR = (u32) GPIO_Speed_100MHz << (8 * 2);

	GPIOB->MODER |= (u32) GPIO_Mode_OUT << (10 * 2) | GPIO_Mode_OUT << (11 * 2) | GPIO_Mode_OUT << (12 * 2) | GPIO_Mode_OUT << (13 * 2) | GPIO_Mode_OUT << (14 * 2) | GPIO_Mode_OUT << (15 * 2);//���
	GPIOB->OTYPER &= ~(GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);//Ĭ���������,����Ҫ����
//	GPIOB->PUPDR &= ;//Ĭ�ϣ�No pull up��pull down
	GPIOB->OSPEEDR = (u32) GPIO_Speed_100MHz << (10 * 2) | GPIO_Speed_100MHz << (11 * 2) | GPIO_Speed_100MHz << (12 * 2) | GPIO_Speed_100MHz << (13 * 2) | GPIO_Speed_100MHz << (14 * 2) | GPIO_Speed_100MHz << (15 * 2);//���

	GPIOC->MODER |= (u32) GPIO_Mode_OUT << (8 * 2) | GPIO_Mode_OUT << (9 * 2);//���
	GPIOC->OTYPER &= ~(GPIO_Pin_8 | GPIO_Pin_9);//Ĭ���������,����Ҫ����
//	GPIOC->PUPDR &= ;//Ĭ�ϣ�No pull up��pull down
	GPIOC->OSPEEDR = (u32) GPIO_Speed_100MHz << (8 * 2) | GPIO_Speed_100MHz << (9 * 2);//���
	
	GPIOF->MODER |= (u32) GPIO_Mode_OUT << (14 * 2);//���
	GPIOF->OTYPER &= ~GPIO_Pin_14;//Ĭ���������,����Ҫ����
//	GPIOF->PUPDR &= ;//Ĭ�ϣ�No pull up��pull down
	GPIOF->OSPEEDR = (u32) GPIO_Speed_100MHz << (14 * 2);

	GPIOG->MODER |= (u32) GPIO_Mode_OUT << (7 * 2) | GPIO_Mode_OUT << (8 * 2);//���
	GPIOG->OTYPER &= ~(GPIO_Pin_7 | GPIO_Pin_8);//Open Drainģʽ
//	GPIOG->PUPDR &= ;//Ĭ�ϣ�No pull up��pull down
	GPIOG->OSPEEDR = (u32) GPIO_Speed_100MHz << (7 * 2) | GPIO_Speed_100MHz << (8 * 2);//���



	//Motor_Control_2����
	GPIOA->MODER |= (u32) GPIO_Mode_OUT << (11 * 2) | GPIO_Mode_OUT << (12 * 2);//���
	GPIOA->OTYPER &= ~(GPIO_Pin_11 | GPIO_Pin_12);;//Ĭ���������,����Ҫ����
	GPIOA->PUPDR &= ~(0x11 << (11 * 2) | 0x11 << (12 * 2));//Ĭ�ϣ�No pull up��pull down
	GPIOA->OSPEEDR = (u32) GPIO_Speed_100MHz << (11 * 2) | GPIO_Speed_100MHz << (12 * 2);

	GPIOC->MODER |= (u32) GPIO_Mode_OUT << (10 * 2) | GPIO_Mode_OUT << (11 * 2) | GPIO_Mode_OUT << (12 * 2);//���
	GPIOC->OTYPER &= ~(GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);//Ĭ���������,����Ҫ����
//	GPIOC->PUPDR &= ;//Ĭ�ϣ�No pull up��pull down
	GPIOC->OSPEEDR = (u32) GPIO_Speed_100MHz << (10 * 2) | GPIO_Speed_100MHz << (11 * 2) | GPIO_Speed_100MHz << (12 * 2);//���

	GPIOD->MODER |= (u32) GPIO_Mode_OUT << (2 * 2);//���
	GPIOD->OTYPER &= ~GPIO_Pin_2;//Ĭ���������,����Ҫ����
//	GPIOD->PUPDR &= ;//Ĭ�ϣ�No pull up��pull down
	GPIOD->OSPEEDR = (u32) GPIO_Speed_100MHz << (2 * 2);//���
	


	GPIOG->MODER |= (u32) GPIO_Mode_OUT << (9 * 2) | GPIO_Mode_OUT << (10 * 2) | GPIO_Mode_OUT << (11 * 2) | GPIO_Mode_OUT << (12 * 2) | GPIO_Mode_OUT << (13 * 2) | GPIO_Mode_OUT << (14 * 2);//���
	GPIOG->OTYPER &= ~(GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14);//Open Drainģʽ
//	GPIOG->PUPDR &= ;//Ĭ�ϣ�No pull up��pull down
	GPIOG->OSPEEDR = (u32) GPIO_Speed_100MHz << (9 * 2) | GPIO_Speed_100MHz << (10 * 2) | GPIO_Speed_100MHz << (11 * 2) | GPIO_Speed_100MHz << (12 * 2) | GPIO_Speed_100MHz << (13 * 2) | GPIO_Speed_100MHz << (14 * 2);//���

}



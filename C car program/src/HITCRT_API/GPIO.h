#ifndef GPIO_H
#define GPIO_H
//ARM_LEDÉÁË¸
#define ARM_LED_FLASH() GPIOF->ODR ^= GPIO_Pin_7
extern void GPIO_Configuration(void);
extern void ARM_LED_Configuration(void);
#endif

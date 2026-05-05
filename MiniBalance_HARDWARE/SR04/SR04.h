#ifndef __SR04_H
#define __SR04_H

#include "AllHeader.h"

#define  HCSR04_PORT_Trig                   GPIOA
#define  HCSR04_PORT_Echo                   GPIOA
#define  HCSR04_CLK_Trig                    RCC_APB2Periph_GPIOA
#define  HCSR04_CLK_Echo                    RCC_APB2Periph_GPIOA
#define  HCSR04_TRIG                        GPIO_Pin_0 
#define  HCSR04_ECHO                        GPIO_Pin_6
// 增加第二路超声波的宏定义
#define HCSR04_PORT_Trig2                   GPIOA
#define HCSR04_PORT_Echo2                   GPIOA
#define HCSR04_CLK_Trig2                    RCC_APB2Periph_GPIOA
#define HCSR04_CLK_Echo2                    RCC_APB2Periph_GPIOA
#define HCSR04_TRIG2                        GPIO_Pin_1 
#define HCSR04_ECHO2                        GPIO_Pin_7

void turnleft_90(void);
void turnright_90(void );
void turn_onecircle(void);
void HC_SR04_init(void);
void HC_SR04_start(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
u32 GetEchoTimer(void);
void TIM3_IRQHandler(void);
float HCSR04GetLength(GPIO_TypeDef* Echo_PORT, uint16_t Echo_PIN);
void avoid_obstacle(float front_dist, float left_dist);


#endif

//Header for n64.c

#ifndef INC_N64_H_
#define INC_N64_H_

#include <stdlib.h>
#include "stm32l4xx_hal.h"

#define GPIOC_ADDR 0x48000800
#define IDR_OFFSET 0x10
#define ODR_OFFSET 0x14
//#define TIMER TIM4 never used!
void delay_us (uint16_t us);
void writeOne();
void writeZero();
uint32_t pollRead();

void N64_init(TIM_HandleTypeDef* countTimer);
#endif

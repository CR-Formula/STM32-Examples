/*
 * This is a header file for ADC read using DMA
 */

#include "stm32h743xx.h"

// https://controllerstech.com/dma-with-adc-using-registers-in-stm32/

static inline void ADC_init() {
	RCC->APB2ENR |= (1<<8);
	RCC->AHB1ENR |= (1<<0);


}

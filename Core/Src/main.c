#include "stm32f407xx.h"
#include "cmsis_os2.h"

#include <stdio.h>

#include "sysclk.h"
#include "uart.h"
#include "timer.h"
#include "gpio.h"
#include "adc.h"

#define HSE_VALUE 8000000U

uint16_t adc_value[3];
volatile uint16_t adc_buffer[16];

void SysClock_Config();

/**
 * @brief Initialize the LED Pins
 * 
 * @note LEDs on the STM32F4-Disco board
 */
void LED_Init() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // Enable GPIO D Clock

  // Set LED Pins to Output mode
  // Pins reset to push pull mode
  GPIOD->MODER &= ~GPIO_MODER_MODE12 & ~GPIO_MODER_MODE13 
                & ~GPIO_MODER_MODE14 & ~GPIO_MODER_MODE15; // Clear LED Pins
  GPIOD->MODER |= (0b01 << GPIO_MODER_MODE12_Pos) | (0b01 << GPIO_MODER_MODE13_Pos) 
                | (0b01 << GPIO_MODER_MODE14_Pos) | (0b01 << GPIO_MODER_MODE15_Pos); // Set LED Pins to output
}

/**
 * @brief Bad Delay Function
 * 
 * @note DO NOT USE in production code
 * @note Only for testing
 */
void Delay_Temp() {
  // When Using OSDelay();
  // seconds = (SysTick value) / (clock frequency)
  for (int i = 0; i < 10000000; i++) {
    __NOP();
  }
}

/**
 * @brief Handle Timer 2 Interrupt
 * 
 */
void TIM2_IRQHandler(void) {
  if (TIM2->SR & TIM_SR_UIF) { // Check status register for update interrupt flag
    TIM2->SR &= ~(TIM_SR_UIF); // Reset the update interrupt flag
    Toggle_Pin(GPIOD, 15); // Toggle the LED output pin.
  }
}

int main() {
  uint8_t ADC_Val[32]; // Buffer for print messages
  Sysclock_168(); // Initalize 
  LED_Init();
  TIM2_Init();
  USART3_Init();
  ADC_Init();
  DMA_ADC1_Init(adc_buffer);

  while(1) {
    // ADC_Read(adc_value);
    if (adc_buffer[1] > 250) {
      GPIOD->ODR |= (1 << 12);
      GPIOD->ODR &= ~(1 << 13);
    } else {
      GPIOD->ODR &= ~(1 << 12);
      GPIOD->ODR |= (1 << 13);
    }
    sprintf(ADC_Val, "%d, %d\n", adc_buffer[1], adc_buffer[5]);
    send_String(USART3, ADC_Val);
  }
  return 0;
}
#include "stm32f407xx.h"
#include "cmsis_os2.h"

#include <stdio.h>

#include "uart.h"
#include "timer.h"
#include "gpio.h"

#define HSE_VALUE 8000000U

volatile int adc_value = 0;

void SysClock_Config();

void LED_Init() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // Enable GPIO D Clock

  // Set LED Pins to Output mode
  // Pins reset to push pull mode
  GPIOD->MODER &= ~GPIO_MODER_MODE12 & ~GPIO_MODER_MODE13 
                & ~GPIO_MODER_MODE14 & ~GPIO_MODER_MODE15; // Clear LED Pins
  GPIOD->MODER |= (0b01 << GPIO_MODER_MODE12_Pos) | (0b01 << GPIO_MODER_MODE13_Pos) 
                | (0b01 << GPIO_MODER_MODE14_Pos) | (0b01 << GPIO_MODER_MODE15_Pos); // Set LED Pins to output
}

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

/**
 * @brief Initialize ADC
 * 
 */
void ADC_Init() {
  ADC->CCR |= (0x1 << ADC_CCR_ADCPRE_Pos); // Set ADC Prescaler to 4 (84MHz / 4 = 21MHz)
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC1 Clock

  ADC1->CR2 |= ADC_CR2_ADON; // Enable ADC

  // Page 272 for GPIO Configuration
  // Set PA1-PA7 to Analog Mode
  GPIOA->MODER |= (0x3 << GPIO_MODER_MODE0_Pos) | (0x3 << GPIO_MODER_MODE1_Pos)
                | (0x3 << GPIO_MODER_MODE2_Pos) | (0x3 << GPIO_MODER_MODE3_Pos)
                | (0x3 << GPIO_MODER_MODE4_Pos) | (0x3 << GPIO_MODER_MODE5_Pos)
                | (0x3 << GPIO_MODER_MODE6_Pos) | (0x3 << GPIO_MODER_MODE7_Pos);
  
  GPIOB->MODER |= (0x3 << GPIO_MODER_MODE0_Pos) | (0x3 << GPIO_MODER_MODE1_Pos);

  GPIOC->MODER |= (0x3 << GPIO_MODER_MODE0_Pos) | (0x3 << GPIO_MODER_MODE1_Pos)
                | (0x3 << GPIO_MODER_MODE2_Pos) | (0x3 << GPIO_MODER_MODE3_Pos)
                | (0x3 << GPIO_MODER_MODE4_Pos) | (0x3 << GPIO_MODER_MODE5_Pos);

  // Clear Pull Up Pull Down Registers
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1 & ~GPIO_PUPDR_PUPDR2 & ~GPIO_PUPDR_PUPDR3
                & ~GPIO_PUPDR_PUPDR4 & ~GPIO_PUPDR_PUPDR5 & ~GPIO_PUPDR_PUPDR6
                & ~GPIO_PUPDR_PUPDR7;

  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR0 & ~GPIO_PUPDR_PUPDR1;

  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR0 & ~GPIO_PUPDR_PUPDR1 & ~GPIO_PUPDR_PUPDR2
                & ~GPIO_PUPDR_PUPDR3 & ~GPIO_PUPDR_PUPDR4 & ~GPIO_PUPDR_PUPDR5;

  ADC1->SQR1 |= 0xF; // Set Regular Sequence Length to 16
  
  // Set Conversion Sequence
  ADC1->SQR1 |= (0xF << ADC_SQR1_SQ16_Pos) | (0xE << ADC_SQR1_SQ15_Pos)
              | (0xD << ADC_SQR1_SQ14_Pos) | (0xC << ADC_SQR1_SQ13_Pos);
  ADC1->SQR2 |= (0xB << ADC_SQR2_SQ12_Pos) | (0xA << ADC_SQR2_SQ11_Pos)
              | (0x9 << ADC_SQR2_SQ10_Pos) | (0x8 << ADC_SQR2_SQ9_Pos)
              | (0x7 << ADC_SQR2_SQ8_Pos) | (0x6 << ADC_SQR2_SQ7_Pos);
  ADC1->SQR3 |= (0x5 << ADC_SQR3_SQ6_Pos) | (0x4 << ADC_SQR3_SQ5_Pos)
              | (0x3 << ADC_SQR3_SQ4_Pos) | (0x2 << ADC_SQR3_SQ3_Pos)
              | (0x1 << ADC_SQR3_SQ2_Pos) | (0x0 << ADC_SQR3_SQ1_Pos);

  // DMA Configuration
  // Use DMA2 Stream 0 Channel 0 for ADC1
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // Enable DMA2 Clock
  DMA2_Stream0->CR &= ~DMA_SxCR_EN; // Disable DMA Stream
  DMA2_Stream0->CR |= (0x0 << DMA_SxCR_CHSEL_Pos); // Set Channel to 0
  DMA2_Stream0->CR |= (0x0 << DMA_SxCR_DIR_Pos); // Set Direction to Peripheral to Memory
  DMA2_Stream0->CR |= (0x2 << DMA_SxCR_PL_Pos); // Set Priority to High
  DMA2_Stream0->CR |= (0x1 << DMA_SxCR_PSIZE_Pos); // Set Peripheral Data Size to 16-bits
  DMA2_Stream0->CR |= (0x1 << DMA_SxCR_MSIZE_Pos); // Set Memory Data Size to 16-bits
  DMA2_Stream0->CR |= (0x1 << DMA_SxCR_CIRC_Pos); // Set Circular Mode


  ADC1->CR2 |= ADC_CR2_CONT; // Set Continuous Conversion Mode
}

/**
 * @brief Read ADC PA1
 * 
 * @return int Value of ADC
 */
void ADC_Read() {
  ADC1->CR2 |= ADC_CR2_SWSTART; // Start Conversion
  while (!(ADC1->SR & ADC_SR_EOC)); // Wait for End of Conversion
  adc_value = ADC1->DR; // Return the Data Register
}

/**
 * @brief Starts the ADC DMA Conversion
 * 
 */
void ADC_Start() {
}

int main() {
  uint8_t message[] = "Hello World!\n";
  uint8_t ADC_Val[32];
  SysClock_Config();
  LED_Init();
  TIM2_Init();
  USART2_Init();
  ADC_Init();

  while(1) {
    ADC_Read();
    if (adc_value > 250) {
      GPIOD->ODR |= (1 << 12);
      GPIOD->ODR &= ~(1 << 13);
    } else {
      GPIOD->ODR &= ~(1 << 12);
      GPIOD->ODR |= (1 << 13);
    }
    sprintf(ADC_Val, "ADC: %d\n", adc_value);
    send_String(ADC_Val);
  }
  return 0;
}

void SysClock_Config() {
  RCC->CR |= RCC_CR_HSEON; // Enable HSE Clock
  while (!(RCC->CR & RCC_CR_HSERDY)); // Wait until HSE is ready

  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable Power Interface Clock
  PWR->CR |= PWR_CR_VOS; // Set Scale 1 mode (max clock frequency)

  RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // Set AHB Prescaler to 1
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // Set APB1 Prescaler to 4
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // Set APB2 Prescaler to 2

  RCC->PLLCFGR = 7 << RCC_PLLCFGR_PLLQ_Pos; // Set PLLQ to 7
  RCC->PLLCFGR |= 0 << RCC_PLLCFGR_PLLP_Pos; // Set PLLP to 2
  RCC->PLLCFGR |= 336 << RCC_PLLCFGR_PLLN_Pos; // Set PLLN to 336
  RCC->PLLCFGR |= 8 << RCC_PLLCFGR_PLLM_Pos; // Set PLLM to 8
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE; // Set PLL Source to HSE

  FLASH->ACR |= FLASH_ACR_PRFTEN; // Enable Prefetch Buffer
  FLASH->ACR |= FLASH_ACR_ICEN; // Enable Instruction Cache
  FLASH->ACR |= FLASH_ACR_DCEN; // Enable Data Cache
  FLASH->ACR |= FLASH_ACR_LATENCY_5WS; // Set Flash Latency to 5 Wait States

  RCC->CR |= RCC_CR_PLLON; // Enable PLL
  while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait until PLL is ready

  RCC->CFGR |= RCC_CFGR_SW_PLL; // Set PLL as System Clock
  while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)); // Wait until PLL is System Clock
  
  SystemCoreClock = 168000000; // Set System Clock to 168MHz
}
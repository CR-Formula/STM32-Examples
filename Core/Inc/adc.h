#include "stm32f407xx.h"

/**
 * @brief Initialize ADC
 * 
 */
void ADC_Init() {
  ADC->CCR |= (0x1 << ADC_CCR_ADCPRE_Pos); // Set ADC Prescaler to 4 (84MHz / 4 = 21MHz)
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC1 Clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIO A Clock

  // Page 272 for GPIO Configuration
  // Set to Analog Mode and disable Pull-up Pull-down
  GPIOA->MODER |= (0x3 << GPIO_MODER_MODE1_Pos); 
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;
  GPIOA->MODER |= (0x3 << GPIO_MODER_MODE4_Pos); 
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4; 
  GPIOA->MODER |= (0x3 << GPIO_MODER_MODE5_Pos); 
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5; 

  ADC1->SQR1 |= (0x2 << ADC_SQR1_L_Pos); // Set Regular Sequence Length to 3
  ADC1->SQR3 |= (0x1 << ADC_SQR3_SQ1_Pos) // Set Regular Sequence 1 to Channel 1
              | (0x4 << ADC_SQR3_SQ2_Pos) // Set Regular Sequence 2 to Channel 4
              | (0x5 << ADC_SQR3_SQ3_Pos); // Set Regular Sequence 3 to Channel 5

  ADC1->CR1 |= ADC_CR1_SCAN; // Enable Scan Mode
  ADC1->CR2 |= ADC_CR2_EOCS; // Enable End of Conversion Selection
  // ADC1->CR2 |= ADC_CR2_CONT; // Enable Continuous Conversion Mode

  ADC1->CR2 |= ADC_CR2_ADON; // Enable ADC
}

/**
 * @brief Read ADC PA1
 * 
 * @return int Value of ADC
 */
void ADC_Read(uint16_t *adc_value) {
  ADC1->CR2 |= ADC_CR2_SWSTART; // Start Conversion

  for (int i = 0; i < 3; i++) {
    while (!(ADC1->SR & ADC_SR_EOC)); // Wait for End of Conversion
    adc_value[i] = ADC1->DR; // Return the Data Register
  }
}
#include "stm32f407xx.h"
#include "cmsis_os2.h"

#include <stdio.h>

#include "sysclk.h"
#include "uart.h"
#include "timer.h"
#include "gpio.h"
#include "adc.h"

#define HSE_VALUE 8000000U


// Global Variables
/*----------------------------------------------------------------*/
uint8_t message[64];
volatile uint16_t adc_buffer[16];


// Function Prototypes
/*----------------------------------------------------------------*/
void SysClock_Config();
void ADCRead(void *argument);
void USART_Print(void *argument);
void Status_LED(void *argument);


// FreeRTOS Threads
/*----------------------------------------------------------------*/

// Prints out ADC values to USART3
osThreadId_t ADC_Read_Handle;
const osThreadAttr_t ADC_Read_Attr = {
  .name = "ADCRead",
  .stack_size = 128 * 4,
  .priority = osPriorityNormal
};

// Prints out "Hello World!" to USART3
osThreadId_t USART_Print_Handle;
const osThreadAttr_t USART_Print_Attr = {
  .name = "USART_Print",
  .stack_size = 128 * 4,
  .priority = osPriorityNormal
};

// Blinks the LED on PD14
osThreadId_t Status_LED_Handle;
const osThreadAttr_t Status_LED_Attr = {
  .name = "Status_LED",
  .stack_size = 128 * 4,
  .priority = osPriorityNormal
};

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

/**
 * @brief 
 * 
 */
void SPI_Init() {
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; // Enable SPI2 Clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB Clock

  SPI2->CR1 &= ~SPI_CR1_SPE; // Disable SPI

  GPIOB->MODER &= ~GPIO_MODER_MODE13 & ~GPIO_MODER_MODE14 
                & ~GPIO_MODER_MODE15; // Clear PB13, PB14, and PB15
  GPIOB->MODER |= (0x2 << GPIO_MODER_MODE13_Pos) 
                | (0x2 << GPIO_MODER_MODE14_Pos) 
                | (0x2 << GPIO_MODER_MODE15_Pos); // Set PB13, PB14, and PB15 to AF
  GPIOB->AFR[1] |= (0x5 << GPIO_AFRH_AFSEL13_Pos) 
                | (0x5 << GPIO_AFRH_AFSEL14_Pos) 
                | (0x5 << GPIO_AFRH_AFSEL15_Pos); // Set PB13, PB14, and PB15 to AF5 (SPI1)

  SPI2->CR1 |= (0x0 << SPI_CR1_BR_Pos); // Set Baud Rate to fPCLK/2 = 21MHz
  SPI2->CR1 |= SPI_CR1_CPOL; // Set Clock Polarity to 1
  SPI2->CR1 &= ~SPI_CR1_CPHA; // Set Clock Phase to 1
  SPI2->CR1 |= SPI_CR1_DFF; // Set Data Frame Format to 16-bit
  SPI2->CR1 &= ~SPI_CR1_LSBFIRST; // Set MSB First
  SPI2->CR1 |= SPI_CR1_SSM; // Set Software Slave Management
  SPI2->CR1 |= SPI_CR1_SSI; // Set Internal Slave Select
  SPI2->CR2 &= ~SPI_CR2_FRF; // Set Frame Format to Motorola
  SPI2->CR1 |= SPI_CR1_MSTR; // Set Master Mode
  SPI2->CR1 |= SPI_CR1_SPE; // Enable SPI
}

/**
 * @brief Main Function
 * 
 * @note Initializes the System Clock, Timer 2, USART3, ADC1, and DMA2
 * @return int 
 */
int main() {
  uint8_t ADC_Val[32];
  Sysclock_168();
  LED_Init();
  TIM2_Init();
  USART3_Init();
  ADC_Init();
  DMA_ADC1_Init(adc_buffer);

  osKernelInitialize(); // Initialize FreeRTOS

  ADC_Read_Handle = osThreadNew(ADCRead, NULL, &ADC_Read_Attr);
  USART_Print_Handle = osThreadNew(USART_Print, NULL, &USART_Print_Attr);
  Status_LED_Handle = osThreadNew(Status_LED, NULL, &Status_LED_Attr);

  osKernelStart(); // Start FreeRTOS

  while(1) {
    // Should not reach here
  }
  return 0;
}


// FreeRTOS Thread Functions
/*----------------------------------------------------------------*/

/**
 * @brief Thread for reading and printing ADC values
 * 
 * @param argument 
 */
void ADCRead(void *argument) {
  while(1) {
    if (adc_buffer[1] > 250) {
      GPIOD->ODR |= (1 << 12);
      GPIOD->ODR &= ~(1 << 13);
    } else {
      GPIOD->ODR &= ~(1 << 12);
      GPIOD->ODR |= (1 << 13);
    }
    sprintf(message, "%d, %d\n", adc_buffer[1], adc_buffer[5]);
    send_String(USART3, message);
    osDelay(50);
  }
}

/**
 * @brief Thread for printing message to USART3
 * 
 * @param argument 
 */
void USART_Print(void *argument) {
  while(1) {
    osDelay(1000);
    sprintf(message, "Hello World!\n");
    send_String(USART3, message);
  }
}

/**
 * @brief Thread for blinking the status led
 * 
 * @param argument 
 */
void Status_LED(void *argument) {
  while(1) {
    osDelay(1000);
    Toggle_Pin(GPIOD, 14);
  }
}
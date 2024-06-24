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
void SPI_Send(void *argument);
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

osThreadId_t SPI_Send_Handle;
const osThreadAttr_t SPI_Attr = {
  .name = "SPI_Send",
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
 * @brief Initialize SPI2
 * 
 */
void SPI2_Init() {
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; // Enable SPI2 Clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB Clock

  SPI2->CR1 &= ~SPI_CR1_SPE; // Disable SPI

  GPIOB->MODER &= ~GPIO_MODER_MODE12 & ~GPIO_MODER_MODE13
                & ~GPIO_MODER_MODE14 & ~GPIO_MODER_MODE15; // Clear PB13, PB14, and PB15
  GPIOB->MODER |= (0x2 << GPIO_MODER_MODE12_Pos)
                | (0x2 << GPIO_MODER_MODE13_Pos) 
                | (0x2 << GPIO_MODER_MODE14_Pos) 
                | (0x2 << GPIO_MODER_MODE15_Pos); // Set PB12, PB13, PB14, and PB15 to AF
  GPIOB->AFR[1] |= (0x5 << GPIO_AFRH_AFSEL12_Pos)
                | (0x5 << GPIO_AFRH_AFSEL13_Pos) 
                | (0x5 << GPIO_AFRH_AFSEL14_Pos) 
                | (0x5 << GPIO_AFRH_AFSEL15_Pos); // Set PB12, PB13, PB14, and PB15 to AF5 (SPI2)

  SPI2->CR1 = (0x4 << SPI_CR1_BR_Pos); // Set Baud Rate to fPCLK/32

  // Set CPOL = 1, SSM to software, SSI to high
  SPI2->CR1 |= SPI_CR1_CPOL | SPI_CR1_SSM 
            | SPI_CR1_SSI; // Set CPOL, DFF, SSM, and SSI

  // Set CPHA = 0, MSB First, Frame Format = Motorola, 8-bit Data
  SPI2->CR1 &= ~SPI_CR1_CPHA & ~SPI_CR1_LSBFIRST
            & ~SPI_CR2_FRF & ~SPI_CR1_DFF;
  
  SPI2->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE; // Set Master and Enable
}

uint8_t SPI_Write(SPI_TypeDef* SPI, uint8_t data, uint8_t device) {
  Toggle_Pin(GPIOB, 12); // Pull GPIOB12 High for CS
  while (!(SPI->SR & SPI_SR_TXE)); // Wait until TXE is set
  SPI->DR = data; // Write data to DR
  while (!(SPI->SR & SPI_SR_RXNE)); // Wait until RXNE is set
  uint8_t Read = SPI->DR; // Read DR to clear RXNE
  Toggle_Pin(GPIOB, 12); // Pull GPIOB12 Low for CS
  return Read;
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
  SPI2_Init();

  osKernelInitialize(); // Initialize FreeRTOS

  ADC_Read_Handle = osThreadNew(ADCRead, NULL, &ADC_Read_Attr);
  USART_Print_Handle = osThreadNew(USART_Print, NULL, &USART_Print_Attr);
  Status_LED_Handle = osThreadNew(Status_LED, NULL, &Status_LED_Attr);
  SPI_Send_Handle = osThreadNew(SPI_Send, NULL, &SPI_Attr);

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
 * @brief Send SPI Message over SPI2
 * 
 * @param argument 
 */
void SPI_Send(void *argument) {
  while(1) {
    SPI_Write(SPI2, 0xAA, 12);
    osDelay(100);
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
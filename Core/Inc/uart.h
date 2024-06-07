#include "stm32f407xx.h"

/**
 * @brief Initialize USART2
 * @note Baud rate = 115200
 */
void static inline USART2_Init() {
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART2 Clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIO A Clock

  USART2->CR1 &= ~USART_CR1_UE; // Disable USART

  GPIOA->MODER &= ~GPIO_MODER_MODE2 & ~GPIO_MODER_MODE3; // Clear PA2 and PA3
  GPIOA->MODER |= (0x2 << GPIO_MODER_MODE2_Pos) 
                | (0x2 << GPIO_MODER_MODE3_Pos); // Set PA2 and PA3 to TX and RX

  GPIOA->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED2_Pos) 
                  | (0x2 << GPIO_OSPEEDR_OSPEED3_Pos); // Set PA2 and PA3 to High Speed

  GPIOA->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL2_Pos) 
                  | (0x7 << GPIO_AFRL_AFSEL3_Pos); // Set PA2 and PA3 to AF7 (USART2)
  
  USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; // Enable Transmitter and Receiver

  // baud rate = fCK / (8 * (2 - OVER8) * USARTDIV)
  // Page 989 of Reference Manual
  // 115200 @ 42MHz = 22.8125
  USART2->BRR |= (0xD << USART_BRR_DIV_Fraction_Pos);
  USART2->BRR |= (0x16 << USART_BRR_DIV_Mantissa_Pos);

  USART2->CR1 |= USART_CR1_UE; // Enable USART
}

/**
 * @brief Sends a byte over USART2
 * 
 * @param byte Byte to send
 */
void static inline send_Byte(uint8_t byte) {
  USART2->DR = byte;
  while (!(USART2->SR & USART_SR_TC));
}

/**
 * @brief send a given string over USART2
 * 
 * @param string String to send
 */
void static inline send_String(uint8_t *string) {
  int i = 0;
  while (string[i] != '\0') {
    send_Byte(string[i]);
    i++;
  }
}
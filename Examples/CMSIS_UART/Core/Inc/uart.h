#include "stm32h743xx.h"

/**
 * @brief Initialize UART3
 * @note Baud rate: 115200, 8 data bits, 1 stop bit, no parity
 * @note Configured for 8Mhz Peripherial Clock
 * @note TODO: Look into FIFO Mode operation
 * @return 0 if successful, -1 if unsuccessful
 */
uint8_t static inline UART3_Init(void) {
    RCC->APB1LENR |= RCC_APB1LENR_USART3EN; // Enable USART3 clock
    USART3->CR1 &= USART_CR1_UE; // Make sure USART3 is disabled to configure

    USART3->CR1 &= ~USART_CR1_OVER8; // Set oversampling to 16
    USART3->CR1 &= ~USART_CR1_FIFOEN; // Disable FIFO mode

    // Set word length to 8 bits '00' in M0 and M1
    USART3->CR1 &= (0UL << USART_CR1_M1_Pos);
    USART3->CR1 &= (0UL << USART_CR1_M0_Pos);

    // Set Baud Rate to 115200
    USART3->BRR = (0x45 << USART_BRR_DIV_MANTISSA_Pos); // See pg 2037 of RM0433 Reference Manual
    USART3->CR2 &= (0UL << USART_CR2_STOP_Pos); // Set stop bits to 1
    USART3->CR1 |= USART_CR1_UE; // Enable USART3
    return 0;
}

/**
 * @brief Sends a character over UART
 * @note Page 2027 of RM0433 Reference Manual describes how to send data
 * @param ch character to send
 * @return 0 if successful, -1 if unsuccessful
 */
uint8_t static inline UART_SendChar(uint8_t data) {
    // Can only write to TDR when TXE/TXFNF is set
    USART3->CR1 |= USART_CR1_TE; // Enable transmitter
    while(!(USART3->ISR & USART_ISR_TXE_TXFNF)); // Wait for Transmit Data Register Empty flag
    USART3->TDR = data; // Write to TDR
    while (!(USART3->ISR & USART_ISR_TXE_TXFNF)); // Wait for Transmit Data Register Empty flag
    // TXE should be cleared by hardware when data is written to TDR
    while (!(USART3->ISR & USART_ISR_TC)); // Wait for Transmission Complete flag

    // TODO: TXE flag not being cleared
    // while ((USART3->ISR & USART_ISR_TXE_TXFNF)); // Wait for Transmit Data Register Empty flag
    USART3->CR1 &= ~USART_CR1_TE; // Disable transmitter
    return 0;
}

/**
 * @brief Sends a String over UART
 * @note Calls UART_SendChar for each character in the string
 * @param str string value to send
 * @return uint8_t 
 */
uint8_t static inline UART_SendString(char *str) {
    for (int i = 0; i < sizeof(str); i++){
        UART_SendChar(str[i]);
    }
    return 0;
}
void static inline UART_ReceiveChar(void) {

}
void static inline UART_ReceiveString(char *str) {

}
#include "stm32f407xx.h"

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
  GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED12_Pos)
                | (0x3 << GPIO_OSPEEDR_OSPEED13_Pos) 
                | (0x3 << GPIO_OSPEEDR_OSPEED14_Pos) 
                | (0x3 << GPIO_OSPEEDR_OSPEED15_Pos); // Set PB12, PB13, PB14, and PB15 to High Speed
  GPIOB->MODER |= (0x1 << GPIO_MODER_MODE12_Pos)
                | (0x2 << GPIO_MODER_MODE13_Pos) 
                | (0x2 << GPIO_MODER_MODE14_Pos)
                | (0x2 << GPIO_MODER_MODE15_Pos); // Set PB12, PB13, PB14, and PB15 to AF
  GPIOB->AFR[1] |= (0x5 << GPIO_AFRH_AFSEL13_Pos) 
                | (0x5 << GPIO_AFRH_AFSEL14_Pos) 
                | (0x5 << GPIO_AFRH_AFSEL15_Pos); // Set PB13, PB14, and PB15 to AF5 (SPI2)

  SPI2->CR1 = (0x4 << SPI_CR1_BR_Pos); // Set Baud Rate to fPCLK/32

  // Set CPOL = 1, SSM to software, SSI to high
  SPI2->CR1 |= SPI_CR1_CPOL | SPI_CR1_SSM 
            | SPI_CR1_SSI; // Set CPOL, DFF, SSM, and SSI

  // Set CPHA = 0, MSB First, Frame Format = Motorola, 8-bit Data
  SPI2->CR1 &= ~SPI_CR1_CPHA & ~SPI_CR1_LSBFIRST
            & ~SPI_CR2_FRF & ~SPI_CR1_DFF;
  
  SPI2->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE; // Set Master and Enable
}

/**
 * @brief 
 * 
 * @param SPI [SPI_TypeDef*] SPI Peripheral to use
 * @param data [uint8_t] Data to write
 * @param device [uint8_t] Chip Select Pin
 * @return uint8_t Data read from shift register
 */
uint8_t SPI_Write(SPI_TypeDef* SPI, uint8_t data, uint8_t device) {
  Clear_Pin(GPIOB, device); 
  GPIOB->ODR &= ~(1 << device); // Pull pin B12 Low for CS
  while (!(SPI->SR & SPI_SR_TXE)); // Wait until TXE is set
  SPI->DR = data; // Write data to Data Register
  while (!(SPI->SR & SPI_SR_RXNE)); // Wait until RXNE is set
  uint8_t Read = SPI->DR; // Read DR to clear RXNE
  Set_Pin(GPIOB, device); // Set pin B12 High for CS
  return Read;
}
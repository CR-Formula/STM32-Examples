/************************************************
* @file    i2c.c 
* @author  APBashara
* @date    6/2024
* 
* @brief   SPI Driver Implementation
***********************************************/

#include "stm32f407xx.h"

/**
 * @brief Initialize I2C1
 * @note 100kHz, 7-bit Addressing
 */
void I2C_Init() {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // Enable I2C1 Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB Clock

    I2C1->CR1 &= ~I2C_CR1_PE; // Disable I2C

    GPIOB->MODER &= ~GPIO_MODER_MODE6 & ~GPIO_MODER_MODE7; // Clear PB6 and PB7
    GPIOB->MODER |= (0x2 << GPIO_MODER_MODE6_Pos) | (0x2 << GPIO_MODER_MODE7_Pos); // Set PB6 and PB7 to AF
    GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED6_Pos) | (0x3 << GPIO_OSPEEDR_OSPEED7_Pos); // Set PB6 and PB7 to High Speed
    GPIOB->AFR[0] |= (0x4 << GPIO_AFRL_AFSEL6_Pos) | (0x4 << GPIO_AFRL_AFSEL7_Pos); // Set PB6 (SCL) and PB7 (SDA) to AF4

    I2C1->CR2 |= (42u << I2C_CR2_FREQ_Pos); // Set Peripheral Clock to 42MHz
}
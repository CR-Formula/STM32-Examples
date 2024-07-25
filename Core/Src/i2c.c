/************************************************
* @file    i2c.c 
* @author  APBashara
* @date    6/2024
* 
* @brief   I2C Driver Implementation
***********************************************/

#include "stm32f407xx.h"


/**
 * @brief Initialize I2C1
 * @note FM, 400kHz, 7-bit Addressing
 */
void I2C1_Init() {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // Enable I2C1 Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB Clock

    I2C1->CR1 &= ~I2C_CR1_PE; // Disable I2C

    GPIOB->MODER &= ~GPIO_MODER_MODE6 & ~GPIO_MODER_MODE7; // Clear PB6 and PB7
    GPIOB->MODER |= (0x2 << GPIO_MODER_MODE6_Pos) | (0x2 << GPIO_MODER_MODE7_Pos); // Set PB6 and PB7 to AF
    GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED6_Pos) | (0x3 << GPIO_OSPEEDR_OSPEED7_Pos); // Set PB6 and PB7 to High Speed
    GPIOB->AFR[0] |= (0x4 << GPIO_AFRL_AFSEL6_Pos) | (0x4 << GPIO_AFRL_AFSEL7_Pos); // Set PB6 (SCL) and PB7 (SDA) to AF4

    I2C1->CR1 |= I2C_CR1_ACK; // Enable Acknowledge
    I2C1->CR2 |= (42u << I2C_CR2_FREQ_Pos); // Set Peripheral Clock to 42MHz
    I2C1->CCR &= ~I2C_CCR_FS; // Set Fast Mode
    I2C1->CCR |= (6u << I2C_CCR_CCR_Pos); // Set Clock Control Register to 400kHz
    // Max Rise Time = 300ns, TpCLK = 1/42MHz = 23.8ns
    // (maximum_rise_time / TfPCLK1) + 1
    I2C1->TRISE |= 0xD; // Set Maximum Rise Time

    I2C1->CR1 |= I2C_CR1_PE; // Enable I2C
}

/**
 * @brief Send a data byte over I2C
 * 
 * @param I2C [I2C_TypeDef*] Peripheral to use
 * @param addr [uint8_t] Address of device [7-bit]
 * @param data [uint8_t] Data to send [8-bit]
 */
void I2C_Write(I2C_TypeDef* I2C, uint8_t addr, uint8_t data) {
    I2C->CR1 |= I2C_CR1_START; // Send Start Bit
    while (!(I2C->SR1 & I2C_SR1_SB)); // Wait for Start Bit
    I2C->DR = (addr << 1) & 0xFE; // Send Address with Write Bit
    while (!(I2C->SR1 & I2C_SR1_ADDR)); // Wait for Address to be sent
    (void) I2C->SR2; // Clear Address Flag
    I2C->DR = data; // Send Data
    while (!(I2C->SR1 & I2C_SR1_TXE)); // Wait for Data to be sent
    I2C->CR1 |= I2C_CR1_STOP; // Send Stop Bit
    while(!(I2C->CR1 & I2C_CR1_STOP)); // Wait for Stop Bit
}
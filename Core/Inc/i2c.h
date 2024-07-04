/************************************************
* @file    i2c.h 
* @author  APBashara
* @date    6/2024
* 
* @brief   Prototype Functions for I2C Driver
***********************************************/

#include "stm32f407xx.h"

/**
 * @brief Initialize I2C1
 * @note 100kHz, 7-bit Addressing
 */
void I2C1_Init();

/**
 * @brief Write a register on an I2C device
 * @note Designed for MPU6050
 * 
 * @param I2C [I2C_TypeDef*] I2C Peripheral to use
 * @param data [uint8_t] Data to write
 */
void I2C_Write(I2C_TypeDef* I2C, uint8_t data);
#include "stm32f407xx.h"

/**
 * @brief Toggle a given GPIO pin on or off
 * 
 * @param GPIO [GPIO_TypeDef*] GPIO Port to use
 * @param pin [uint8_t] Pin to toggle
 */
void static inline Toggle_Pin(GPIO_TypeDef* GPIO, uint8_t pin) {
  GPIO->ODR ^= (1 << pin);
}

/**
 * @brief Set a given GPIO pin high (Set to 1)
 * 
 * @param GPIO [GPIO_TypeDef*] GPIO Port to use
 * @param pin [uint8_t] Pin to set high
 */
void static inline Set_Pin(GPIO_TypeDef* GPIO, uint8_t pin) {
  GPIO->ODR |= (1 << pin);
}

/**
 * @brief Clear a given GPIO pin (Set to 0)
 * 
 * @param GPIO [GPIO_TypeDef*] GPIO Port to use
 * @param pin [uint8_t] Pin to set low
 */
void static inline Clear_Pin(GPIO_TypeDef* GPIO, uint8_t pin) {
  GPIO->ODR &= ~(1 << pin);
}
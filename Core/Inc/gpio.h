#include "stm32f407xx.h"

void static inline Toggle_Pin(GPIO_TypeDef* GPIO, int pin) {
  GPIO->ODR ^= (1 << pin);
}
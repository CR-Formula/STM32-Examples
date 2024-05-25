#include "stm32f407xx.h"
#include "cmsis_os2.h"

void SysClock_Config();

void LED_Init() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // Enable GPIO D Clock

  // Set LED Pins to Output mode
  // Pins reset to push pull mode
  GPIOD->MODER &= ~GPIO_MODER_MODE12 & ~GPIO_MODER_MODE13 
                & ~GPIO_MODER_MODE14 & ~GPIO_MODER_MODE15; // Clear LED Pins
  GPIOD->MODER |= (0b01 << GPIO_MODER_MODE12_Pos) | (0b01 << GPIO_MODER_MODE13_Pos) 
                | (0b01 << GPIO_MODER_MODE14_Pos) | (0b01 << GPIO_MODER_MODE15_Pos); // Set LED Pins to output
}

void Toggle_Pin(int pin) {
  GPIOD->ODR ^= (1 << pin);
}

void Delay_Temp() {
  for (int i = 0; i < 10000000; i++) {
    __NOP();
  }
}

void TIM2_IRQHandler(void) {
  // Handle a timer 'update' interrupt event
  if (TIM2->SR & TIM_SR_UIF) { // Check status register for update interrupt flag
    TIM2->SR &= ~(TIM_SR_UIF); // Reset the update interrupt flag
    Toggle_Pin(15); // Toggle the LED output pin.
  }
}

/**
 * @brief Initialize Timer 2
 * @note Timer 2 runs at APB1(42Mhz) * 2 = 84Mhz
 * @note Timer runs at fCK_PSC / (PSC + 1)
 */
void TIM2_Init() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 Clock

  TIM2->CR1 &= ~TIM_CR1_CEN; // Disable Timer
  TIM2->CR1 &= ~TIM_CR1_DIR; // Counting Up Direction

  NVIC_SetPriority(TIM2_IRQn, 2); // Set Priority to 2
  NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 Interrupt

  // Configure and start the Timer
  TIM2->PSC = 8400 - 1; // Set Prescaler to 8399 (10KHz)
  TIM2->ARR = 1000; // Set Auto Reload Register to 1000

  TIM2->DIER |= TIM_DIER_UIE; // Enable Update Interrupt
  
  TIM2->CR1 |= TIM_CR1_CEN; // Enable Timer
}

int main() {
  SysClock_Config();
  LED_Init();
  TIM2_Init();

  while(1) {
    Toggle_Pin(12);
    Delay_Temp();
    Toggle_Pin(13);
    Delay_Temp();
    Toggle_Pin(14);
    Delay_Temp();
  }
  return 0;
}

void SysClock_Config() {
  RCC->CR |= RCC_CR_HSEON; // Enable HSE Clock
  while (!(RCC->CR & RCC_CR_HSERDY)); // Wait until HSE is ready

  RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable Power Interface Clock
  PWR->CR |= PWR_CR_VOS; // Set Scale 1 mode (max clock frequency)

  RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // Set AHB Prescaler to 1
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // Set APB1 Prescaler to 4
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // Set APB2 Prescaler to 2

  RCC->PLLCFGR = 7 << RCC_PLLCFGR_PLLQ_Pos; // Set PLLQ to 7
  RCC->PLLCFGR |= 0 << RCC_PLLCFGR_PLLP_Pos; // Set PLLP to 2
  RCC->PLLCFGR |= 336 << RCC_PLLCFGR_PLLN_Pos; // Set PLLN to 336
  RCC->PLLCFGR |= 8 << RCC_PLLCFGR_PLLM_Pos; // Set PLLM to 8
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE; // Set PLL Source to HSE

  FLASH->ACR |= FLASH_ACR_PRFTEN; // Enable Prefetch Buffer
  FLASH->ACR |= FLASH_ACR_ICEN; // Enable Instruction Cache
  FLASH->ACR |= FLASH_ACR_DCEN; // Enable Data Cache
  FLASH->ACR |= FLASH_ACR_LATENCY_5WS; // Set Flash Latency to 5 Wait States

  RCC->CR |= RCC_CR_PLLON; // Enable PLL
  while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait until PLL is ready

  RCC->CFGR |= RCC_CFGR_SW_PLL; // Set PLL as System Clock
  while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)); // Wait until PLL is System Clock
  
  SystemCoreClock = 168000000; // Set System Clock to 168MHz
}
#include "main.h"
#include "stm32h743xx.h"
// usbipd list
// Its saying that time1 is being "optimized out" so I need to figure out why time1 isnt getting the value of the counter.

/*Note that if you change the IR_PIN value to something other than 
0 or 1, you will also need to use a different NVIC interrupt. 
The EXTI0_1 interrupt only works for pins 0 and 1. For pins 2-3 you can 
use EXTI2_3, and for pins 4-15 you can use EXTI4_15.*/


// #define IR_PIN 1

// int lap = 0;
//uint32_t time1 = 0;

// //void SystemClock_Config(void);
// void EXTI0_1_IRQ_handler(void);
// void GPIO_INTERRUPT_Init();
void TIMER_Init();
// //void GPIO_USART_Init();

// void IR() {
//   //ends timer
//   TIM2->CR1 = (0UL << TIM_CR1_CEN_Pos);
//   //time held in the timer (idk what the units are yet)
//   uint32_t time = (TIM2->CNT) / 3.750;
//   //resets timer
//   TIM2->CNT = (0UL << TIM_CNT_CNT_Pos);
//   //print to user the time of the lap
//   //int minutes = time / 60000;
//   //int seconds = (time % 60000) / 1000;
//   //int milliseconds = time % 1000;
//   lap++;
//   //print time
//   // while( !( USART2->ISR & USART_ISR_TXFE ) ) {};
//   //   USART2->TDR = time;
//   //starts timer
//   TIM2->CR1 = (1UL << TIM_CR1_CEN_Pos);
// }

int main(void)
{
  //HAL_Init();
  //SystemClock_Config();
  //added initializations
  //GPIO_INTERRUPT_Init();
  TIMER_Init();
  //GPIO_USART_Init();

  //start timer
  TIM2->CR1 = (1UL << TIM_CR1_CEN_Pos);
  while (1) {
    //time1 = TIM2->CNT;
    //time1 /= 3.750;
    //printf("%lu.....", time1);
    //for (volatile uint32_t i = 0; i < 1000000; ++i) {}
    // while( !( USART2->ISR & USART_ISR_TXFE ) ) {};
    //   USART2->TDR = time1;
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   /** Supply configuration update enable
//   */
//   HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

//   /** Configure the main internal regulator output voltage
//   */
//   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

//   while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//   RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
//   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
//                               |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//   RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
//   RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
//   RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
// }
// void EXTI0_1_IRQ_handler(void) {
//   if (EXTI->PR1 & (1 << IR_PIN)) {
//     //clear the EXTI status flag
//     EXTI->PR1 |= (1 << IR_PIN);
//     //determine stage
//     if(stage == 1){
//       stage = 0;
//     } else {
//       stage = 1;
//     }
//     //run the lap timer
//     IR();
//   }
// }
// void GPIO_INTERRUPT_Init(){
// //system config peripheral
//   //sensor information GPIO pin B
//   GPIOB->MODER  &= ~(0x3 << (IR_PIN*2));
//   GPIOB->PUPDR  &= ~(0x3 << (IR_PIN*2));
//   GPIOB->PUPDR  |=  (0x1 << (IR_PIN*2));
//   //enable and set SYSCFG to connect the IR EXTI line to GPIOB
//   RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
//   SYSCFG->EXTICR[(IR_PIN/4)] &= ~(0xF << ((IR_PIN % 4) * 4));
//   SYSCFG->EXTICR[(IR_PIN/4)] |=  (0x1 << ((IR_PIN % 4) * 4));
//   //setup the IR's EXTI line as an interrupt
//   EXTI->IMR1  |=  (1 << IR_PIN);
//   //disable the 'falling edge' trigger
//   EXTI->FTSR1 &=  ~(1 << IR_PIN);
//   //enable the 'rising edge' trigger
//   EXTI->RTSR1 |=  (1 << IR_PIN);
//   //Enable the NVIC interrupt for EXTI9 and EXTI5 at maximum priority.
//   NVIC_SetPriority(EXTI9_5_IRQn, 0);
//   NVIC_EnableIRQ(EXTI9_5_IRQn);
// }
void TIMER_Init(){
  //Enables TIM2?
  RCC->APB1LENR = (1UL << RCC_APB1LENR_TIM2EN_Pos);
  //Stops counting
  //TIM2->CR1 = (0UL << TIM_CR1_CEN_Pos);
  //Resets counter to 0
  //TIM2->CNT = 0000000000000000;
  //sets prescalar to 64000
  TIM2->PSC = 0xF9FF;
  TIM2->PSC = 0xF9FF;
  TIM2->PSC = 0xF9FF;
  //sets scalar to 1
  TIM2->CR1 = (00UL << TIM_CR1_CKD_Pos);
  //Makes timer count upwards
  TIM2->CR1 = (0UL << TIM_CR1_DIR_Pos);
}
// void GPIO_USART_Init(){
//   // Enable peripheral clocks: USART2.
//   RCC->APB1LENR  |=  ( RCC_APB1LENR_USART2EN );
//   //RCC->APB2ENR  |=  ( RCC_APB2ENR_IOPAEN );
//   // Configure pins A2, A3 for USART2.
//   // GPIOA->CRL &= ( GPIO_CRL_MODE2 | GPIO_CRL_CNF2 |
//   //   GPIO_CRL_MODE3 | GPIO_CRL_CNF3 );
//   // GPIOA->CRL |= ( ( 0x1 << GPIO_CRL_MODE2_Pos ) | 
//   //   ( 0x2 << GPIO_CRL_CNF2_Pos ) | ( 0x0 << GPIO_CRL_MODE3_Pos ) |
//   //   ( 0x1 << GPIO_CRL_CNF3_Pos ) );
//   //Sets baud rate
//   USART2->BRR = 480000000 / 115200;
//   //transmit and usart enable
//   USART2->CR1 |= (USART_CR1_TE | USART_CR1_UE );
// }

#include "main.h"
#include "stm32h743xx.h"
#include "cmsis_os.h"
#include "uart.h"
#include "gpio.h"

uint8_t data[] = "Hello World!\n"; // 8 Bit Buffer for text data

void BlinkStatusTask(void *argument);
void UARTTask(void *argument);

osThreadId_t BlinkStatusHandle;
const osThreadAttr_t BlinkStatus_attributes = {
  .name = "BlinkStatus",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

osThreadId_t UARTHandle;
const osThreadAttr_t UART_attributes = {
  .name = "UART",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};


int main() {
  // Initialize UART3
  UART3_Init();
  GPIOB_Init();
  osKernelInitialize();

  BlinkStatusHandle = osThreadNew(BlinkStatusTask, 0, &BlinkStatus_attributes);
  UARTHandle = osThreadNew(UARTTask, 99, &UART_attributes);

  osKernelStart();
  
  while(1) {}
  return 0;
}

void BlinkStatusTask(void *argument) {
  while(1) {
    // Toggle LED
    Toggle_Pin(GPIOB, 14);
    osDelay(1000);
  }
}

void UARTTask(void *argument) {
  while(1) {
    // Send data to UART3
    UART_SendString(data);
    osDelay(500);
  }
}
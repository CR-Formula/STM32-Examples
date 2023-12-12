/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author         : ABashara
  * @authors        : Cyclone Racing
  ******************************************************************************/

#include "main.h"
#include "cmsis_os2.h"
#include "stm32h725xx.h"

/* Private Variables */
/**
 * @brief Data Struct to hold Telem Data
 */
typedef struct data_struct {
  float RPM;        // RPM
  float TPS;        // TPS
  float FOT;        // Fuel Open Time
  float IA;         // Ignition Angle
  float Lam;        // Lambda
  float AirT;       // Air Temp
  float CoolT;      // Coolant Temp
  float Lat;        // Latitude
  float Lng;        // Longitude
  float Speed;      // GPS Speed
  float OilP;       // Oil Pressure
  float FuelP;      // Fuel Pressure
  float FLTemp;     // Front Left Brake Temp
  float FRTemp;     // Front Right Brake Temp
  float RLTemp;     // Rear Left Brake Temp
  float RRTemp;     // Rear Right Brake Temp
  float FRPot;      // Front Right Suspension Damper
  float FLPot;      // Front Left Suspension Damper
  float RRPot;      // Rear Right Suspension Damper
  float RLPot;      // Rear Left Suspension Damper
  float BrakeFront; // Front Brake Pressure
  float BrakeRear;  // Rear Brake Pressure
  float BrakeBias;  // Brake Bias
  float AccX;       // Accelerometer X Axis
  float AccY;       // Accelerometer Y Axis
  float AccZ;       // Accelerometer Z Axis
  float GyrX;       // Gyroscope X Axis
  float GyrY;       // Gyroscope Y Axis
  float GyrZ;       // Gyroscope Z Axis
  float MagX;       // Magnetometer X Axis
  float MagY;       // Magnetometer Y Axis
  float MagZ;       // Magnetometer Z Axis
} telemetry;

// Analog Variables
volatile uint16_t ADC_Buffer[16]; // DMA Buffer for Analog Data
volatile int ADC_Ready = 0;       // Flag for Analog Data Ready

// Dash Variables
char Dash_Message[64];
char HMIEnd[4] = "0xff"; // End of Message marker for Nextion

/* Create RTOS Tasks (Threads) */
/**
 * @brief RTOS Task for defaultTask
 * @retval None
*/
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/**
 * @brief Task for Reading ECU CAN Data
 * @retval None
 */
osThreadId_t CAN_ReadHandle;
const osThreadAttr_t CAN_Read_attributes = {
  .name = "CAN_Read",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/**
 * @brief Task for writing CAN Data
 * @retval None
 */
osThreadId_t CAN_WriteHandle;
const osThreadAttr_t CAN_Write_attributes = {
  .name = "CAN_Write",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/**
 * @brief Task for reading Analog DMA Buffer
 * @retval None
 */
osThreadId_t Analog_ReadHandle;
const osThreadAttr_t Analog_Read_attributes = {
  .name = "Analog_Read",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/**
 * @brief Task for Blinking Status LED
 * @retval None
 */
osThreadId_t Status_LEDHandle;
const osThreadAttr_t Status_LED_attributes = {
  .name = "Status_LED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/**
 * @brief Task for Sending Data over LoRa
 * @retval None
 */
osThreadId_t LoRaHandle;
const osThreadAttr_t LoRa_attributes = {
  .name = "LoRa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/**
 * @brief Task for Reading GPS Data
 * @retval None
 */
osThreadId_t GPSHandle;
const osThreadAttr_t GPS_attributes = {
  .name = "GPS",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/**
 * @brief Task for Logging to uSD Card
 * @retval None
 */
osThreadId_t SD_LogHandle;
const osThreadAttr_t SD_Log_attributes = {
  .name = "SD_Log",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/**
 * @brief Task for reading IMU Data
 * @retval None
 */
osThreadId_t IMUHandle;
const osThreadAttr_t IMU_attributes = {
  .name = "IMU",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/**
 * @brief Task for sending Dash Data
 * @retval None
 */
osThreadId_t DashHandle;
const osThreadAttr_t Dash_attributes = {
  .name = "Dash",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask(void *argument);
void StartCAN_Read(void *argument);
void StartCAN_Write(void *argument);
void StartAnalog_Read(void *argument);
void StartStatus_LED(void *argument);
void StartLoRa(void *argument);
void StartGPS(void *argument);
void StartSD_Log(void *argument);
void StartIMU(void *argument);
void StartDash(void *argument);

/**
  * @brief  Configures the MCU and Peripherals
  * @retval int
  */
int main(void) {
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

  /* Init scheduler */
  osKernelInitialize();

  /* add mutexes, ... */

  /* add semaphores, ... */

  /* start timers, add new ones, ... */

  /* add queues, ... */

  /* Create the RTOS threads */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  CAN_ReadHandle = osThreadNew(StartCAN_Read, NULL, &CAN_Read_attributes);
  CAN_WriteHandle = osThreadNew(StartCAN_Write, NULL, &CAN_Write_attributes);
  Analog_ReadHandle = osThreadNew(StartAnalog_Read, NULL, &Analog_Read_attributes);
  Status_LEDHandle = osThreadNew(StartStatus_LED, NULL, &Status_LED_attributes);
  LoRaHandle = osThreadNew(StartLoRa, NULL, &LoRa_attributes);
  GPSHandle = osThreadNew(StartGPS, NULL, &GPS_attributes);
  SD_LogHandle = osThreadNew(StartSD_Log, NULL, &SD_Log_attributes);
  IMUHandle = osThreadNew(StartIMU, NULL, &IMU_attributes);
  DashHandle = osThreadNew(StartDash, NULL, &Dash_attributes);

  /* add threads, ... */

  /* add events, ... */

  /* Start scheduler */
  osKernelStart();

  while (1) {
    Error_Handler(); // Should not access the while loop as RTOS should take control
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartDefaultTask(void *argument) {
  for(;;) {
    osDelay(1);
  }
}

/**
 * @brief Starts CAN Read Thread
 * @param argument: Not used
 * @retval None
 */
void StartCAN_Read(void *argument) {
  for(;;) {
    osDelay(1);
  }
}

/**
 * @brief Starts CAN Write Thread
 * @param argument: Not used
 * @retval None
 */
void StartCAN_Write(void *argument) {
  for(;;) {
    osDelay(1);
  }
}

/**
 * @brief Starts Analog Read Thread
 * @param argument: Not used
 * @retval None
 */
void StartAnalog_Read(void *argument) {
  for(;;) {
    osDelay(1);
  }
}

/**
 * @brief Starts Status LED Thread
 * @param argument: Not used
 * @retval None
 */
void StartStatus_LED(void *argument) {
  for(;;) {
    osDelay(1);
  }
}

/**
 * @brief Starts LoRa Thread
 * @param argument: Not used
 * @retval None
 */
void StartLoRa(void *argument) {
  for(;;) {
    osDelay(1);
  }
}

/**
 * @brief Starts GPS Thread
 * @param argument: Not used
 * @retval None
 */
void StartGPS(void *argument) {
  for(;;) {
    osDelay(1);
  }
}

/**
 * @brief Starts SD Log Thread
 * @param argument: Not used
 * @retval None
 */
void StartSD_Log(void *argument) {
  for(;;) {
    osDelay(1);
  }
}

/**
 * @brief Starts IMU Thread
 * @param argument: Not used
 * @retval None
 */
void StartIMU(void *argument) {
  for(;;) {
    osDelay(1);
  }
}

/**
 * @brief Starts Dash Thread
 * @param argument: Not used
 * @retval None
 */
void StartDash(void *argument) {
  for(;;) {
    osDelay(1);
  }
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  __disable_irq();
  while (1) {
    // Kick to this loop if there is an error
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif

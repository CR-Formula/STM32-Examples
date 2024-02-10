#include "main.h"
#include "stm32h743xx.h"

void SystemClock_Config(void);

void SPI_Config() {
  SPI1->CFG2 = (0UL << SPI_CFG2_COMM_Pos);
}
typedef struct data_struct {
  float RPM; //RPM Value
  float TPS; //TPS Value
  float FOT; //Fuel Open Time Value
  float IA; //Ignition Angle
  float Lam; //Lambda
  float AirT; //Air Temp
  float CoolT; //Coolent Temp
  float Lat; //Latitude
  float Lng; // Longitude  
  float Speed; //GPS Speed
  float OilP; //Oil Pressure
  float FuelP; //Fuel Pressure
  float FLTemp; //Brake Temps
  float FRTemp;
  float RLTemp;
  float RRTemp;
  float FRPot; //Suspension Dampeners
  float FLPot;
  float RRPot;
  float RLPot;
  float BrakeFront; //Brake Pressures
  float BrakeRear;
  float BrakeBias; //Brake Bias
  float AccX; //Acclerometer
  float AccY;
  float AccZ;
  float GyrX; //Gyroscope
  float GyrY;
  float GyrZ;
  float MagX; //Magnetometer
  float MagY;
  float MagZ;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  HAL_GPIO_Init();

  HAL_Delay(1000);
  FATFS FatFs;
  FIL fil;
  FRESULT fres;

  fres=f_mount(&FatFs, "", 1);

  while (1) {

  }
  return 0;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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


git config --global user.email "you@example.com"
  git config --global user.name "Your Name"

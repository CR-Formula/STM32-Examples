#include "main.h"
#include "stm32h743xx.h"
#include <stdio.h>
#include <string.h>
#include <ff.h>




void SystemClock_Config(void);

void SPI_Config() 
/**
 * to configure:
 * SPI4_SCK
 * SPI4_NSS
 * SPI4_MISO
 * SPI4_MOSI
 * 
 * SPI1_SCK
 * SPI1_MISO
 * SPI1_MOSI
 * 
 * SPI3_SCK
 * SPI3_MISO
 * SPI3_NSS
 * SPI3_MOSI
 * 
 * along with what is required to be configured from RM0433 sheet
*/
{
  SPI1->CFG2 = (0UL << SPI_CFG2_COMM_Pos);  
}
typedef struct {
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
} data_struct;
data_struct telemetry;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();


  HAL_Delay(1000);

  FATFS FatFs;
  FIL fil;
  FRESULT fres;

//open the file system
  fres=f_mount(&FatFs, "", 1);
  if (fres != FR_OK){
    while(1);
    //error occurred while mounting the drive
  }

  //get some stats from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS* getFreeFs;

  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK){
    myprintf("f_getfree error (%i) \r\n", fres);
    while(1);
  }

  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  myprintf("SD card stats: \r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

  char readBuf[120];
  char *ptr = readBuf;
  int ret;

  fres = f_open(&fil, "test.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  if(fres == FR_OK) {
    myprintf("File opened successfully. \r\n");
  } else {
    myprintf("File open error (%i) \r\n", fres);
    while(1);
  }

  while (1) {
   ret = sprintf(ptr, "%f, %f, %f, %f, %f, %f, %f, %f,", telemetry.RPM, telemetry.TPS, telemetry.FOT, telemetry.IA, telemetry.Lam, telemetry.AirT, telemetry.CoolT, telemetry.Lat);
ptr += ret;
ret = sprintf(ptr, "%f, %f, %f, %f, %f, %f, %f, %f,", telemetry.Lng, telemetry.Speed, telemetry.OilP, telemetry.FuelP, telemetry.FLTemp, telemetry.FRTemp, telemetry.RLTemp, telemetry.RRTemp);
ptr += ret;
ret = sprintf(ptr, "%f, %f, %f, %f, %f, %f, %f, %f,", telemetry.FRPot, telemetry.FLPot, telemetry.RRPot, telemetry.RLPot, telemetry.BrakeFront, telemetry.BrakeRear, telemetry.BrakeBias, telemetry.AccX);
ptr += ret;
ret = sprintf(ptr, "%f, %f, %f, %f, %f, %f, %f, %f", telemetry.AccY, telemetry.AccZ, telemetry.GyrX, telemetry.GyrY, telemetry.GyrZ, telemetry.MagX, telemetry.MagY, telemetry.MagZ);
uint8_t bytesWrote; //edited from BYTE to uint8_t
f_write(&fil, readBuf, strlen(readBuf), &bytesWrote);
if(fres == FR_OK) {
  f_printf(&fil, "File written successfully. Wrote %i bytes to file. \r\n", bytesWrote);
  }else {
    myprintf("File write error (%i) \r\n", fres);
    while(1);
  }
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
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FDCAN_TxHeaderTypeDef TxHeader1;
FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t TxData1[8];
uint8_t RxData1[8];

FDCAN_TxHeaderTypeDef TxHeader2;
FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t TxData2[8];
uint8_t RxData2[8];

typedef struct Telemetry {
    int rpm;
    int tps;
    int fuelOpenTime;
    int ignistionAngle;
    int lam;
    int airTemp;
    int coloingTemp;

} Telemetry;

Telemetry telemetry;
uint32_t received_ID;

void update_data(uint8_t *data) {
    if (received_ID == 0x0CFF048) {
        telemetry.rpm = data[0] + data[1] * 256;
        telemetry.tps = data[2] + data[3] * 256;
        telemetry.fuelOpenTime = data[4] + data[5] * 256;
        telemetry.ignistionAngle = data[6] + data[7] * 256;
    }
    if (received_ID == 0x0CFF148) {
        telemetry.lam = data[4] + data[5] * 256;
    }
    if (received_ID == 0x0CFF548) {
        telemetry.airTemp = data[2] + data[3] * 256;
        telemetry.coloingTemp = data[4] + data[5] * 256;
    }
}

// called when msg is recived and went through filter
void HAL_FDCAN_RxFio0Callback(FDCAN_HandleTypeDef *hfdcan1,
                              uint32_t RxFiofo0ITs) {

    // Check if there's a new message in RX FIFO0
    while ((RxFiofo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {

        // Get the message from RX FIFO0
        if (HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &RxHeader1,
                                   RxData1) != HAL_OK) {
            Error_Handler();
        }

        // Process the message
        received_ID = RxHeader1.Identifier;
        update_data(RxData1);

        // Check again for more messages in RX FIFO0
        RxFiofo0ITs = HAL_FDCAN_GetRxFifoFillLevel(hfdcan1, FDCAN_RX_FIFO0);
    }

    // Reactivate notifications for RX FIFO0 to recive msg again
    if (HAL_FDCAN_ActivateNotification(
            hfdcan1, FDCAN_RX_FIFO0, FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != HAL_OK) {
        Error_Handler();
    }
}

// for function as above but for hfdcan2
void HAL_FDCAN_RxFio1Callback(FDCAN_HandleTypeDef *hfdcan2,
                              uint32_t RXFiofo1ITs) {
    while ((RXFiofo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {

        if (HAL_FDCAN_GetRxMessage(hfdcan2, FDCAN_RX_FIFO1, &RxHeader2,
                                   RxData2) != HAL_OK) {
            Error_Handler();
        }

        received_ID = RxHeader2.Identifier;
        update_data(RxData2);

        RXFiofo1ITs = HAL_FDCAN_GetRxFifoFillLevel(hfdcan2, FDCAN_RX_FIFO1);
    }

    if (HAL_FDCAN_ActivateNotification(
            hfdcan2, FDCAN_RX_FIFO0, FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != HAL_OK) {
        Error_Handler();
    }
}

// temp fucntion to replicate the ecu sim
void packet1(double rpm, double tps, double fuelOpenTime,
             double ignistionAngle) {

    TxData2[0] = (rpm / 2);
    TxData2[1] = (rpm / 2) / 256;
    TxData2[2] = tps / 2;
    TxData2[3] = (tps / 2) / 256;
    TxData2[4] = fuelOpenTime / 2;
    TxData2[5] = (fuelOpenTime / 2) / 256;
    TxData2[6] = ignistionAngle / 2;
    TxData2[7] = (ignistionAngle / 2) / 256;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_FDCAN1_Init();
    MX_FDCAN2_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    // initalizes can
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    // initalizes can
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
        Error_Handler();
    }
    // ready to recive msgs
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0) != HAL_OK) {
        Error_Handler();
    }
    // ready to recive msgs
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
                                       0) != HAL_OK) {
        Error_Handler();
    }

    TxHeader1.Identifier = 0x0cff104;
    TxHeader1.IdType = FDCAN_EXTENDED_ID;
    TxHeader1.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader1.DataLength = FDCAN_DATA_BYTES_8;
    TxHeader1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader1.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader1.FDFormat = FDCAN_FD_CAN;
    TxHeader1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader1.MessageMarker = 0;

    TxHeader2.Identifier = 0x0CFF048;
    TxHeader2.IdType = FDCAN_EXTENDED_ID;
    TxHeader2.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader2.DataLength = FDCAN_DATA_BYTES_8;
    TxHeader2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader2.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader2.FDFormat = FDCAN_FD_CAN;
    TxHeader2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader2.MessageMarker = 0;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */

        // test function replicating the ecu simulatior
        packet1(100.000, 28.0000, 25.0000, 15.0000);

        // sends data from test fdcan
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, TxData2) !=
            HAL_OK) {
            Error_Handler();
        }
        HAL_Delay(1000);

        // test the RXdata1 (data buffer that will recive data from main can)
        //        HAL_UART_Transmit(&huart2, RxData1, 8,
        //                          HAL_MAX_DELAY);

        // test the telemptry struct
        HAL_UART_Transmit(&huart2, (uint8_t *)telemetry.rpm,
                          sizeof((uint8_t *)telemetry.rpm), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (uint8_t *)telemetry.tps,
                          sizeof((uint8_t *)telemetry.tps), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (uint8_t *)telemetry.fuelOpenTime,
                          sizeof((uint8_t *)telemetry.fuelOpenTime),
                          HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (uint8_t *)telemetry.ignistionAngle,
                          sizeof((uint8_t *)telemetry.ignistionAngle),
                          HAL_MAX_DELAY);
    }
    /* USER CODE END 3 */
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
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 9;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 3;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
    RCC_OscInitStruct.PLL.PLLFRACN = 3072;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                  RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void) {

    /* USER CODE BEGIN FDCAN1_Init 0 */

    /* USER CODE END FDCAN1_Init 0 */

    /* USER CODE BEGIN FDCAN1_Init 1 */

    /* USER CODE END FDCAN1_Init 1 */
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    hfdcan1.Init.NominalPrescaler = 1;
    hfdcan1.Init.NominalSyncJumpWidth = 13;
    hfdcan1.Init.NominalTimeSeg1 = 86;
    hfdcan1.Init.NominalTimeSeg2 = 13;
    hfdcan1.Init.DataPrescaler = 25;
    hfdcan1.Init.DataSyncJumpWidth = 1;
    hfdcan1.Init.DataTimeSeg1 = 2;
    hfdcan1.Init.DataTimeSeg2 = 1;
    hfdcan1.Init.MessageRAMOffset = 0;
    hfdcan1.Init.StdFiltersNbr = 0;
    hfdcan1.Init.ExtFiltersNbr = 3;
    hfdcan1.Init.RxFifo0ElmtsNbr = 3;
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxFifo1ElmtsNbr = 0;
    hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxBuffersNbr = 0;
    hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.TxEventsNbr = 0;
    hfdcan1.Init.TxBuffersNbr = 0;
    hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN FDCAN1_Init 2 */
// ids that will be sent over can
#define ID1 0x0CFF048;
#define ID2 0x0CFF148;
#define ID3 0x0CFF548;

    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = ID1;
    sFilterConfig.FilterID2 = 0x1FFFFFFF;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    // Filter for ID2
    sFilterConfig.FilterID1 = ID2;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    // Filter for ID3
    sFilterConfig.FilterID1 = ID3;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE END FDCAN1_Init 2 */
}

/**
 * @brief FDCAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN2_Init(void) {

    /* USER CODE BEGIN FDCAN2_Init 0 */

    /* USER CODE END FDCAN2_Init 0 */

    /* USER CODE BEGIN FDCAN2_Init 1 */

    /* USER CODE END FDCAN2_Init 1 */
    hfdcan2.Instance = FDCAN2;
    hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan2.Init.AutoRetransmission = ENABLE;
    hfdcan2.Init.TransmitPause = DISABLE;
    hfdcan2.Init.ProtocolException = DISABLE;
    hfdcan2.Init.NominalPrescaler = 1;
    hfdcan2.Init.NominalSyncJumpWidth = 13;
    hfdcan2.Init.NominalTimeSeg1 = 86;
    hfdcan2.Init.NominalTimeSeg2 = 13;
    hfdcan2.Init.DataPrescaler = 25;
    hfdcan2.Init.DataSyncJumpWidth = 1;
    hfdcan2.Init.DataTimeSeg1 = 2;
    hfdcan2.Init.DataTimeSeg2 = 1;
    hfdcan2.Init.MessageRAMOffset = 1240;
    hfdcan2.Init.StdFiltersNbr = 0;
    hfdcan2.Init.ExtFiltersNbr = 1;
    hfdcan2.Init.RxFifo0ElmtsNbr = 0;
    hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan2.Init.RxFifo1ElmtsNbr = 1;
    hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan2.Init.RxBuffersNbr = 0;
    hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    hfdcan2.Init.TxEventsNbr = 0;
    hfdcan2.Init.TxBuffersNbr = 0;
    hfdcan2.Init.TxFifoQueueElmtsNbr = 1;
    hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN FDCAN2_Init 2 */
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x1FFFFFFF;
    sFilterConfig.FilterID2 = 0x1FFFFFFF;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE END FDCAN2_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00707CBB;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) !=
        HAL_OK) {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) !=
        HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) !=
        HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

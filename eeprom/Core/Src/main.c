/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "exteprom.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_SYS_CLK_MHZ   16u      /* system clock in MHz (HSI16, no PLL) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* ===== ALL GLOBAL -> add them to Live Expressions ===== */

/* ---- the monitor array ---- */
uint8_t  ee_dump[EXTEPROM_SIZE_BYTES];       /* WHOLE eeprom: read at reset, then again after every write/erase */
uint8_t  ee_dump_boot[EXTEPROM_SIZE_BYTES];  /* copy of the eeprom as it was at reset (before any write)       */
uint32_t ee_dump_count = 0;                  /* +1 each time ee_dump is refreshed                              */
volatile EXTEPROM_Status_t ee_dump_status = EXTEPROM_ERR_NOT_INIT;  /* status of that read (0 = OK)          */
uint8_t  test_step = 0;                      /* which test step is running (1..7)                              */

/* ---- test data ---- */
uint8_t  ee_wr_data[21];                     /* 30, 31, ... 50                                                 */
uint8_t  ee_wr_small[5] = { 1, 2, 3, 4, 5 };
uint8_t  ee_rd_byte = 0;                     /* result of the direct single-byte read                          */
uint8_t  ee_rd_page[EXTEPROM_PAGE_SIZE_BYTES]; /* result of the direct page read                               */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void EEPROM_Debug_ReadAll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Debug function: reads the WHOLE eeprom into ee_dump.
 * Called once at reset, then after every write / erase. */
static void EEPROM_Debug_ReadAll(void)
{
  EXTEPROM_Data_Direct_Read(0, ee_dump, EXTEPROM_SIZE_BYTES);
  ee_dump_status = EXTEPROM_Status_Get();
  ee_dump_count++;                 /* <- put a breakpoint on this line to step through the tests */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* 1. init the driver with the system clock in MHz */
  test_step = 1;
  EXTEPROM_init(EEPROM_SYS_CLK_MHZ);

  /* 2. read the whole eeprom at reset and keep a copy of what it held */
  test_step = 2;
  EEPROM_Debug_ReadAll();
  memcpy(ee_dump_boot, ee_dump, sizeof(ee_dump_boot));

  /* 3. write 30..50 at addresses 30..50 (crosses 3 page edges: 32, 40, 48) */
  test_step = 3;
  for (uint8_t i = 0; i < sizeof(ee_wr_data); i++)
  {
    ee_wr_data[i] = (uint8_t)(30u + i);
  }
  EXTEPROM_Data_Direct_Write(30, ee_wr_data, sizeof(ee_wr_data));
  EEPROM_Debug_ReadAll();

  /* 4. direct reads: byte 33 must be 33, page 4 (addresses 32..39) must be 32..39 */
  test_step = 4;
  EXTEPROM_Data_Direct_Read(33, &ee_rd_byte, 1);
  EXTEPROM_Data_Direct_Read(32, ee_rd_page, EXTEPROM_PAGE_SIZE_BYTES);

  /* 5. erase ONE byte: address 40 becomes 255, its neighbours stay */
  test_step = 5;
  EXTEPROM_Data_Byte_Erase(40);
  EEPROM_Debug_ReadAll();

  /* 6. erase ONE block (block 4 = addresses 32..39) */
  test_step = 6;
  EXTEPROM_flashData_erase(4);
  EEPROM_Debug_ReadAll();

  /* 7. write 5 bytes at address 6 (crosses the page edge at 8) */
  test_step = 7;
  EXTEPROM_Data_Direct_Write(6, ee_wr_small, sizeof(ee_wr_small));
  EEPROM_Debug_ReadAll();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
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
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

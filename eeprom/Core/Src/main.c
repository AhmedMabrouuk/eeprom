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
#include "eeprom.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Commands you write into dbg_cmd from Live Expressions */
typedef enum
{
  DBG_CMD_NONE       = 0,
  DBG_CMD_READ_BYTE  = 1,   /* read byte at dbg_addr           -> dbg_byte  */
  DBG_CMD_READ_PAGE  = 2,   /* read page number dbg_page_no    -> dbg_page  */
  DBG_CMD_READ_ALL   = 3,   /* read all 256 bytes              -> dbg_all   */
  DBG_CMD_WRITE_TEST = 4,   /* (re)write the 30..50 test array              */
  DBG_CMD_ERASE_ALL  = 5    /* fill the whole chip with 0xFF                */
} dbg_cmd_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_START_ADDR   30u
#define TEST_END_ADDR     50u                                   /* inclusive */
#define TEST_LEN          (TEST_END_ADDR - TEST_START_ADDR + 1u) /* = 21     */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* All of these are GLOBAL on purpose -> add them to Live Expressions */

eeprom_t ee;                          /* driver object                        */

/* ---- test pattern ---- */
uint8_t  ee_tx[TEST_LEN];             /* what we write: ee_tx[i] = 30 + i     */
uint8_t  ee_rx[TEST_LEN];             /* what we read back from addr 30..50   */

/* ---- command interface (you WRITE these in Live Expressions) ---- */
volatile uint8_t  dbg_cmd     = DBG_CMD_NONE;  /* set 1..5, returns to 0 when done */
volatile uint16_t dbg_addr    = 33;            /* byte address for READ_BYTE       */
volatile uint8_t  dbg_page_no = 0;             /* 0..31 for READ_PAGE              */

/* ---- results (you READ these) ---- */
uint8_t  dbg_byte             = 0;             /* result of READ_BYTE              */
uint8_t  dbg_page[EEPROM_PAGE_SIZE];           /* result of READ_PAGE (8 bytes)    */
uint8_t  dbg_all[EEPROM_SIZE];                 /* result of READ_ALL (256 bytes)   */

volatile eeprom_status_t dbg_status = EEPROM_OK; /* status of the LAST driver call */
uint8_t  dbg_verify_ok        = 0;   /* 1 = every addr 30..50 holds its own value */
uint16_t dbg_mismatch_count   = 0;   /* how many of addr 30..50 are wrong         */
uint32_t dbg_cmd_done_count   = 0;   /* increments after each command finishes    */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void EEPROM_Boot(void);
static void DBG_Service(void);
static void DBG_CheckRegion(const uint8_t *img);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Compare a full 256-byte image against the rule "address N holds value N"
 * for N = 30..50 only. (The rest of the chip is not part of the test.) */
static void DBG_CheckRegion(const uint8_t *img)
{
  uint16_t bad = 0;

  for (uint16_t a = TEST_START_ADDR; a <= TEST_END_ADDR; a++)
  {
    if (img[a] != (uint8_t)a)
    {
      bad++;
    }
  }
  dbg_mismatch_count = bad;
  dbg_verify_ok      = (bad == 0u) ? 1u : 0u;
}

/* Runs once at boot: build the pattern, store it, read it back, verify. */
static void EEPROM_Boot(void)
{
  /* 1. Build pattern: ee_tx[0]=30, ee_tx[1]=31 ... ee_tx[20]=50 */
  for (uint16_t i = 0; i < TEST_LEN; i++)
  {
    ee_tx[i] = (uint8_t)(TEST_START_ADDR + i);
  }

  /* 2. Init driver (probes the chip) */
  dbg_status = eeprom_init(&ee, &hi2c1);
  if (dbg_status != EEPROM_OK) { return; }

  /* 3. Store at EEPROM addresses 30..50.
   *    update() instead of write(): if the chip already holds the pattern
   *    (e.g. after a reset) nothing is rewritten, so no wear. */
  dbg_status = eeprom_update(&ee, TEST_START_ADDR, ee_tx, TEST_LEN);
  if (dbg_status != EEPROM_OK) { return; }

  /* 4. Read the same region back */
  dbg_status = eeprom_read(&ee, TEST_START_ADDR, ee_rx, TEST_LEN);
  if (dbg_status != EEPROM_OK) { return; }

  /* 5. Read the whole chip so you can inspect it right away, then verify */
  dbg_status = eeprom_read(&ee, 0, dbg_all, EEPROM_SIZE);
  if (dbg_status != EEPROM_OK) { return; }

  DBG_CheckRegion(dbg_all);
}

/* Polled from the main loop. Write a command into dbg_cmd (Live Expressions),
 * this executes it and clears dbg_cmd back to 0 when finished. */
static void DBG_Service(void)
{
  uint8_t cmd = dbg_cmd;

  if (cmd == DBG_CMD_NONE)
  {
    return;
  }

  switch (cmd)
  {
    case DBG_CMD_READ_BYTE:
      dbg_status = eeprom_read(&ee, dbg_addr, &dbg_byte, 1);
      break;

    case DBG_CMD_READ_PAGE:
      /* page n starts at n*8. page 32 -> addr 256 -> driver returns ERR_RANGE */
      dbg_status = eeprom_read(&ee, (uint16_t)(dbg_page_no * EEPROM_PAGE_SIZE),
                               dbg_page, EEPROM_PAGE_SIZE);
      break;

    case DBG_CMD_READ_ALL:
      dbg_status = eeprom_read(&ee, 0, dbg_all, EEPROM_SIZE);
      if (dbg_status == EEPROM_OK) { DBG_CheckRegion(dbg_all); }
      break;

    case DBG_CMD_WRITE_TEST:
      dbg_status = eeprom_write(&ee, TEST_START_ADDR, ee_tx, TEST_LEN);
      break;

    case DBG_CMD_ERASE_ALL:
      memset(dbg_all, 0xFF, sizeof(dbg_all));
      dbg_status = eeprom_write(&ee, 0, dbg_all, EEPROM_SIZE);
      break;

    default:
      break;                     /* unknown command: just clear it */
  }

  dbg_cmd_done_count++;
  dbg_cmd = DBG_CMD_NONE;        /* tells you "done" */
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
  EEPROM_Boot();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    DBG_Service();
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

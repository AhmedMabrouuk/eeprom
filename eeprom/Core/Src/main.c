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

/* Commands: write a number into dbg_cmd (Live Expressions). The loop runs it
 * and sets dbg_cmd back to 0 when finished. */
typedef enum
{
  DBG_CMD_NONE        = 0,
  DBG_CMD_READ_BYTE   = 1,   /* EXTEPROM_Data_Direct_Read  1 byte  @ dbg_addr      */
  DBG_CMD_READ_PAGE   = 2,   /* EXTEPROM_Data_Direct_Read  8 bytes @ dbg_page_no*8 */
  DBG_CMD_READ_ALL    = 3,   /* EXTEPROM_Data_Direct_Read  256 bytes               */
  DBG_CMD_WRITE_TEST  = 4,   /* EXTEPROM_Data_Direct_Write the 30..50 pattern      */
  DBG_CMD_WRITE_BYTE  = 5,   /* EXTEPROM_Data_Direct_Write dbg_wr_value @ dbg_wr_addr */
  DBG_CMD_WRITE_BLOCK = 6,   /* EXTEPROM_Data_Direct_Write dbg_wr_buf[dbg_wr_len] @ dbg_wr_addr */
  DBG_CMD_ERASE_BYTE  = 7,   /* EXTEPROM_Data_Byte_Erase   @ dbg_er_addr           */
  DBG_CMD_ERASE_BLOCK = 8,   /* EXTEPROM_flashData_erase   block dbg_er_block      */
  DBG_CMD_ERASE_ALL   = 9,   /* EXTEPROM_flashData_eraseAll                        */
  DBG_CMD_REINIT      = 10,  /* EXTEPROM_init(dbg_clk_mhz) + refresh capabilities  */
  DBG_CMD_SELFTEST    = 11   /* automatic test of ALL APIs (erases the chip!)      */
} dbg_cmd_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_CLK_MHZ      16u                                  /* I2C kernel clock */
#define TEST_START_ADDR     30u
#define TEST_END_ADDR       50u                                  /* inclusive        */
#define TEST_LEN            (TEST_END_ADDR - TEST_START_ADDR + 1u) /* = 21           */
#define DBG_WR_BUF_SIZE     32u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* ======================= ALL GLOBAL -> Live Expressions ==================== */

/* ---- per-API result: status returned by EXTEPROM_Status_Get() right after
 *      that API was called (0 = EXTEPROM_OK) ---- */
volatile EXTEPROM_Status_t st_init        = EXTEPROM_ERR_NOT_INIT; /* EXTEPROM_init            */
volatile EXTEPROM_Status_t st_write       = EXTEPROM_OK;           /* EXTEPROM_Data_Direct_Write */
volatile EXTEPROM_Status_t st_read        = EXTEPROM_OK;           /* EXTEPROM_Data_Direct_Read  */
volatile EXTEPROM_Status_t st_erase_block = EXTEPROM_OK;           /* EXTEPROM_flashData_erase   */
volatile EXTEPROM_Status_t st_erase_byte  = EXTEPROM_OK;           /* EXTEPROM_Data_Byte_Erase   */
volatile EXTEPROM_Status_t st_erase_all   = EXTEPROM_OK;           /* EXTEPROM_flashData_eraseAll*/

/* ---- per-API verdict: 1 = the API did what it should, verified by reading
 *      the chip back (not just "no error") ---- */
uint8_t ok_write        = 0;
uint8_t ok_read         = 0;
uint8_t ok_erase_block  = 0;
uint8_t ok_erase_byte   = 0;
uint8_t ok_erase_all    = 0;

/* ---- test pattern (written to EEPROM addresses 30..50) ---- */
uint8_t ee_tx[TEST_LEN];                 /* ee_tx[i] = 30 + i                    */
uint8_t ee_rx[TEST_LEN];                 /* read back from addresses 30..50      */
uint8_t dbg_verify_ok       = 0;         /* 1 = every address 30..50 holds its own number */
uint16_t dbg_mismatch_count = 0;

/* ---- command interface (you WRITE these) ---- */
volatile uint8_t  dbg_cmd      = DBG_CMD_NONE;
volatile uint16_t dbg_addr     = 33;     /* READ_BYTE address                    */
volatile uint8_t  dbg_page_no  = 0;      /* READ_PAGE: 0..31                     */
volatile uint16_t dbg_wr_addr  = 0;      /* WRITE_BYTE / WRITE_BLOCK address     */
volatile uint8_t  dbg_wr_value = 0;      /* WRITE_BYTE value                     */
volatile uint8_t  dbg_wr_len   = 0;      /* WRITE_BLOCK length (1..32)           */
uint8_t           dbg_wr_buf[DBG_WR_BUF_SIZE]; /* WRITE_BLOCK data               */
volatile uint16_t dbg_er_addr  = 0;      /* ERASE_BYTE address                   */
volatile uint16_t dbg_er_block = 0;      /* ERASE_BLOCK number: 0..31            */
volatile uint8_t  dbg_clk_mhz  = EEPROM_CLK_MHZ; /* REINIT clock (try 100 -> ERR_CLOCK) */

/* ---- results of the read / write / erase commands ---- */
uint8_t  dbg_byte = 0;                           /* READ_BYTE result             */
uint8_t  dbg_page[EXTEPROM_PAGE_SIZE_BYTES];     /* READ_PAGE / ERASE_BLOCK read-back */
uint8_t  dbg_all[EXTEPROM_SIZE_BYTES];           /* READ_ALL / ERASE_ALL read-back    */
uint8_t  dbg_wr_rb[DBG_WR_BUF_SIZE];             /* read-back after WRITE_*           */
uint8_t  dbg_er_rb = 0;                          /* read-back after ERASE_BYTE (expect 255) */

/* ---- bookkeeping ---- */
volatile EXTEPROM_Status_t dbg_status = EXTEPROM_OK; /* status of the LAST API call   */
uint32_t dbg_last_op_us     = 0;     /* measured duration of the last API call (us)   */
uint32_t dbg_cmd_done_count = 0;     /* +1 after every finished command               */
EXTEPROM_Capabilities_t ee_caps;     /* sizes, limits, timing (from EXTEPROM_Capabilities_Get) */

/* ---- self-test result (command 11) ---- */
uint8_t selftest_pass      = 0;      /* 1 = every API passed                          */
uint8_t selftest_fail_step = 0;      /* 0 = none; 1 init, 2 eraseAll, 3 write, 4 read,
                                        5 byte erase, 6 block erase, 7 range checks   */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void    Tmr_Start(void);
static void    Tmr_Stop(void);
static uint8_t All_Equal(const uint8_t *p, uint16_t n, uint8_t v);
static void    Check_Region(const uint8_t *img);
static void    EEPROM_Boot(void);
static uint8_t SelfTest(void);
static void    DBG_Service(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---- microsecond stopwatch (DWT cycle counter; falls back to 1 ms tick) ---- */
static uint32_t t_start;

#if defined(DWT)
static void Tmr_Start(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
  t_start = DWT->CYCCNT;
}
static void Tmr_Stop(void)
{
  dbg_last_op_us = (DWT->CYCCNT - t_start) / (SystemCoreClock / 1000000u);
}
#else
static void Tmr_Start(void) { t_start = HAL_GetTick(); }
static void Tmr_Stop(void)  { dbg_last_op_us = (HAL_GetTick() - t_start) * 1000u; }
#endif

static uint8_t All_Equal(const uint8_t *p, uint16_t n, uint8_t v)
{
  for (uint16_t i = 0; i < n; i++)
  {
    if (p[i] != v) { return 0u; }
  }
  return 1u;
}

/* Rule under test: in a 256-byte image, address N holds value N for N = 30..50. */
static void Check_Region(const uint8_t *img)
{
  uint16_t bad = 0;

  for (uint16_t a = TEST_START_ADDR; a <= TEST_END_ADDR; a++)
  {
    if (img[a] != (uint8_t)a) { bad++; }
  }
  dbg_mismatch_count = bad;
  dbg_verify_ok      = (bad == 0u) ? 1u : 0u;
}

/* Runs once at boot: init, store the 30..50 pattern, read it back, verify. */
static void EEPROM_Boot(void)
{
  for (uint16_t i = 0; i < TEST_LEN; i++)
  {
    ee_tx[i] = (uint8_t)(TEST_START_ADDR + i);
  }

  EXTEPROM_init(EEPROM_CLK_MHZ);
  st_init    = EXTEPROM_Status_Get();
  dbg_status = st_init;
  EXTEPROM_Capabilities_Get(&ee_caps);
  if (st_init != EXTEPROM_OK) { return; }

  Tmr_Start();
  EXTEPROM_Data_Direct_Write(TEST_START_ADDR, ee_tx, TEST_LEN);
  Tmr_Stop();
  st_write = EXTEPROM_Status_Get();
  if (st_write != EXTEPROM_OK) { dbg_status = st_write; return; }

  EXTEPROM_Data_Direct_Read(TEST_START_ADDR, ee_rx, TEST_LEN);
  st_read    = EXTEPROM_Status_Get();
  dbg_status = st_read;
  if (st_read != EXTEPROM_OK) { return; }
  ok_write = (memcmp(ee_tx, ee_rx, TEST_LEN) == 0) ? 1u : 0u;

  EXTEPROM_Data_Direct_Read(0, dbg_all, EXTEPROM_SIZE_BYTES);
  st_read    = EXTEPROM_Status_Get();
  dbg_status = st_read;
  if (st_read == EXTEPROM_OK)
  {
    Check_Region(dbg_all);
    ok_read = dbg_verify_ok;
  }
}

/* Automatic test of every API. Returns 0 if all passed, else the failing step.
 * WARNING: step 2 erases the whole chip. */
static uint8_t SelfTest(void)
{
  uint8_t b = 0;
  uint8_t pg[EXTEPROM_PAGE_SIZE_BYTES];

  ok_write = ok_read = ok_erase_block = ok_erase_byte = ok_erase_all = 0;

  /* 1. init */
  if (st_init != EXTEPROM_OK) { return 1; }

  /* 2. eraseAll: whole chip must read back 0xFF */
  EXTEPROM_flashData_eraseAll();
  st_erase_all = EXTEPROM_Status_Get();
  if (st_erase_all != EXTEPROM_OK) { return 2; }
  EXTEPROM_Data_Direct_Read(0, dbg_all, EXTEPROM_SIZE_BYTES);
  if (EXTEPROM_Status_Get() != EXTEPROM_OK) { return 2; }
  if (!All_Equal(dbg_all, EXTEPROM_SIZE_BYTES, 0xFFu)) { return 2; }
  ok_erase_all = 1;

  /* 3. write: pattern 30..50 crosses 3 page edges (32, 40, 48) */
  EXTEPROM_Data_Direct_Write(TEST_START_ADDR, ee_tx, TEST_LEN);
  st_write = EXTEPROM_Status_Get();
  if (st_write != EXTEPROM_OK) { return 3; }
  EXTEPROM_Data_Direct_Read(TEST_START_ADDR, ee_rx, TEST_LEN);
  if (EXTEPROM_Status_Get() != EXTEPROM_OK) { return 3; }
  if (memcmp(ee_tx, ee_rx, TEST_LEN) != 0) { return 3; }
  ok_write = 1;

  /* 4. read: single byte, one page, and the whole chip.
   *    Everything outside 30..50 must still be 0xFF (proves no page wrap-around damage). */
  EXTEPROM_Data_Direct_Read(33, &b, 1);
  st_read = EXTEPROM_Status_Get();
  if ((st_read != EXTEPROM_OK) || (b != 33u)) { return 4; }

  EXTEPROM_Data_Direct_Read(32, pg, EXTEPROM_PAGE_SIZE_BYTES);
  if (EXTEPROM_Status_Get() != EXTEPROM_OK) { return 4; }
  for (uint8_t i = 0; i < EXTEPROM_PAGE_SIZE_BYTES; i++)
  {
    if (pg[i] != (uint8_t)(32u + i)) { return 4; }
  }

  EXTEPROM_Data_Direct_Read(0, dbg_all, EXTEPROM_SIZE_BYTES);
  if (EXTEPROM_Status_Get() != EXTEPROM_OK) { return 4; }
  Check_Region(dbg_all);
  if (!dbg_verify_ok) { return 4; }
  for (uint16_t a = 0; a < EXTEPROM_SIZE_BYTES; a++)
  {
    if (((a < TEST_START_ADDR) || (a > TEST_END_ADDR)) && (dbg_all[a] != 0xFFu)) { return 4; }
  }
  ok_read = 1;

  /* 5. byte erase: address 40 becomes 0xFF, neighbours 39 and 41 untouched */
  EXTEPROM_Data_Byte_Erase(40);
  st_erase_byte = EXTEPROM_Status_Get();
  if (st_erase_byte != EXTEPROM_OK) { return 5; }
  EXTEPROM_Data_Direct_Read(39, pg, 3);                 /* addresses 39, 40, 41 */
  if (EXTEPROM_Status_Get() != EXTEPROM_OK) { return 5; }
  if ((pg[0] != 39u) || (pg[1] != 0xFFu) || (pg[2] != 41u)) { return 5; }
  ok_erase_byte = 1;

  /* 6. block erase: block 4 = addresses 32..39 become 0xFF, 31 and 41 untouched */
  EXTEPROM_flashData_erase(4);
  st_erase_block = EXTEPROM_Status_Get();
  if (st_erase_block != EXTEPROM_OK) { return 6; }
  EXTEPROM_Data_Direct_Read(32, pg, EXTEPROM_PAGE_SIZE_BYTES);
  if (EXTEPROM_Status_Get() != EXTEPROM_OK) { return 6; }
  if (!All_Equal(pg, EXTEPROM_PAGE_SIZE_BYTES, 0xFFu)) { return 6; }
  EXTEPROM_Data_Direct_Read(31, &b, 1);
  if ((EXTEPROM_Status_Get() != EXTEPROM_OK) || (b != 31u)) { return 6; }
  EXTEPROM_Data_Direct_Read(41, &b, 1);
  if ((EXTEPROM_Status_Get() != EXTEPROM_OK) || (b != 41u)) { return 6; }
  ok_erase_block = 1;

  /* 7. error handling: out-of-range requests must be refused and touch nothing */
  EXTEPROM_Data_Direct_Read(250, dbg_all, 10);
  if (EXTEPROM_Status_Get() != EXTEPROM_ERR_RANGE) { return 7; }
  EXTEPROM_Data_Direct_Write(256, ee_tx, 1);
  if (EXTEPROM_Status_Get() != EXTEPROM_ERR_RANGE) { return 7; }
  EXTEPROM_Data_Byte_Erase(256);
  if (EXTEPROM_Status_Get() != EXTEPROM_ERR_RANGE) { return 7; }
  EXTEPROM_flashData_erase(EXTEPROM_BLOCK_COUNT);
  if (EXTEPROM_Status_Get() != EXTEPROM_ERR_RANGE) { return 7; }
  EXTEPROM_Data_Direct_Read(0, NULL, 1);
  if (EXTEPROM_Status_Get() != EXTEPROM_ERR_PARAM) { return 7; }

  /* leave the chip with the pattern restored */
  EXTEPROM_Data_Direct_Write(TEST_START_ADDR, ee_tx, TEST_LEN);
  st_write = EXTEPROM_Status_Get();
  return 0;
}

/* Polled from the main loop. */
static void DBG_Service(void)
{
  uint8_t cmd = dbg_cmd;

  if (cmd == DBG_CMD_NONE) { return; }

  switch (cmd)
  {
    case DBG_CMD_READ_BYTE:
      Tmr_Start();
      EXTEPROM_Data_Direct_Read(dbg_addr, &dbg_byte, 1);
      Tmr_Stop();
      st_read = EXTEPROM_Status_Get();  dbg_status = st_read;
      break;

    case DBG_CMD_READ_PAGE:
      Tmr_Start();
      EXTEPROM_Data_Direct_Read((uint16_t)(dbg_page_no * EXTEPROM_PAGE_SIZE_BYTES),
                                dbg_page, EXTEPROM_PAGE_SIZE_BYTES);
      Tmr_Stop();
      st_read = EXTEPROM_Status_Get();  dbg_status = st_read;
      break;

    case DBG_CMD_READ_ALL:
      Tmr_Start();
      EXTEPROM_Data_Direct_Read(0, dbg_all, EXTEPROM_SIZE_BYTES);
      Tmr_Stop();
      st_read = EXTEPROM_Status_Get();  dbg_status = st_read;
      if (st_read == EXTEPROM_OK) { Check_Region(dbg_all); }
      break;

    case DBG_CMD_WRITE_TEST:
      Tmr_Start();
      EXTEPROM_Data_Direct_Write(TEST_START_ADDR, ee_tx, TEST_LEN);
      Tmr_Stop();
      st_write = EXTEPROM_Status_Get();  dbg_status = st_write;
      ok_write = 0;
      if (st_write == EXTEPROM_OK)
      {
        EXTEPROM_Data_Direct_Read(TEST_START_ADDR, ee_rx, TEST_LEN);
        ok_write = ((EXTEPROM_Status_Get() == EXTEPROM_OK) &&
                    (memcmp(ee_tx, ee_rx, TEST_LEN) == 0)) ? 1u : 0u;
      }
      break;

    case DBG_CMD_WRITE_BYTE:
    {
      uint16_t a = dbg_wr_addr;
      uint8_t  v = dbg_wr_value;

      Tmr_Start();
      EXTEPROM_Data_Direct_Write(a, &v, 1);
      Tmr_Stop();
      st_write = EXTEPROM_Status_Get();  dbg_status = st_write;
      ok_write = 0;
      if (st_write == EXTEPROM_OK)
      {
        EXTEPROM_Data_Direct_Read(a, dbg_wr_rb, 1);
        ok_write = ((EXTEPROM_Status_Get() == EXTEPROM_OK) && (dbg_wr_rb[0] == v)) ? 1u : 0u;
      }
      break;
    }

    case DBG_CMD_WRITE_BLOCK:
    {
      uint16_t a = dbg_wr_addr;
      uint8_t  n = dbg_wr_len;

      ok_write = 0;
      if ((n == 0u) || (n > DBG_WR_BUF_SIZE)) { dbg_status = EXTEPROM_ERR_PARAM; break; }

      Tmr_Start();
      EXTEPROM_Data_Direct_Write(a, dbg_wr_buf, n);
      Tmr_Stop();
      st_write = EXTEPROM_Status_Get();  dbg_status = st_write;
      if (st_write == EXTEPROM_OK)
      {
        memset(dbg_wr_rb, 0, sizeof(dbg_wr_rb));
        EXTEPROM_Data_Direct_Read(a, dbg_wr_rb, n);
        ok_write = ((EXTEPROM_Status_Get() == EXTEPROM_OK) &&
                    (memcmp(dbg_wr_buf, dbg_wr_rb, n) == 0)) ? 1u : 0u;
      }
      break;
    }

    case DBG_CMD_ERASE_BYTE:
    {
      uint16_t a = dbg_er_addr;

      Tmr_Start();
      EXTEPROM_Data_Byte_Erase(a);
      Tmr_Stop();
      st_erase_byte = EXTEPROM_Status_Get();  dbg_status = st_erase_byte;
      ok_erase_byte = 0;
      if (st_erase_byte == EXTEPROM_OK)
      {
        EXTEPROM_Data_Direct_Read(a, &dbg_er_rb, 1);
        ok_erase_byte = ((EXTEPROM_Status_Get() == EXTEPROM_OK) && (dbg_er_rb == 0xFFu)) ? 1u : 0u;
      }
      break;
    }

    case DBG_CMD_ERASE_BLOCK:
    {
      uint16_t blk = dbg_er_block;

      Tmr_Start();
      EXTEPROM_flashData_erase(blk);
      Tmr_Stop();
      st_erase_block = EXTEPROM_Status_Get();  dbg_status = st_erase_block;
      ok_erase_block = 0;
      if (st_erase_block == EXTEPROM_OK)
      {
        EXTEPROM_Data_Direct_Read((uint16_t)(blk * EXTEPROM_BLOCK_SIZE_BYTES),
                                  dbg_page, EXTEPROM_BLOCK_SIZE_BYTES);
        ok_erase_block = ((EXTEPROM_Status_Get() == EXTEPROM_OK) &&
                          All_Equal(dbg_page, EXTEPROM_BLOCK_SIZE_BYTES, 0xFFu)) ? 1u : 0u;
      }
      break;
    }

    case DBG_CMD_ERASE_ALL:
      Tmr_Start();
      EXTEPROM_flashData_eraseAll();
      Tmr_Stop();
      st_erase_all = EXTEPROM_Status_Get();  dbg_status = st_erase_all;
      ok_erase_all = 0;
      if (st_erase_all == EXTEPROM_OK)
      {
        EXTEPROM_Data_Direct_Read(0, dbg_all, EXTEPROM_SIZE_BYTES);
        ok_erase_all = ((EXTEPROM_Status_Get() == EXTEPROM_OK) &&
                        All_Equal(dbg_all, EXTEPROM_SIZE_BYTES, 0xFFu)) ? 1u : 0u;
        Check_Region(dbg_all);          /* now dbg_verify_ok = 0, as expected */
      }
      break;

    case DBG_CMD_REINIT:
      Tmr_Start();
      EXTEPROM_init(dbg_clk_mhz);
      Tmr_Stop();
      st_init = EXTEPROM_Status_Get();  dbg_status = st_init;
      EXTEPROM_Capabilities_Get(&ee_caps);
      break;

    case DBG_CMD_SELFTEST:
      selftest_fail_step = SelfTest();
      selftest_pass      = (selftest_fail_step == 0u) ? 1u : 0u;
      dbg_status         = EXTEPROM_Status_Get();
      break;

    default:
      break;                            /* unknown command: just clear it */
  }

  dbg_cmd_done_count++;
  dbg_cmd = DBG_CMD_NONE;               /* tells you "done" */
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

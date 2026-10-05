/**
 * @file    exteprom.c
 * @brief   External I2C EEPROM driver (24AA02 / 24LC02B), STM32 HAL backend.
 *          See exteprom.h for the API and exteprom_cfg.h for the settings.
 */
#include "exteprom.h"
#include <string.h>

extern I2C_HandleTypeDef EXTEPROM_I2C_HANDLE;

/* ---- settings with safe fall-backs if the cfg file does not define them --- */
#ifndef EXTEPROM_I2C_CLK_DIV
#define EXTEPROM_I2C_CLK_DIV        1u
#endif
#ifndef EXTEPROM_CHECK_BUS_SPEED
#define EXTEPROM_CHECK_BUS_SPEED    1u
#endif

/* ---- compile-time sanity checks (negative array size = build error) ------ */
#define EXTEPROM_STATIC_ASSERT(cond, name)  typedef char name[(cond) ? 1 : -1]
EXTEPROM_STATIC_ASSERT(EXTEPROM_SIZE_BYTES <= 256u,                                  exteprom_chk_1addr_byte);
EXTEPROM_STATIC_ASSERT((EXTEPROM_SIZE_BYTES % EXTEPROM_PAGE_SIZE_BYTES) == 0u,       exteprom_chk_pages);
EXTEPROM_STATIC_ASSERT((EXTEPROM_BLOCK_SIZE_BYTES % EXTEPROM_PAGE_SIZE_BYTES) == 0u, exteprom_chk_block);
EXTEPROM_STATIC_ASSERT((EXTEPROM_SIZE_BYTES % EXTEPROM_BLOCK_SIZE_BYTES) == 0u,      exteprom_chk_blocks);

/* ---- optional hooks (defaults do nothing) -------------------------------- */
#ifndef EXTEPROM_LOCK
#define EXTEPROM_LOCK()     ((void)0)
#endif
#ifndef EXTEPROM_UNLOCK
#define EXTEPROM_UNLOCK()   ((void)0)
#endif

#if defined(EXTEPROM_WP_PORT) && defined(EXTEPROM_WP_PIN)
#define EXTEPROM_WP_UNPROTECT()  HAL_GPIO_WritePin(EXTEPROM_WP_PORT, EXTEPROM_WP_PIN, GPIO_PIN_RESET)
#define EXTEPROM_WP_PROTECT()    HAL_GPIO_WritePin(EXTEPROM_WP_PORT, EXTEPROM_WP_PIN, GPIO_PIN_SET)
#else
#define EXTEPROM_WP_UNPROTECT()  ((void)0)
#define EXTEPROM_WP_PROTECT()    ((void)0)
#endif

/* ---- constants ----------------------------------------------------------- */
#define EXTEPROM_DEV_ADDR_HAL        ((uint16_t)(EXTEPROM_I2C_ADDR_7BIT << 1))

/* Bus clocks (SCL pulses) per transaction, from the datasheet frames:
 *   read  : S + ctrl(W) + addr + Sr + ctrl(R) + N data + P  ~ 30 + 9*N
 *   write : S + ctrl(W) + addr + N data + P                 ~ 21 + 9*N
 * (9 clocks per byte = 8 bits + ACK; ~3 clocks for START/STOP overhead)      */
#define EXTEPROM_CLK_READ_BASE       30u
#define EXTEPROM_CLK_WRITE_BASE      21u
#define EXTEPROM_CLK_PER_BYTE        9u

/* Timeouts are never computed from a bus faster than 100 kHz, so an
 * over-optimistic clock estimate can not cause false timeouts. */
#define EXTEPROM_MIN_TIMEOUT_PERIOD_NS  10000u

/* ---- module state -------------------------------------------------------- */
static EXTEPROM_Status_t s_status            = EXTEPROM_ERR_NOT_INIT;
static uint8_t           s_ready             = 0u;
static uint32_t          s_timeout_period_ns = EXTEPROM_MIN_TIMEOUT_PERIOD_NS;

/* ========================================================================== *
 * Internal helpers
 * ========================================================================== */

/* Estimate the SCL period from the I2C TIMINGR register:
 *   period = (SCLH+1 + SCLL+1) * (PRESC+1) / f_i2c_clk
 * Ignores rise/fall/sync time, so the real bus is ~5-10 % slower.
 * Returns 0 if the peripheral looks uninitialised (TIMINGR == 0). */
static uint32_t exteprom_scl_period_ns(uint32_t i2c_clk_mhz)
{
#if defined(I2C_TIMINGR_SCLL)
    uint32_t t = EXTEPROM_I2C_HANDLE.Instance->TIMINGR;

    if (t == 0u)
    {
        return 0u;
    }
    return ((((t >> 8) & 0xFFu) + 1u) + ((t & 0xFFu) + 1u)) *
           (((t >> 28) & 0x0Fu) + 1u) * 1000u / i2c_clk_mhz;
#else
    (void)i2c_clk_mhz;
    return 10000u;                    /* unknown peripheral: assume 100 kHz */
#endif
}

/* Timeout for one HAL transfer: 3x the expected time + 3 ms. */
static uint32_t exteprom_timeout_ms(uint32_t bus_clocks)
{
    uint64_t us = ((uint64_t)bus_clocks * s_timeout_period_ns) / 1000u;

    return 3u + (uint32_t)((3u * us) / 1000u);
}

static EXTEPROM_Status_t exteprom_map(HAL_StatusTypeDef hs)
{
    if (hs == HAL_OK)      { return EXTEPROM_OK; }
    if (hs == HAL_TIMEOUT) { return EXTEPROM_ERR_TIMEOUT; }
    return EXTEPROM_ERR_BUS;
}

/* Common argument check for read / write. */
static EXTEPROM_Status_t exteprom_check(uint16_t addr, const void *buf, uint16_t len)
{
    if (s_ready == 0u)                                { return EXTEPROM_ERR_NOT_INIT; }
    if (len == 0u)                                    { return EXTEPROM_OK; }
    if (buf == NULL)                                  { return EXTEPROM_ERR_PARAM; }
    if (((uint32_t)addr + len) > EXTEPROM_SIZE_BYTES) { return EXTEPROM_ERR_RANGE; }
    return EXTEPROM_OK;
}

/* ACK polling: the chip does not answer its address while it is busy storing
 * the data. Knock (START + address) until it answers or we give up. */
static EXTEPROM_Status_t exteprom_wait_ready(void)
{
    uint32_t t0 = HAL_GetTick();

    do
    {
        if (HAL_I2C_IsDeviceReady(&EXTEPROM_I2C_HANDLE, EXTEPROM_DEV_ADDR_HAL, 1u, 2u) == HAL_OK)
        {
            return EXTEPROM_OK;
        }
    } while ((HAL_GetTick() - t0) < (2u * EXTEPROM_WRITE_CYCLE_MAX_MS));

    return EXTEPROM_ERR_TIMEOUT;
}

/* One physical write: n bytes, guaranteed NOT to cross a page boundary. */
static EXTEPROM_Status_t exteprom_write_chunk(uint16_t addr, const uint8_t *data, uint16_t n)
{
    HAL_StatusTypeDef hs = HAL_I2C_Mem_Write(&EXTEPROM_I2C_HANDLE, EXTEPROM_DEV_ADDR_HAL,
                                             addr, I2C_MEMADD_SIZE_8BIT,
                                             (uint8_t *)data, n,
                                             exteprom_timeout_ms(EXTEPROM_CLK_WRITE_BASE +
                                                                 EXTEPROM_CLK_PER_BYTE * n));
    if (hs != HAL_OK)
    {
        return exteprom_map(hs);
    }
    return exteprom_wait_ready();
}

/* Write len bytes at addr, split on page boundaries.
 * data == NULL means "write 0xFF" (this is how erase is implemented).
 * Arguments must already be validated. */
static EXTEPROM_Status_t exteprom_write_range(uint16_t addr, const uint8_t *data, uint16_t len)
{
    uint8_t           blank[EXTEPROM_PAGE_SIZE_BYTES];
    EXTEPROM_Status_t st = EXTEPROM_OK;

    if (data == NULL)
    {
        memset(blank, EXTEPROM_ERASED_VALUE, sizeof(blank));
    }

    EXTEPROM_WP_UNPROTECT();

    while (len > 0u)
    {
        uint16_t room = (uint16_t)(EXTEPROM_PAGE_SIZE_BYTES - (addr % EXTEPROM_PAGE_SIZE_BYTES));
        uint16_t n    = (len < room) ? len : room;

        st = exteprom_write_chunk(addr, (data != NULL) ? data : blank, n);
        if (st != EXTEPROM_OK)
        {
            break;
        }

        addr = (uint16_t)(addr + n);
        len  = (uint16_t)(len - n);
        if (data != NULL)
        {
            data += n;
        }
    }

    EXTEPROM_WP_PROTECT();      /* always re-protect, also after an error */
    return st;
}

static EXTEPROM_Status_t exteprom_init_internal(uint8_t sys_clk_mhz)
{
    uint32_t i2c_clk_mhz;
    uint32_t period_ns;

    if (sys_clk_mhz == 0u)                          { return EXTEPROM_ERR_PARAM; }
    if (EXTEPROM_I2C_HANDLE.Instance == NULL)       { return EXTEPROM_ERR_BUS;   }

    i2c_clk_mhz = (uint32_t)sys_clk_mhz / EXTEPROM_I2C_CLK_DIV;
    if (i2c_clk_mhz == 0u)                          { return EXTEPROM_ERR_PARAM; }

    period_ns = exteprom_scl_period_ns(i2c_clk_mhz);
    if (period_ns == 0u)                            { return EXTEPROM_ERR_BUS;   }  /* I2C not initialised */

    s_timeout_period_ns = (period_ns > EXTEPROM_MIN_TIMEOUT_PERIOD_NS) ? period_ns
                                                                       : EXTEPROM_MIN_TIMEOUT_PERIOD_NS;

#if (EXTEPROM_CHECK_BUS_SPEED != 0u)
    /* 15 % tolerance because the estimate ignores rise/fall/sync time */
    if ((1000000u / period_ns) > ((EXTEPROM_MAX_BUS_KHZ * 115u) / 100u))
    {
        return EXTEPROM_ERR_CLOCK;
    }
#endif

    if (HAL_I2C_IsDeviceReady(&EXTEPROM_I2C_HANDLE, EXTEPROM_DEV_ADDR_HAL, 3u, 10u) != HAL_OK)
    {
        return EXTEPROM_ERR_BUS;
    }
    return EXTEPROM_OK;
}

/* ========================================================================== *
 * Public API
 * ========================================================================== */

void EXTEPROM_init(uint8_t ClockIn_MHZ)
{
    EXTEPROM_LOCK();
    s_ready  = 0u;
    s_status = exteprom_init_internal(ClockIn_MHZ);
    if (s_status == EXTEPROM_OK)
    {
        s_ready = 1u;
    }
    EXTEPROM_UNLOCK();
}

void EXTEPROM_flashData_erase(uint16_t blockNumber)
{
    EXTEPROM_LOCK();
    if (s_ready == 0u)
    {
        s_status = EXTEPROM_ERR_NOT_INIT;
    }
    else if (blockNumber >= EXTEPROM_BLOCK_COUNT)
    {
        s_status = EXTEPROM_ERR_RANGE;
    }
    else
    {
        s_status = exteprom_write_range((uint16_t)(blockNumber * EXTEPROM_BLOCK_SIZE_BYTES),
                                        NULL, EXTEPROM_BLOCK_SIZE_BYTES);
    }
    EXTEPROM_UNLOCK();
}

void EXTEPROM_Data_Byte_Erase(uint16_t Block_Address)
{
    EXTEPROM_LOCK();
    if (s_ready == 0u)
    {
        s_status = EXTEPROM_ERR_NOT_INIT;
    }
    else if (Block_Address >= EXTEPROM_SIZE_BYTES)
    {
        s_status = EXTEPROM_ERR_RANGE;
    }
    else
    {
        s_status = exteprom_write_range(Block_Address, NULL, 1u);
    }
    EXTEPROM_UNLOCK();
}

void EXTEPROM_Data_Direct_Write(uint16_t Block_Address, uint8_t *Dtat_Buffer, uint16_t Bytes_Num)
{
    EXTEPROM_LOCK();
    s_status = exteprom_check(Block_Address, Dtat_Buffer, Bytes_Num);
    if ((s_status == EXTEPROM_OK) && (Bytes_Num > 0u))
    {
        /* Dtat_Buffer is guaranteed non-NULL here, so this is a real write */
        s_status = exteprom_write_range(Block_Address, Dtat_Buffer, Bytes_Num);
    }
    EXTEPROM_UNLOCK();
}

void EXTEPROM_Data_Direct_Read(uint16_t Block_Address, uint8_t *Dtat_Buffer, uint16_t Bytes_Num )
{
    EXTEPROM_LOCK();
    s_status = exteprom_check(Block_Address, Dtat_Buffer, Bytes_Num);
    if ((s_status == EXTEPROM_OK) && (Bytes_Num > 0u))
    {
        /* S ctrl(W) addr  Sr ctrl(R)  D0 ACK ... Dn NACK P  -> one transaction */
        HAL_StatusTypeDef hs = HAL_I2C_Mem_Read(&EXTEPROM_I2C_HANDLE, EXTEPROM_DEV_ADDR_HAL,
                                                Block_Address, I2C_MEMADD_SIZE_8BIT,
                                                Dtat_Buffer, Bytes_Num,
                                                exteprom_timeout_ms(EXTEPROM_CLK_READ_BASE +
                                                                    EXTEPROM_CLK_PER_BYTE * Bytes_Num));
        s_status = exteprom_map(hs);
    }
    EXTEPROM_UNLOCK();
}

EXTEPROM_Status_t EXTEPROM_Status_Get(void)
{
    return s_status;
}

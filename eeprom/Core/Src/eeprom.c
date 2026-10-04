/**
 * @file    eeprom.c
 * @brief   24AA02 / 24LC02B driver, STM32 HAL backend. See eeprom.h.
 */
#include "eeprom.h"
#include <string.h>

/* ---- internal helpers --------------------------------------------------- */

static void ee_wp(const eeprom_t *e, GPIO_PinState state)
{
    if (e->wp_port != NULL)
    {
        HAL_GPIO_WritePin(e->wp_port, e->wp_pin, state);
    }
}

static eeprom_status_t ee_check_args(const eeprom_t *e, uint16_t addr,
                                     const void *buf, uint16_t len)
{
    if (e == NULL || e->hi2c == NULL || buf == NULL)
    {
        return EEPROM_ERR_PARAM;
    }
    if ((uint32_t)addr + len > EEPROM_SIZE)
    {
        return EEPROM_ERR_RANGE;
    }
    return EEPROM_OK;
}

/* ACK polling (datasheet section 5): the chip NACKs its address while the
 * internal write cycle runs. Send START + control byte until it ACKs. */
static eeprom_status_t ee_wait_ready(const eeprom_t *e)
{
    uint32_t t0 = HAL_GetTick();

    do
    {
        if (HAL_I2C_IsDeviceReady(e->hi2c, e->dev_addr, 1u, 2u) == HAL_OK)
        {
            return EEPROM_OK;
        }
    } while ((HAL_GetTick() - t0) < EEPROM_WRITE_TIMEOUT_MS);

    return EEPROM_ERR_TIMEOUT;
}

/* One page-write transaction (n must not cross a page boundary). */
static eeprom_status_t ee_write_chunk(const eeprom_t *e, uint16_t addr,
                                      const uint8_t *data, uint16_t n)
{
    if (HAL_I2C_Mem_Write(e->hi2c, e->dev_addr, addr, I2C_MEMADD_SIZE_8BIT,
                          (uint8_t *)data, n, EEPROM_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return EEPROM_ERR_BUS;
    }
    return ee_wait_ready(e);
}

/* Bytes left in the page that contains addr. */
static uint16_t ee_room_in_page(uint16_t addr)
{
    return (uint16_t)(EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE));
}

/* ---- public API --------------------------------------------------------- */

eeprom_status_t eeprom_init(eeprom_t *e, I2C_HandleTypeDef *hi2c)
{
    if (e == NULL || hi2c == NULL)
    {
        return EEPROM_ERR_PARAM;
    }

    e->hi2c     = hi2c;
    e->dev_addr = (uint16_t)(EEPROM_I2C_ADDR_7BIT << 1);
    e->wp_port  = NULL;
    e->wp_pin   = 0u;

    if (HAL_I2C_IsDeviceReady(hi2c, e->dev_addr, 3u, 10u) != HAL_OK)
    {
        return EEPROM_ERR_BUS;
    }
    return EEPROM_OK;
}

eeprom_status_t eeprom_read(const eeprom_t *e, uint16_t addr,
                            uint8_t *buf, uint16_t len)
{
    eeprom_status_t st = ee_check_args(e, addr, buf, len);
    if (st != EEPROM_OK || len == 0u)
    {
        return st;
    }

    /* Random read + sequential read in one go: START, ctrl(W), addr,
     * RESTART, ctrl(R), data... NACK on the last byte, STOP. The chip
     * auto-increments its pointer, so the whole array can be read at once.
     * Timeout scales with len (~9 clocks per byte, worst case 100 kHz). */
    if (HAL_I2C_Mem_Read(e->hi2c, e->dev_addr, addr, I2C_MEMADD_SIZE_8BIT,
                         buf, len, EEPROM_I2C_TIMEOUT_MS + len) != HAL_OK)
    {
        return EEPROM_ERR_BUS;
    }
    return EEPROM_OK;
}

eeprom_status_t eeprom_write(const eeprom_t *e, uint16_t addr,
                             const uint8_t *buf, uint16_t len)
{
    eeprom_status_t st = ee_check_args(e, addr, buf, len);
    if (st != EEPROM_OK || len == 0u)
    {
        return st;
    }

    ee_wp(e, GPIO_PIN_RESET);                 /* unprotect */

    while (len > 0u)
    {
        uint16_t room = ee_room_in_page(addr);
        uint16_t n    = (len < room) ? len : room;

        st = ee_write_chunk(e, addr, buf, n);
        if (st != EEPROM_OK)
        {
            break;
        }
        addr += n;
        buf  += n;
        len  -= n;
    }

    ee_wp(e, GPIO_PIN_SET);                   /* protect again */
    return st;
}

eeprom_status_t eeprom_update(const eeprom_t *e, uint16_t addr,
                              const uint8_t *buf, uint16_t len)
{
    eeprom_status_t st = ee_check_args(e, addr, buf, len);
    uint8_t cur[EEPROM_PAGE_SIZE];

    if (st != EEPROM_OK || len == 0u)
    {
        return st;
    }

    ee_wp(e, GPIO_PIN_RESET);

    while (len > 0u)
    {
        uint16_t room = ee_room_in_page(addr);
        uint16_t n    = (len < room) ? len : room;

        st = eeprom_read(e, addr, cur, n);
        if (st != EEPROM_OK)
        {
            break;
        }
        if (memcmp(cur, buf, n) != 0)
        {
            st = ee_write_chunk(e, addr, buf, n);
            if (st != EEPROM_OK)
            {
                break;
            }
        }
        addr += n;
        buf  += n;
        len  -= n;
    }

    ee_wp(e, GPIO_PIN_SET);
    return st;
}

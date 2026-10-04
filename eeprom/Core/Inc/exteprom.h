/**
 * @file    exteprom.h
 * @brief   External I2C EEPROM component (Microchip 24AA02 / 24LC02B, 256 bytes)
 *          on top of the STM32 HAL I2C driver.
 *
 * Typical use:
 *      EXTEPROM_init(16);                              // I2C kernel clock = 16 MHz
 *      EXTEPROM_Data_Direct_Write(100, buf, 4);        // save 4 bytes at address 100
 *      EXTEPROM_Data_Direct_Read (100, rd,  4);        // read them back
 *      if (EXTEPROM_Status_Get() != EXTEPROM_OK) { ... }
 *
 * All requested API functions return void. The result of the LAST call is
 * available from EXTEPROM_Status_Get().
 *
 * The component is blocking (polling) and not re-entrant: do not call it from
 * an ISR. For RTOS use, define EXTEPROM_LOCK()/EXTEPROM_UNLOCK() in the cfg.
 */
#ifndef EXTEPROM_H
#define EXTEPROM_H

#include "main.h"          /* pulls in the correct stm32xxxx_hal.h */
#include <stdint.h>
#include "exteprom_cfg.h"

/* ------------------------------------------------------------------------ *
 * Capabilities known at compile time
 * ------------------------------------------------------------------------ */
#define EXTEPROM_PAGE_COUNT           (EXTEPROM_SIZE_BYTES / EXTEPROM_PAGE_SIZE_BYTES)
#define EXTEPROM_BLOCK_COUNT          (EXTEPROM_SIZE_BYTES / EXTEPROM_BLOCK_SIZE_BYTES)

#define EXTEPROM_MAX_READ_PER_CALL    EXTEPROM_SIZE_BYTES        /* one sequential read  */
#define EXTEPROM_MAX_WRITE_PER_CALL   EXTEPROM_SIZE_BYTES        /* driver splits pages  */
#define EXTEPROM_MAX_WRITE_PER_CYCLE  EXTEPROM_PAGE_SIZE_BYTES   /* physical limit       */

/* ------------------------------------------------------------------------ *
 * Types
 * ------------------------------------------------------------------------ */
typedef enum
{
    EXTEPROM_OK = 0,          /* last call succeeded                                   */
    EXTEPROM_ERR_NOT_INIT,    /* EXTEPROM_init() not called, or it failed              */
    EXTEPROM_ERR_PARAM,       /* NULL buffer / ClockIn_MHZ == 0                        */
    EXTEPROM_ERR_RANGE,       /* address or address+length is outside the chip         */
    EXTEPROM_ERR_BUS,         /* I2C error or chip does not answer (ACK missing)       */
    EXTEPROM_ERR_TIMEOUT,     /* transfer or internal write cycle took too long        */
    EXTEPROM_ERR_CLOCK        /* I2C bus is faster than the chip allows                */
} EXTEPROM_Status_t;

typedef struct
{
    /* --- memory organisation --- */
    uint16_t total_bytes;           /* 256                                              */
    uint16_t page_count;            /* 32                                               */
    uint8_t  page_bytes;            /* 8                                                */
    uint16_t block_bytes;           /* erase granularity of EXTEPROM_flashData_erase()  */
    uint16_t block_count;
    uint8_t  i2c_addr_7bit;         /* 0x50                                             */

    /* --- limits --- */
    uint16_t max_read_per_call;     /* bytes                                            */
    uint16_t max_write_per_call;    /* bytes (driver splits into page cycles)           */
    uint8_t  max_write_per_cycle;   /* bytes in ONE physical write cycle                */
    uint32_t endurance_cycles;      /* erase/write cycles per location                  */

    /* --- timing (microseconds). Bus-time parts are ESTIMATES from the I2C
     *     timing register; write times are WORST CASE (5 ms cycle per page).  --- */
    uint16_t bus_khz;               /* estimated SCL frequency                          */
    uint32_t write_cycle_max_us;    /* 5000                                             */
    uint32_t t_read_byte_us;        /* read 1 byte                                      */
    uint32_t t_read_page_us;        /* read 8 bytes                                     */
    uint32_t t_read_all_us;         /* read all 256 bytes                               */
    uint32_t t_write_byte_us;       /* write (or erase) 1 byte, incl. write cycle       */
    uint32_t t_write_page_us;       /* write (or erase) 1 aligned page                  */
    uint32_t t_write_all_us;        /* write (or erase) the whole chip                  */
} EXTEPROM_Capabilities_t;

/* ------------------------------------------------------------------------ *
 * API
 * ------------------------------------------------------------------------ */

/**
 * @brief  Initialise the component and check that the chip answers.
 * @param  ClockIn_MHZ  Kernel clock of the I2C peripheral in MHz (e.g. 16).
 *                      Used to work out the real SCL speed from the I2C timing
 *                      register, to size the timeouts, and to refuse a bus
 *                      that is too fast for the chip.
 * @note   The I2C peripheral itself must already be initialised (CubeMX /
 *         HAL_I2C_Init). Result: EXTEPROM_Status_Get().
 */
void EXTEPROM_init(uint8_t ClockIn_MHZ);

/**
 * @brief  Erase one block (EXTEPROM_BLOCK_SIZE_BYTES, default 8 bytes = 1 page)
 *         by writing 0xFF to it.
 * @param  blockNumber  0 .. EXTEPROM_BLOCK_COUNT-1 (block n starts at n*block size)
 */
void EXTEPROM_flashData_erase(uint16_t blockNumber);

/**
 * @brief  Write Bytes_Num bytes to the chip starting at Block_Address.
 *         Page boundaries and the internal write cycle are handled for you.
 * @note   Blocks until the data is stored (up to ~5 ms per page touched).
 */
void EXTEPROM_Data_Direct_Write(uint16_t Block_Address, uint8_t *Data_Buffer, uint16_t Bytes_Num);

/**
 * @brief  Read Bytes_Num bytes from the chip starting at Block_Address
 *         (one sequential-read transaction).
 */
void EXTEPROM_Data_Direct_Read(uint16_t Block_Address, uint8_t *Data_Buffer, uint16_t Bytes_Num);

/**
 * @brief  Erase ONE byte (writes 0xFF to that address).
 */
void EXTEPROM_Data_Byte_Erase(uint16_t Block_Address);

/**
 * @brief  Erase the whole chip (writes 0xFF everywhere). Blocks for up to
 *         ~190 ms at 100 kHz.
 */
void EXTEPROM_flashData_eraseAll(void);

/** @brief  Result of the most recent API call. */
EXTEPROM_Status_t EXTEPROM_Status_Get(void);

/** @brief  Fill a structure with the capabilities / timing of chip + driver. */
void EXTEPROM_Capabilities_Get(EXTEPROM_Capabilities_t *cap);

#endif /* EXTEPROM_H */

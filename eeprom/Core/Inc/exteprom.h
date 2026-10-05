/**
 * @file    exteprom.h
 * @brief   External I2C EEPROM driver (Microchip 24AA02 / 24LC02B, 256 bytes)
 *          on top of the STM32 HAL I2C driver.
 *
 * MANDATORY API (names and signatures are a contract, do not change):
 *      void EXTEPROM_init(uint8_t ClockIn_MHZ);
 *      void EXTEPROM_flashData_erase(uint16_t blockNumber);
 *      void EXTEPROM_Data_Direct_Write(uint16_t Block_Address, uint8_t *Dtat_Buffer, uint16_t Bytes_Num);
 *      void EXTEPROM_Data_Direct_Read(uint16_t Block_Address, uint8_t *Dtat_Buffer, uint16_t Bytes_Num );
 * ADDITIONS:
 *      void EXTEPROM_Data_Byte_Erase(uint16_t Block_Address);   erase a single byte
 *      EXTEPROM_Status_t EXTEPROM_Status_Get(void);             result of the last call
 *
 * Typical use:
 *      EXTEPROM_init(16);                              // system clock = 16 MHz
 *      EXTEPROM_Data_Direct_Write(100, buf, 4);        // save 4 bytes at address 100
 *      EXTEPROM_Data_Direct_Read (100, rd,  4);        // read them back
 *
 * CAPABILITIES (24LC02B)
 *      Size ................ 256 bytes = 32 pages x 8 bytes, 1 address byte (0..255)
 *      Max read per call ... 256 bytes (one sequential read)
 *      Max write per call .. 256 bytes (driver splits it on page edges)
 *      Max per write cycle . 8 bytes (one page, must not cross a page edge)
 *      Read time ........... 1 byte / 8 bytes / 256 bytes = 0.39 / 1.0 / 23 ms @100 kHz
 *                                                           0.10 / 0.26 / 5.8 ms @400 kHz
 *      Write / erase time .. bus time + internal write cycle (2 ms typ, 5 ms max) per
 *                            page touched: 1 byte or 1 page <= ~5.9 ms, 20 bytes at a
 *                            misaligned address <= ~23 ms, whole chip <= ~190 ms @100 kHz
 *      Endurance ........... 1,000,000 erase/write cycles per location
 *      Erase ............... the chip has no erase command: erase = write 0xFF
 *
 * The driver is blocking (polling) and not re-entrant: do not call it from an
 * ISR. For RTOS use, define EXTEPROM_LOCK()/EXTEPROM_UNLOCK() in the cfg file.
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

/* ------------------------------------------------------------------------ *
 * API
 * ------------------------------------------------------------------------ */

/**
 * @brief  Initialise the driver and check that the chip answers.
 * @param  ClockIn_MHZ  SYSTEM clock in MHz (e.g. 16). Together with the divider in
 *                      exteprom_cfg.h (EXTEPROM_I2C_CLK_DIV) it gives the I2C
 *                      peripheral clock, which is used to work out the real SCL
 *                      speed, to size the timeouts, and (optionally) to refuse a
 *                      bus that is too fast for the chip.
 * @note   The I2C peripheral must already be initialised (CubeMX / HAL_I2C_Init).
 */
void EXTEPROM_init(uint8_t ClockIn_MHZ);

/**
 * @brief  Erase one block (EXTEPROM_BLOCK_SIZE_BYTES, default 8 bytes = 1 page)
 *         by writing 0xFF to it. Block n starts at address n * block size.
 * @param  blockNumber  0 .. EXTEPROM_BLOCK_COUNT-1
 */
void EXTEPROM_flashData_erase(uint16_t blockNumber);

/**
 * @brief  Write Bytes_Num bytes to the chip starting at Block_Address.
 *         Page boundaries and the internal write cycle are handled for you.
 * @note   Blocks until the data is stored (up to ~5 ms per page touched).
 */
void EXTEPROM_Data_Direct_Write(uint16_t Block_Address, uint8_t *Dtat_Buffer, uint16_t Bytes_Num);

/**
 * @brief  Read Bytes_Num bytes from the chip starting at Block_Address
 *         (one sequential-read transaction).
 */
void EXTEPROM_Data_Direct_Read(uint16_t Block_Address, uint8_t *Dtat_Buffer, uint16_t Bytes_Num );

/**
 * @brief  Erase ONE byte (writes 0xFF to that address).
 */
void EXTEPROM_Data_Byte_Erase(uint16_t Block_Address);

/**
 * @brief  Result of the most recent API call (the APIs above return void).
 */
EXTEPROM_Status_t EXTEPROM_Status_Get(void);

#endif /* EXTEPROM_H */

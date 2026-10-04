/**
 * @file    eeprom.h
 * @brief   Driver for Microchip 24AA02 / 24LC02B (2 Kbit, 256 x 8) I2C EEPROM
 *          on top of the STM32 HAL I2C driver.
 *
 * Datasheet facts this driver is built around (DS21709C):
 *  - Control byte = 1010 xxx R/W. A0..A2 are NOT connected inside the chip,
 *    so the 7-bit address is always 0x50 (HAL wants it shifted: 0xA0).
 *  - 1 word-address byte (256 bytes total, 8-bit address is enough).
 *  - Page = 8 bytes. A write that crosses a page boundary WRAPS to the start
 *    of the same page and corrupts data -> the driver splits writes at page
 *    boundaries.
 *  - Write cycle <= 5 ms, during which the chip does not ACK -> ACK polling.
 *  - WP pin high = writes inhibited (reads still work).
 */
#ifndef EEPROM_H
#define EEPROM_H

#include "main.h"          /* pulls in the right stm32xxxx_hal.h */
#include <stdint.h>

#define EEPROM_I2C_ADDR_7BIT    0x50u
#define EEPROM_SIZE             256u
#define EEPROM_PAGE_SIZE        8u
#define EEPROM_WRITE_TIMEOUT_MS 10u   /* datasheet max is 5 ms, 2x margin */
#define EEPROM_I2C_TIMEOUT_MS   20u   /* base timeout per HAL transfer    */

typedef enum
{
    EEPROM_OK = 0,
    EEPROM_ERR_PARAM,     /* NULL pointer / zero handle           */
    EEPROM_ERR_RANGE,     /* addr + len beyond 256 bytes          */
    EEPROM_ERR_BUS,       /* I2C error or device not responding   */
    EEPROM_ERR_TIMEOUT    /* write cycle did not finish in time   */
} eeprom_status_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint16_t           dev_addr;   /* 8-bit form (7-bit << 1), as HAL expects */

    /* Optional WP pin (active high = protect). Leave port NULL if WP is
     * hard-wired. The driver releases WP only for the duration of a write. */
    GPIO_TypeDef      *wp_port;
    uint16_t           wp_pin;
} eeprom_t;

/** Bind to an I2C handle and check the chip answers. Set wp_port/wp_pin
 *  on the struct afterwards if you have a WP GPIO. */
eeprom_status_t eeprom_init(eeprom_t *e, I2C_HandleTypeDef *hi2c);

/** Read len bytes starting at addr (single sequential read). */
eeprom_status_t eeprom_read(const eeprom_t *e, uint16_t addr,
                            uint8_t *buf, uint16_t len);

/** Write len bytes starting at addr. Handles page boundaries and waits for
 *  each internal write cycle (ACK polling) before returning. */
eeprom_status_t eeprom_write(const eeprom_t *e, uint16_t addr,
                             const uint8_t *buf, uint16_t len);

/** Like eeprom_write but skips pages whose content already matches.
 *  Saves endurance (1M cycles) and time. Prefer this for config storage. */
eeprom_status_t eeprom_update(const eeprom_t *e, uint16_t addr,
                              const uint8_t *buf, uint16_t len);

#endif /* EEPROM_H */

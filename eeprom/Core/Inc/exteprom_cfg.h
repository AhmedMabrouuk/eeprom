/**
 * @file    exteprom_cfg.h
 * @brief   Per-project configuration for the EXTEPROM component.
 *
 * This is the ONLY file you should need to edit when you move the component
 * to another board or another chip of the same family.
 */
#ifndef EXTEPROM_CFG_H
#define EXTEPROM_CFG_H

/* ------------------------------------------------------------------------ *
 * 1. Wiring (edit per board)
 * ------------------------------------------------------------------------ */

/* Name of the HAL I2C handle the EEPROM is connected to (must be a global,
 * e.g. CubeMX generates "I2C_HandleTypeDef hi2c1;"). */
#define EXTEPROM_I2C_HANDLE          hi2c1

/* 7-bit I2C address. 24AA02 / 24LC02B ignore A0..A2, so it is always 0x50. */
#define EXTEPROM_I2C_ADDR_7BIT       0x50u

/* EXTEPROM_init(ClockIn_MHZ) receives the SYSTEM clock in MHz. The I2C
 * peripheral clock is  ClockIn_MHZ / EXTEPROM_I2C_CLK_DIV.
 * 1 = I2C runs at the system clock (AHB and APB1 prescalers = 1, I2C clock
 * source = PCLK1). Use 2, 4 ... if the bus prescalers divide the clock. */
#define EXTEPROM_I2C_CLK_DIV         1u

/* 1 = EXTEPROM_init() refuses (EXTEPROM_ERR_CLOCK) a bus clearly faster than
 * the chip allows. Set to 0 if the I2C clock source is not derived from the
 * system clock (e.g. I2C fed by HSI16 while the CPU runs from a PLL). */
#define EXTEPROM_CHECK_BUS_SPEED     1u

/* Optional Write-Protect pin driven by the MCU (WP high = chip is read-only).
 * Leave both lines commented out if WP is wired to GND (or VCC) on the board.
 * The pin must be configured as a push-pull output by your GPIO init code. */
/* #define EXTEPROM_WP_PORT          GPIOB      */
/* #define EXTEPROM_WP_PIN           GPIO_PIN_0 */

/* ------------------------------------------------------------------------ *
 * 2. Chip description (values from datasheet DS21709C, 24AA02 / 24LC02B)
 * ------------------------------------------------------------------------ */
#define EXTEPROM_SIZE_BYTES          256u    /* 2 Kbit = 256 x 8 bits          */
#define EXTEPROM_PAGE_SIZE_BYTES     8u      /* page write buffer              */

/* "Block" in the EXTEPROM_flashData_erase() API = this many bytes.
 * Default is one page (8 bytes). Must be a multiple of the page size. */
#define EXTEPROM_BLOCK_SIZE_BYTES    EXTEPROM_PAGE_SIZE_BYTES

#define EXTEPROM_ERASED_VALUE        0xFFu   /* an EEPROM "erase" writes 0xFF  */
#define EXTEPROM_WRITE_CYCLE_MAX_MS  5u      /* TWC max (datasheet table 1-2)  */
#define EXTEPROM_WRITE_CYCLE_TYP_MS  2u      /* typical page write cycle       */
#define EXTEPROM_MAX_BUS_KHZ         400u    /* 24LC02B at Vcc >= 2.5 V        */
#define EXTEPROM_ENDURANCE_CYCLES    1000000UL

/* ------------------------------------------------------------------------ *
 * 3. Optional RTOS protection (the driver itself is NOT re-entrant)
 * ------------------------------------------------------------------------ */
/* #define EXTEPROM_LOCK()           osMutexAcquire(eepromMutex, osWaitForever) */
/* #define EXTEPROM_UNLOCK()         osMutexRelease(eepromMutex)                */

#endif /* EXTEPROM_CFG_H */

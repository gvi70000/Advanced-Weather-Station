/***************************************************************************
 * @file [ENS160].h/.c
 * This file contains definitions, data structures, and functions
 * for interfacing with the ENS160 air quality sensor.
 *
 * Copyright (c) [2024] Grozea Ion gvi70000
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ***************************************************************************/
 
#ifndef ENS160_H
#define ENS160_H

#include "stm32f3xx_hal.h"

/** @brief ENS160 I2C Address */
#define ENS160_I2C_ADDRESS (0x52 << 1) /**< 7-bit I2C address of the ENS160 sensor */

/** @brief ENS160 PART ID */
#define ENS160_PART_ID 0x0160 /**< ENS160 part ID used for identification */

/** @brief ENS160 Register Addresses */
#define ENS160_REG_PART_ID		0x00 /**< Device identity register address */
#define ENS160_REG_OPMODE			0x10 /**< Operating mode register address */
#define ENS160_REG_CONFIG			0x11 /**< Interrupt pin configuration register address */
#define ENS160_REG_COMMAND		0x12 /**< System commands register address */
#define ENS160_REG_TEMP_IN		0x13 /**< Ambient temperature input register address */
#define ENS160_REG_RH_IN			0x15 /**< Relative humidity input register address */
#define ENS160_REG_STATUS			0x20 /**< Device status register address */
#define ENS160_REG_DATA_AQI		0x21 /**< Air quality index data register address */
#define ENS160_REG_DATA_TVOC	0x22 /**< Total volatile organic compounds data register address */
#define ENS160_REG_DATA_ECO2	0x24 /**< Equivalent CO2 concentration data register address */
#define ENS160_REG_DATA_T			0x30 /**< Calculated temperature data register address */
#define ENS160_REG_DATA_RH		0x32 /**< Calculated relative humidity data register address */
#define ENS160_REG_DATA_MISR	0x38 /**< Data integrity field register address */
#define ENS160_REG_GPR_WRITE	0x40 /**< General-purpose write registers address */
#define ENS160_REG_GPR_READ		0x48 /**< General-purpose read registers address */

/** @brief ENS160 Register Sizes */
#define ENS160_SIZE1 1 /**< 1-byte register size */
#define ENS160_SIZE2 2 /**< 2-byte register size */
#define ENS160_SIZE3 3 /**< 3-byte register size */
#define ENS160_SIZE5 5 /**< 5-byte register size */
#define ENS160_SIZE8 8 /**< 8-byte register size */

#define ENS160_CLEAR_REG_DELAY 2
/** @brief ENS160 Operating Modes */
typedef enum {
    ENS160_OPMODE_DEEP_SLEEP = 0x00, /**< Deep sleep mode (low-power standby) */
    ENS160_OPMODE_IDLE       = 0x01, /**< Idle mode (low power) */
    ENS160_OPMODE_STANDARD   = 0x02, /**< Standard gas sensing mode */
    ENS160_OPMODE_RESET      = 0xF0  /**< Reset mode */
} ENS160_OPMODE_t;

/** @brief ENS160 Configuration Register Fields */
typedef struct __attribute__((packed)) {
    uint8_t INTEN    : 1; /**< Interrupt enable bit */
    uint8_t INTDAT   : 1; /**< Interrupt for data registers */
    uint8_t RESERVED1: 1; /**< Reserved */
    uint8_t INTGPR   : 1; /**< Interrupt for GPR registers */
    uint8_t RESERVED2: 1; /**< Reserved */
    uint8_t INT_CFG  : 1; /**< INTn pin drive configuration */
    uint8_t INTPOL   : 1; /**< INTn pin polarity configuration */
    uint8_t RESERVED3: 1; /**< Reserved */
} ENS160_CONFIG_t;

/** @brief ENS160 Command Set */
typedef enum {
    ENS160_COMMAND_NOP       = 0x00, /**< No operation command */
    ENS160_COMMAND_GET_FWVER = 0x0E, /**< Get firmware version command */
    ENS160_COMMAND_CLRGPR    = 0xCC  /**< Clear GPR read registers command */
} ENS160_COMMAND_t;

/** @brief ENS160 Status Register Fields */
typedef struct __attribute__((packed)) {
    uint8_t NEWGPR   : 1; /**< New data in GPR read registers */
    uint8_t NEWDAT   : 1; /**< New data in data registers */
    uint8_t VALIDITY : 2; /**< Data validity status */
    uint8_t RESERVED : 2; /**< Reserved */
    uint8_t STATER   : 1; /**< Error detected status */
    uint8_t STATAS   : 1; /**< Operating mode running status */
} ENS160_STATUS_t;

/** @brief Structure to hold ENS160 air quality data */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Buffer[ENS160_SIZE5]; /**< Raw data buffer for direct I2C read */
        struct __attribute__((packed)) {
            uint8_t AQI;       /**< Air quality index */
            uint16_t TVOC;     /**< Total volatile organic compounds */
            uint16_t eCO2;     /**< Equivalent CO2 concentration */
        } Fields;
    };
} ENS160_Data_t;

/** @brief Structure to hold ENS160 firmware version information */
typedef struct __attribute__((packed)) {
		uint8_t Major;   /**< Major firmware version */
		uint8_t Minor;   /**< Minor firmware version */
		uint8_t Release; /**< Firmware release version */
} ENS160_FW_t;

/** @brief Structure to hold ENS160 resistance values */
typedef struct __attribute__((packed)) {
    uint32_t R1; /**< Resistance value for R1 in Ohms */
    uint32_t R2; /**< Resistance value for R2 in Ohms */
    uint32_t R3; /**< Resistance value for R3 in Ohms */
    uint32_t R4; /**< Resistance value for R4 in Ohms */
} ENS160_Resistance_t;

/** @brief ENS160 Register Map Structure */
typedef struct __attribute__((packed)) {
    ENS160_OPMODE_t OPMODE;  /**< Operating mode register 1 byte */
    ENS160_CONFIG_t CONFIG;  /**< Configuration register 1 byte */
    ENS160_COMMAND_t COMMAND;/**< Command register 1 byte */
    uint16_t TEMP_IN;        /**< Ambient temperature input register 2 bytes */
    uint16_t RH_IN;          /**< Ambient relative humidity input register 2 bytes */
    ENS160_STATUS_t STATUS;  /**< Device status register 1 byte */
	
    uint16_t DATA_T;         /**< Calculated temperature data 2 bytes */
    uint16_t DATA_RH;        /**< Calculated relative humidity data 2 bytes */
    uint8_t DATA_MISR;       /**< Data integrity field */
    uint8_t GPR_WRITE[ENS160_SIZE8]; /**< General-purpose write registers */
    uint8_t GPR_READ[ENS160_SIZE8];  /**< General-purpose read registers */
} ENS160_Registers_t;

/** @brief Initializes the ENS160 sensor.
 * @return HAL status
 */
HAL_StatusTypeDef ENS160_Init(void);

/** @brief Reads the part ID of the ENS160 sensor.
 * @return HAL status
 */
HAL_StatusTypeDef ENS160_ReadPartID(void);
HAL_StatusTypeDef ENS160_GetFirmwareVersion(void);
/** @brief Sets environmental compensation parameters.
 * @param temperatureC Ambient temperature in Celsius.
 * @param humidityPercent Relative humidity in percentage.
 * @return HAL status
 */
HAL_StatusTypeDef ENS160_SetEnvCompensation(float temperatureC, float humidityPercent);

/** @brief Sets the operating mode of the ENS160 sensor.
 * @param mode Desired operating mode.
 * @return HAL status
 */
HAL_StatusTypeDef ENS160_SetOperatingMode(ENS160_OPMODE_t mode);

/** @brief Reads all air quality data from the ENS160 sensor.
 * @param data Pointer to store air quality data.
 * @param resistance Pointer to store resistance values.
 * @return HAL status
 */
HAL_StatusTypeDef ENS160_ReadAllData(ENS160_Data_t *data, ENS160_Resistance_t *resistance);

#endif // ENS160_H
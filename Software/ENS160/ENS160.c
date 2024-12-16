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

#include "ENS160.h"
#include "i2c.h"
#include <math.h> // Required for the pow function

/** @brief External declaration for the I2C handle. */
extern I2C_HandleTypeDef hi2c2;

/** @brief Global instance of the ENS160 register structure. */
static ENS160_Registers_t ENS160_Sensor;
static uint16_t PART_ID;        /**< Part ID register 2 bytes */
static ENS160_FW_t FW;          /**< Firmware version structure 3 bytes */
/**
 * @brief Write data to a register of the ENS160 sensor.
 *
 * @param reg Register address to write to.
 * @param data Pointer to the data buffer to be written.
 * @param len Length of data to write.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef ENS160_WriteRegister(uint8_t reg, uint8_t* data, uint8_t len) {
    return WriteRegister(ENS160_I2C_ADDRESS, reg, data, len, &hi2c2);
}

/**
 * @brief Read data from a register of the ENS160 sensor.
 *
 * @param reg Register address to read from.
 * @param data Pointer to the data buffer to store the read data.
 * @param len Length of data to read.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef ENS160_ReadRegister(uint8_t reg, uint8_t* data, uint8_t len) {
    return ReadRegister(ENS160_I2C_ADDRESS, reg, data, len, &hi2c2);
}

/**
 * @brief Invoke a command on the ENS160 sensor with a specified delay.
 *
 * @param command Command to invoke (see @ref ENS160_COMMAND_t).
 * @param delay_ms Delay in milliseconds after issuing the command.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef ENS160_ExecuteCommand() {
    uint8_t cmd = ENS160_COMMAND_NOP;
		// Set to IDLE mode before issuing commands
    if (ENS160_SetOperatingMode(ENS160_OPMODE_IDLE) != HAL_OK) {
        return HAL_ERROR;
    }

    // Issue NOP command
		cmd = ENS160_COMMAND_NOP; // Store the command in a variable
    if (ENS160_WriteRegister(ENS160_REG_COMMAND, &cmd, ENS160_SIZE1) != HAL_OK) {
        return HAL_ERROR;
    }

    // Clear General Purpose Registers
		cmd = ENS160_COMMAND_CLRGPR;
    if (ENS160_WriteRegister(ENS160_REG_COMMAND, &cmd, ENS160_SIZE1) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(ENS160_CLEAR_REG_DELAY);

    // Issue the specified command
    if (ENS160_WriteRegister(ENS160_REG_COMMAND, (uint8_t *)&ENS160_Sensor.COMMAND, ENS160_SIZE1) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_Delay(50);
    return HAL_OK;
}

/**
 * @brief Calculate resistance based on raw sensor data.
 *
 * @param raw Raw 16-bit resistance data.
 * @return Calculated resistance value in ohms.
 */
static inline uint32_t ENS160_CalculateResistance(uint16_t raw) {
    return (uint32_t)pow(2.0, (float)raw / 2048.0);
}

/**
 * @brief Initialize the ENS160 sensor.
 *
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef ENS160_Init(void) {
    // Reset the sensor
    if (ENS160_SetOperatingMode(ENS160_OPMODE_RESET) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(10);

    // Verify the PART ID
    if (ENS160_ReadPartID() != HAL_OK || PART_ID != ENS160_PART_ID) {
        return HAL_ERROR;
    }
		if (ENS160_GetFirmwareVersion() != HAL_OK) {
				return HAL_ERROR;
		}
    // Set the sensor to IDLE mode
    if (ENS160_SetOperatingMode(ENS160_OPMODE_IDLE) != HAL_OK) {
        return HAL_ERROR;
    }

    // Set default environmental compensation values
    if (ENS160_SetEnvCompensation(20, 50) != HAL_OK) {
        return HAL_ERROR;
    }

    // Configure interrupt settings
    ENS160_Sensor.CONFIG.INTEN = 1;
    ENS160_Sensor.CONFIG.INTDAT = 1;
    ENS160_Sensor.CONFIG.INTGPR = 1;
    ENS160_Sensor.CONFIG.INT_CFG = 0;
    ENS160_Sensor.CONFIG.INTPOL = 0;
    if (ENS160_WriteRegister(ENS160_REG_CONFIG, (uint8_t *)&ENS160_Sensor.CONFIG, ENS160_SIZE1) != HAL_OK) {
        return HAL_ERROR;
    }

    // Switch to STANDARD sensing mode
    if (ENS160_SetOperatingMode(ENS160_OPMODE_STANDARD) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Read and verify the PART_ID of the ENS160 sensor.
 *
 * @return HAL_StatusTypeDef HAL_OK if the PART_ID matches, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef ENS160_ReadPartID(void) {
    return ENS160_ReadRegister(ENS160_REG_PART_ID, (uint8_t *)&PART_ID, ENS160_SIZE2);
}

/**
 * @brief Read the firmware version of the ENS160 sensor.
 *
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef ENS160_GetFirmwareVersion(void) {
    // Issue the Get Firmware Version command
		ENS160_Sensor.COMMAND = ENS160_COMMAND_GET_FWVER;
    if (ENS160_ExecuteCommand() != HAL_OK) {
        return HAL_ERROR;
    }
		if (ENS160_ReadRegister(ENS160_REG_GPR_READ, ENS160_Sensor.GPR_READ, ENS160_SIZE8) != HAL_OK) {
        return HAL_ERROR;
		}
		FW.Major		= ENS160_Sensor.GPR_READ[4];
		FW.Minor		= ENS160_Sensor.GPR_READ[5];
		FW.Release	= ENS160_Sensor.GPR_READ[6];

    return HAL_OK;
}

/**
 * @brief Set environmental compensation values for the ENS160 sensor.
 *
 * @param temperatureC Temperature in degrees Celsius.
 * @param humidityPercent Relative humidity in percentage.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef ENS160_SetEnvCompensation(float temperatureC, float humidityPercent) {
    uint16_t temp = (uint16_t)((temperatureC + 273.15f) * 64);
    uint16_t rh = (uint16_t)(humidityPercent * 512);

    if (ENS160_WriteRegister(ENS160_REG_TEMP_IN, (uint8_t *)&temp, ENS160_SIZE2) != HAL_OK) {
        return HAL_ERROR;
    }
    if (ENS160_WriteRegister(ENS160_REG_RH_IN, (uint8_t *)&rh, ENS160_SIZE2) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Read and decode the device status of the ENS160 sensor.
 *
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef ENS160_ReadDeviceStatus(void) {
    return ENS160_ReadRegister(ENS160_REG_STATUS, (uint8_t *)&ENS160_Sensor.STATUS, ENS160_SIZE1);
}

/**
 * @brief Set the operating mode for the ENS160 sensor.
 *
 * @param mode Desired operating mode (see @ref ENS160_OPMODE_t).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef ENS160_SetOperatingMode(ENS160_OPMODE_t mode) {
    if (ENS160_WriteRegister(ENS160_REG_OPMODE, (uint8_t *)&mode, ENS160_SIZE1) != HAL_OK) {
        return HAL_ERROR;
    }

    ENS160_Sensor.OPMODE = mode;
    return HAL_OK;
}

/**
 * @brief Read air quality and resistance data from the ENS160 sensor.
 *
 * @param data Pointer to the structure to store air quality data.
 * @param resistance Pointer to the structure to store resistance values.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef ENS160_ReadAllData(ENS160_Data_t *data, ENS160_Resistance_t *resistance) {
    // Read device status
    if (ENS160_ReadDeviceStatus() != HAL_OK) {
        return HAL_ERROR;
    }

    // Read air quality data if available
    if (ENS160_Sensor.STATUS.NEWDAT) {
        if (ENS160_ReadRegister(ENS160_REG_DATA_AQI, data->Buffer, ENS160_SIZE5) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    // Read resistance data if available
    if (ENS160_Sensor.STATUS.NEWGPR) {
        if (ENS160_ReadRegister(ENS160_REG_GPR_READ, ENS160_Sensor.GPR_READ, ENS160_SIZE8) != HAL_OK) {
            return HAL_ERROR;
        }

        resistance->R1 = ENS160_CalculateResistance(*(uint16_t *)&ENS160_Sensor.GPR_READ[0]);
        resistance->R2 = ENS160_CalculateResistance(*(uint16_t *)&ENS160_Sensor.GPR_READ[2]);
        resistance->R3 = ENS160_CalculateResistance(*(uint16_t *)&ENS160_Sensor.GPR_READ[4]);
        resistance->R4 = ENS160_CalculateResistance(*(uint16_t *)&ENS160_Sensor.GPR_READ[6]);
    }

    return (ENS160_Sensor.STATUS.NEWDAT || ENS160_Sensor.STATUS.NEWGPR) ? HAL_OK : HAL_ERROR;
}
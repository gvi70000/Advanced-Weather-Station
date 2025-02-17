/**
 * @file HDC302x.c
 * @brief Implementation file for HDC302x temperature and humidity sensor library.
 *
 * Contains functions for initializing, reading data, and configuring the HDC302x sensor.
 */

#include "HDC302x.h"
#include "i2c.h"
#include <math.h>
#include <stdio.h>

static HDC302x_t HDC3020_Sensors[2]; // Sensor connected to 0x44 0x45

/**
 * @brief Calculate CRC checksum for the HDC302x.
 *
 * This function calculates the CRC-8/NRSC-5 checksum as described in the datasheet.
 * It is compatible with both the sensor's transmitted data and byte-swapped values sent to the sensor.
 *
 * @param data Pointer to the two-byte data array.
 * @param byteSwapped Set to true if the bytes are swapped in the data being sent to the sensor.
 * @return The calculated 8-bit CRC checksum.
 */
static uint8_t CalculateCRC(uint8_t *data) {
    uint8_t crc = 0xFF; // Initial value
    for (uint8_t i = 0; i < 2; i++) { // Process 2 bytes
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Encode temperature and humidity thresholds into a 16-bit value with little-endian byte order.
 *
 * This function converts temperature and humidity values into a 16-bit threshold representation
 * as per the datasheet, with 7 MSBs for humidity and 9 MSBs for temperature. The returned value
 * is swapped to little-endian format.
 *
 * @param threshold Pointer to the HDC302x_Data_t structure containing temperature and humidity values.
 * @return Encoded 16-bit threshold value in little-endian format.
 */
static uint16_t EncodeThreshold(HDC302x_Data_t *threshold) {
    uint16_t rawRH = (uint16_t)(threshold->Humidity * HDC302X_RH_COEFF_INV);
    uint16_t rawTemp = (uint16_t)(threshold->Temperature * HDC302X_TEMP_COEFF1_INV + HDC302X_TEMP_COEFF3);
    uint16_t msbRH = (rawRH >> 9) & 0x7F;
    uint16_t msbTemp = (rawTemp >> 7) & 0x1FF;
    uint16_t encoded = (msbRH << 9) | msbTemp;

    // Swap bytes to match little-endian format
    return (encoded >> 8) | (encoded << 8);
}

/**
 * @brief Decode a 16-bit threshold value into temperature and humidity and store in a structure.
 *
 * This function decodes the 16-bit big-endian threshold value by extracting the 7 MSBs
 * for humidity and the 9 MSBs for temperature, converting them to floating-point values,
 * and storing them in the provided `HDC302x_Data_t` structure.
 *
 * @param rawThreshold The raw 16-bit threshold value (big-endian).
 * @param data Pointer to an `HDC302x_Data_t` structure to store the decoded temperature and humidity values.
 */
static void DecodeThreshold(uint16_t rawThreshold, HDC302x_Data_t *data) {
    uint8_t msbRH = (rawThreshold >> 9) & 0x7F;
    uint16_t msbTemp = rawThreshold & 0x1FF;

    data->Humidity = (msbRH * 100.0f) / 127.0f;
    data->Temperature = ((msbTemp * 175.0f) / 511.0f) - 45.0f;
}

/**
 * @brief Write a command to the sensor.
 *
 * @param sensor_index Index of the sensor in HDC3020_Sensors (0 for 0x44, 1 for 0x45).
 * @param command The 16-bit command to send (already byte-swapped).
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
static HAL_StatusTypeDef HDC302x_WriteCommand(uint8_t sensor_index, uint16_t command) {
    return HAL_I2C_Master_Transmit(&hi2c2, HDC3020_Sensors[sensor_index].Address, (uint8_t *)&command, 2, I2C_TIMEOUT);
}

/**
 * @brief Write a command with 16-bit data and CRC to the sensor.
 *
 * This function sends a 16-bit command, 16-bit data, and a CRC byte in a single I2C transaction.
 *
 * @param sensor_index Index of the sensor in HDC3020_Sensors (0 for 0x44, 1 for 0x45).
 * @param command The 16-bit command to send (already byte-swapped if needed).
 * @param data The 16-bit data to send (already byte-swapped if needed).
 * @param crc The CRC byte to append to the transmission.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef HDC302x_WriteCommandWithData(uint8_t sensor_index, uint16_t command, uint16_t data, uint8_t crc) {
    uint8_t buffer[5]; // Command (2 bytes) + Data (2 bytes) + CRC (1 byte)
    // Assign command to the first 2 bytes of the buffer
    *(uint16_t *)&buffer[0] = command;
    // Assign data to the next 2 bytes of the buffer
    *(uint16_t *)&buffer[2] = data;
    // Assign the CRC byte to the last position in the buffer
    buffer[4] = crc;
    // Transmit the buffer via I2C
    return HAL_I2C_Master_Transmit(&hi2c2, HDC3020_Sensors[sensor_index].Address, buffer, sizeof(buffer), I2C_TIMEOUT);
}

/**
 * @brief Read multiple data results from the sensor and return them in little-endian format.
 *
 * @param sensor_index Index of the sensor in HDC3020_Sensors (0 for 0x44, 1 for 0x45).
 * @param command The 16-bit command to read (already byte-swapped).
 * @param results Pointer to store multiple 16-bit results (converted to little-endian).
 * @param result_count Number of 16-bit results to read.
 * @return HAL_StatusTypeDef Status of the I2C operation.
 */
static HAL_StatusTypeDef HDC302x_ReadCmdResults(uint8_t sensor_index, uint16_t command, uint16_t *results, uint8_t result_count) {
    uint8_t recv_buffer[6]; // Buffer for results + CRCs (ensure enough space)
    uint8_t recv_size = result_count * 3; // Each result is 2 bytes + 1 CRC
    // Send the command to the sensor
    if (HDC302x_WriteCommand(sensor_index, command) != HAL_OK) {
        return HAL_ERROR; // Command transmission failed
    }
		HAL_Delay(20);
    // Receive the results from the sensor
    if (HAL_I2C_Master_Receive(&hi2c2, HDC3020_Sensors[sensor_index].Address, recv_buffer, recv_size, I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR; // Reception failed
    }
    // Extract results and validate CRC for each
    for (uint8_t i = 0; i < result_count; i++) {
        uint8_t *current_result = &recv_buffer[i * 3];
        // Combine MSB and LSB into big-endian format
        results[i] = (current_result[0] << 8) | current_result[1];
        // Validate CRC
        if (CalculateCRC(current_result) != current_result[2]) {
            return HAL_ERROR; // CRC mismatch
        }
    }
    return HAL_OK; // All results successfully read, validated, and converted
}

/**
 * @brief Initialize the HDC302x sensor.
 *
 * This function performs the following steps:
 * - Configures the sensor address based on the ID.
 * - Performs a soft reset.
 * - Clears the status register.
 * - Configures default alert limits for temperature and humidity.
 * - Starts auto-measurement at 1 Hz with the lowest noise.
 *
 * @param senID Sensor ID (0 or 1).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC302x_Init(uint8_t senID) {
		uint16_t mID = 0;
    // Set the sensor address based on the sensor ID
    HDC3020_Sensors[senID].Address = (senID == 0) ? HDC302X_SENSOR_1_ADDR : HDC302X_SENSOR_2_ADDR;
    // Step 1: Perform a soft reset
    if (HDC3020_SoftReset(senID) != HAL_OK) {
        return HAL_ERROR; // Soft reset failed
    }
		if (HDC302x_ReadManufacturerID(senID, &mID) != HAL_OK) {
			return HAL_ERROR;
		}
    // Step 2: Clear the status register
    if (HDC3020_ClearStatusRegister(senID) != HAL_OK) {
        return HAL_ERROR; // Clearing the status register failed
    }
		HAL_Delay(100);
		if (HDC302x_SetConfiguration(senID, &HDC302X_CONFIG_1HZ_LOWEST_NOISE) != HAL_OK) {
			return HAL_ERROR;
		}
		
		float o_rh = 1.5;
		float o_t = 5.3;
		if (HDC3020_SetOffset(senID, o_rh, o_t) != HAL_OK) {
			return HAL_ERROR;
		}
//		HAL_Delay(100);
//		if (HDC3020_GetOffset(senID, &o_rh, &o_t) != HAL_OK) {
//			return HAL_ERROR;
//		}
    // Step 3: Configure default alert limits
    // Default limits: RH (0% - 100%) and Temp (-40°C to 125°C)
		
    HDC302x_Data_t highAlertValue = { .Temperature = 120.0f, .Humidity = 2.0f };
		HDC302x_Data_t lowAlertValue = { .Temperature = 120.0f, .Humidity = 2.0f };
		HDC302x_Data_t highAlertClear = { .Temperature = 120.0f, .Humidity = 2.0f };
		HDC302x_Data_t lowAlertClear = { .Temperature = 120.0f, .Humidity = 2.0f };
		
    if (HDC302x_SetAlertLimits(senID, highAlertValue, lowAlertValue, highAlertClear, lowAlertClear) != HAL_OK) {
        return HAL_ERROR; // Setting alert limits failed
    }
		if (HDC302x_GetAlertLimits(senID, &highAlertValue, &lowAlertValue, &highAlertClear, &lowAlertClear) != HAL_OK) {
			return HAL_ERROR; // Setting alert limits failed
		}
    // Step 4: Start auto-measurement mode with the lowest noise at 1 Hz
    if (HDC3020_SelectMeasurementMode(senID, HDC302X_CMD_MEASURE_01_LPM0) != HAL_OK) {
        return HAL_ERROR; // Starting auto-measurement mode failed
    }
    return HAL_OK; // Initialization successful
}


/**
 * @brief Perform a soft reset on the HDC302x sensor.
 *
 * This function sends the soft reset command to the sensor and allows time for the device to reset.
 *
 * @param senID Sensor ID (0 or 1).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC3020_SoftReset(uint8_t senID) {
    // Send the soft reset command
    if (HDC302x_WriteCommand(senID, HDC302X_CMD_SOFT_RESET) != HAL_OK) {
        return HAL_ERROR; // Return error if the command fails
    }
    // Allow time for the device to reset
    HAL_Delay(20);
    return HAL_OK;
}

/**
 * @brief Set the configuration of the HDC302x sensor.
 *
 * This function writes a configuration value and its CRC to the sensor to set its operating mode.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @param config Pointer to the HDC302x_Config_t structure containing the configuration value and CRC.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC302x_SetConfiguration(uint8_t senID, const HDC302x_Config_t *config) {
    uint8_t buffer[5]; // Configuration value (2 bytes) + CRC (1 byte)
    // Prepare the buffer
		*(uint16_t *)&buffer[0] = HDC302X_CMD_PROGRAM_READ_DEFAULT_STATE;
		*(uint16_t *)&buffer[2] = config->CFG_VAL;
    buffer[4] = config->CFG_CRC; // Pre-calculated CRC value
    // Write the configuration value and CRC to the sensor
    return HAL_I2C_Master_Transmit(&hi2c2, HDC3020_Sensors[senID].Address, buffer, sizeof(buffer), I2C_TIMEOUT);
}

/**
 * @brief Read the status register of the HDC3020 sensor and store it in the corresponding structure.
 *
 * This function performs the I2C sequence to read the status register,
 * including sending the appropriate command using `SendDeviceCommand`
 * and storing the status in the corresponding sensor structure.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC3020_ReadStatusRegister(uint8_t senID) {
    // Read the status register using HDC302x_ReadMultipleResults with result_count = 1
		return HDC302x_ReadCmdResults(senID, HDC302X_READ_STATUS, &HDC3020_Sensors[senID].Status.Val.Value, 1);// Single result
}

/**
 * @brief Clear the status register of the HDC3020 sensor.
 *
 * This function uses the `SendDeviceCommand` function to clear the status register
 * by sending the predefined command.
 *
 * @param senID Sensor ID (0 or 1).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC3020_ClearStatusRegister(uint8_t sensor_index) {
    // Send the CLEAR STATUS command using HDC302x_WriteCommand
    if (HDC302x_WriteCommand(sensor_index, HDC302X_CLEAR_STATUS) != HAL_OK) {
        return HAL_ERROR; // Return the error if the operation failed
    }
    // Clear the status in the corresponding sensor's data structure
    HDC3020_Sensors[sensor_index].Status.Val.Value = 0;
    return HAL_OK;
}

/**
 * @brief Read the temperature and humidity from the sensor.
 * @param sensor_index Sensor ID (0 or 1).
 * @param data Pointer to store the read data.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC302x_ReadTemperatureAndHumidity(uint8_t sensor_index, HDC302x_Data_t *data) {
    uint16_t results[2]; // Raw temperature and humidity (big-endian format from the sensor)
    // Read raw temperature and humidity
    if (HDC302x_ReadCmdResults(sensor_index, HDC302X_CMD_MEASURE_READ, results, 2) != HAL_OK) {
        return HAL_ERROR; // Communication or CRC failure
    }
    // Convert results from big-endian to little-endian
    uint16_t rawHumidity = results[0];//(results[0] >> 8) | (results[0] << 8); // Swap bytes for humidity
    uint16_t rawTemperature = results[1];//(results[1] >> 8) | (results[1] << 8); // Swap bytes for temperature
    // Convert raw data to physical values
    data->Temperature = (rawTemperature * HDC302X_TEMP_COEFF1) - HDC302X_TEMP_COEFF2; // Convert raw temp to °C
    data->Humidity = rawHumidity * HDC302X_RH_COEFF; // Convert raw humidity to %
    return HAL_OK;
}

/**
 * @brief Select the measurement mode for the HDC3020 sensor.
 *
 * This function sends the specified measurement mode command to the sensor to configure its operation.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @param command The 16-bit command representing the desired measurement mode (already byte-swapped).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC3020_SelectMeasurementMode(uint8_t senID, uint16_t command) {
    // Return the result of the command transmission
    return HDC302x_WriteCommand(senID, command);
}

/**
 * @brief Retrieve the measurement history from the HDC3020 sensor.
 *
 * This function reads the minimum and maximum temperature and humidity values from the sensor and stores them
 * in the provided `HDC302x_History_t` structure.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @param history Pointer to an `HDC302x_History_t` structure to store the measurement history.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC3020_GetMeasurementHistory(uint8_t senID, HDC302x_History_t *history) {
    uint16_t rawMinTemp, rawMaxTemp, rawMinHumidity, rawMaxHumidity;
    // Read minimum temperature
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_HISTORY_MIN_T, &rawMinTemp, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to read minimum temperature
    }
    // Read maximum temperature
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_HISTORY_MAX_T, &rawMaxTemp, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to read maximum temperature
    }
    // Read minimum humidity
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_HISTORY_MIN_RH, &rawMinHumidity, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to read minimum humidity
    }
    // Read maximum humidity
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_HISTORY_MAX_RH, &rawMaxHumidity, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to read maximum humidity
    }
    // Convert raw values to physical units using macros
    history->MIN.Temperature = (rawMinTemp * HDC302X_TEMP_COEFF1) - HDC302X_TEMP_COEFF2;
    history->MAX.Temperature = (rawMaxTemp * HDC302X_TEMP_COEFF1) - HDC302X_TEMP_COEFF2;
    history->MIN.Humidity = rawMinHumidity * HDC302X_RH_COEFF;
    history->MAX.Humidity = rawMaxHumidity * HDC302X_RH_COEFF;
    return HAL_OK; // All history values successfully read
}

/**
 * @brief Set the alert limits for temperature and humidity on the HDC302x sensor.
 *
 * This function configures the high and low alert limits and their respective clear thresholds
 * for temperature and humidity.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @param highAlertValue High alert threshold values for temperature and humidity.
 * @param lowAlertValue Low alert threshold values for temperature and humidity.
 * @param highAlertClear High alert clear values for temperature and humidity.
 * @param lowAlertClear Low alert clear values for temperature and humidity.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC302x_SetAlertLimits(uint8_t senID, HDC302x_Data_t highAlertValue, HDC302x_Data_t lowAlertValue, HDC302x_Data_t highAlertClear, HDC302x_Data_t lowAlertClear) {
    uint16_t threshold = 0;
    uint8_t crc = 0;
    
    // Set High Alert Threshold
    threshold = EncodeThreshold(&highAlertValue);
    threshold = __builtin_bswap16(threshold); // Convert to big-endian
    crc = CalculateCRC((uint8_t *)&threshold);
    if (HDC302x_WriteCommandWithData(senID, HDC302X_CMD_SET_ALERT_HIGH, threshold, crc) != HAL_OK) {
        return HAL_ERROR;
    }

    // Set Low Alert Threshold
    threshold = EncodeThreshold(&lowAlertValue);
    threshold = __builtin_bswap16(threshold); // Convert to big-endian
    crc = CalculateCRC((uint8_t *)&threshold);
    if (HDC302x_WriteCommandWithData(senID, HDC302X_CMD_SET_ALERT_LOW, threshold, crc) != HAL_OK) {
        return HAL_ERROR;
    }

    // Set High Alert Clear Threshold
    threshold = EncodeThreshold(&highAlertClear);
    threshold = __builtin_bswap16(threshold); // Convert to big-endian
    crc = CalculateCRC((uint8_t *)&threshold);
    if (HDC302x_WriteCommandWithData(senID, HDC302X_CMD_CLR_ALERT_HIGH, threshold, crc) != HAL_OK) {
        return HAL_ERROR;
    }

    // Set Low Alert Clear Threshold
    threshold = EncodeThreshold(&lowAlertClear);
    threshold = __builtin_bswap16(threshold); // Convert to big-endian
    crc = CalculateCRC((uint8_t *)&threshold);
    if (HDC302x_WriteCommandWithData(senID, HDC302X_CMD_CLR_ALERT_LOW, threshold, crc) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Get the alert limits for temperature and humidity from the HDC302x sensor.
 *
 * This function reads the high and low alert thresholds and their respective clear thresholds
 * for temperature and humidity from the sensor and decodes them into `HDC302x_Data_t` structures.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @param highAlertValue Pointer to store high alert threshold values for temperature and humidity.
 * @param lowAlertValue Pointer to store low alert threshold values for temperature and humidity.
 * @param highAlertClear Pointer to store high alert clear values for temperature and humidity.
 * @param lowAlertClear Pointer to store low alert clear values for temperature and humidity.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC302x_GetAlertLimits(uint8_t senID, HDC302x_Data_t *highAlertValue, HDC302x_Data_t *lowAlertValue, HDC302x_Data_t *highAlertClear, HDC302x_Data_t *lowAlertClear) {
    uint16_t rawThreshold;

    // Get High Alert Threshold
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_SET_ALERT_HIGH, &rawThreshold, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    rawThreshold = __builtin_bswap16(rawThreshold); // Convert from big-endian to little-endian
    DecodeThreshold(rawThreshold, highAlertValue);

    // Get Low Alert Threshold
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_SET_ALERT_LOW, &rawThreshold, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    rawThreshold = __builtin_bswap16(rawThreshold); // Convert from big-endian to little-endian
    DecodeThreshold(rawThreshold, lowAlertValue);

    // Get High Alert Clear Threshold
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_CLR_ALERT_HIGH, &rawThreshold, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    rawThreshold = __builtin_bswap16(rawThreshold); // Convert from big-endian to little-endian
    DecodeThreshold(rawThreshold, highAlertClear);

    // Get Low Alert Clear Threshold
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_CLR_ALERT_LOW, &rawThreshold, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    rawThreshold = __builtin_bswap16(rawThreshold); // Convert from big-endian to little-endian
    DecodeThreshold(rawThreshold, lowAlertClear);

    return HAL_OK;
}

/**
 * @brief Set the temperature and humidity offset values for the HDC3020 sensor.
 *
 * This function programs the combined temperature and humidity offset values into the sensor.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @param RH_Offset Offset for relative humidity (in percentage, e.g., +8.2% or -7.8%).
 * @param T_Offset Offset for temperature (in degrees Celsius, e.g., +7.1°C or -10.9°C).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC3020_SetOffset(uint8_t senID, float RH_Offset, float T_Offset) {
    uint16_t CodeMask = 0x8080;
    
    // Remove fabs() and use conditional negation for speed
    if (T_Offset < 0) {
        T_Offset = -T_Offset;
        CodeMask &= 0xFF7F;
    }
    if (RH_Offset < 0) {
        RH_Offset = -RH_Offset;
        CodeMask &= 0x7FFF;
    }

    // Integer scaling to avoid floating point division
    uint16_t tCode = (uint16_t)(T_Offset * 65535 / 175);
    uint16_t hCode = (uint16_t)(RH_Offset * 65535 / 100);

    // Encode the offsets
    uint16_t combinedOffset = ((hCode << 1) & 0x7F00) | ((tCode >> 6) & 0x7F) | CodeMask;

    // Swap bytes to match STM32 endianness
    combinedOffset = (combinedOffset >> 8) | (combinedOffset << 8);

    // Print debug information
    printf("Writing Combined Offset: 0x%04X\n", combinedOffset);

    // Compute CRC
    uint8_t crc = CalculateCRC((uint8_t *)&combinedOffset);  // Pass 2 bytes only

    // Print CRC value
    printf("CRC Sent: 0x%02X\n", crc);

    // Write to sensor via I2C
    return HDC302x_WriteCommandWithData(senID, HDC302X_CMD_PROGRAM_READ_OFFSET_VALUES, combinedOffset, crc);
}

/**
 * @brief Verify Programmed RH and Temperature Offset Values.
 *
 * This function reads the programmed RH and temperature offset values, decodes them,
 * and calculates the respective floating-point offsets.
 *
 * @param senID Sensor ID (0 or 1).
 * @param rhOffset Pointer to store the RH offset value.
 * @param tOffset Pointer to store the T offset value.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC3020_GetOffset(uint8_t senID, float *rhOffset, float *tOffset) {
    uint16_t combinedOffset;

    // Read the combined offset value from the sensor
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_PROGRAM_READ_OFFSET_VALUES, &combinedOffset, 1) != HAL_OK) {
        return HAL_ERROR;
    }

    // Print the raw value read from the sensor
    printf("Read Combined Offset: 0x%04X\n", combinedOffset);

    // Extract RH Offset and Temperature Offset
    uint8_t RH_offset = (combinedOffset >> 8) & 0xFF;  // High byte
    uint8_t T_offset  = combinedOffset & 0xFF;         // Low byte

    printf("Extracted RH Offset Byte: 0x%02X, Extracted T Offset Byte: 0x%02X\n", RH_offset, T_offset);

    // Extract sign bits (Bit 7)
    uint8_t RH_negative = !(RH_offset & 0x80);
    uint8_t T_negative  = !(T_offset & 0x80);

    // Remove sign bits
    RH_offset &= 0x7F;
    T_offset  &= 0x7F;

    // Convert back to floating-point values
    *rhOffset = RH_offset * 0.1953125f; // Convert RH back to % units
    *tOffset  = T_offset * 0.1708984375f; // Convert Temp back to Celsius

    // Apply sign if needed
    if (RH_negative) *rhOffset = -*rhOffset;
    if (T_negative)  *tOffset  = -*tOffset;

    // Print final computed values
    printf("Final RH Offset: %.3f%%, Final T Offset: %.3fC\n", *rhOffset, *tOffset);

    return HAL_OK;
}

/**
 * @brief Check if the heater on the HDC302x sensor is currently enabled.
 *
 * This function reads the status register of the specified sensor and
 * returns the heater status bit.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @return uint8_t Heater status:
 *         - 1: Heater is enabled.
 *         - 0: Heater is disabled.
 *         - 9: Error reading the status register.
 */
uint8_t HDC3020_IsHeaterOn(uint8_t senID) {
    // Read the status register
    if (HDC3020_ReadStatusRegister(senID) != HAL_OK) {
        return 9; // Error reading the status register
    }
    // Return the heater status bit
    return HDC3020_Sensors[senID].Status.Val.BitField.heater_status;
}

/**
 * @brief Control the heater of the HDC302x sensor.
 *
 * This function enables or disables the heater based on the provided configuration.
 * If the configuration is `HDC302X_HEATER_OFF`, the heater is disabled.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @param config Pointer to the heater configuration (HDC302x_HeaterConfig_t). Use `HDC302X_HEATER_OFF` to disable the heater.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC3020_ControlHeater(uint8_t senID, const HDC302x_HeaterConfig_t *config) {
    // Check if the heater should be disabled
    if (config->HEATER_VAL == HDC302X_HEATER_OFF.HEATER_VAL) {
        // Send the command to disable the heater
        if (HDC302x_WriteCommand(senID, HDC302X_CMD_HEATER_DISABLE) != HAL_OK) {
            return HAL_ERROR; // Failed to disable heater
        }
        return HAL_OK; // Heater successfully disabled
    } else {
				// Enable the heater
				if (HDC302x_WriteCommand(senID, HDC302X_CMD_HEATER_ENABLE) != HAL_OK) {
						return HAL_ERROR; // Failed to enable heater
				}
				// Send the heater configuration command
				if (HDC302x_WriteCommandWithData(senID, HDC302X_CMD_HEATER_CONFIG, config->HEATER_VAL, config->HCRC) != HAL_OK) {
						return HAL_ERROR; // Failed to configure heater
				}
		}	
    return HAL_OK; // Heater successfully enabled and configured
}

/**
 * @brief Program alert thresholds to NVM (EEPROM) on the HDC302x sensor.
 *
 * @param senID Sensor ID (0 or 1).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC302x_ProgramAlertThresholdsToNVM(uint8_t senID) {
    // Send the program alert thresholds command
    if (HDC302x_WriteCommand(senID, HDC302X_CMD_PROGRAM_ALERT_THRESHOLDS_TO_NVM) != HAL_OK) {
        return HAL_ERROR; // Command transmission failed
    }
    // Delay to allow programming to complete
    HAL_Delay(10); // Datasheet recommends a delay of at least 10 ms for NVM programming
    return HAL_OK;
}

/**
 * @brief Program or read the default power-on/reset measurement state for the HDC302x sensor.
 *
 * @param senID Sensor ID (0 or 1).
 * @param programDefault Set to true to program default state; false to read current state.
 * @param state Pointer to a 16-bit value containing the measurement state.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC302x_ProgramReadDefaultState(uint8_t senID, uint8_t programDefault, uint16_t *state) {
		if (programDefault) {
        // Program the default state
        return HDC302x_WriteCommandWithData(senID, HDC302X_CMD_PROGRAM_READ_DEFAULT_STATE, *state, CalculateCRC((uint8_t *)state));
    } else {
        // Read the default state
        if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_PROGRAM_READ_DEFAULT_STATE, state, 1) != HAL_OK) {
            return HAL_ERROR; // Read operation failed
        }
        return HAL_OK;
    }
}

/**
 * @brief Read the Manufacturer ID from the HDC302x sensor.
 *
 * This function sends the command to read the Manufacturer ID and retrieves the 16-bit ID value.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @param manufacturerID Pointer to store the 16-bit Manufacturer ID.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC302x_ReadManufacturerID(uint8_t senID, uint16_t *manufacturerID) {
    return HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_MANUFACTURER_ID, manufacturerID, 1);// Single result
}

float CalculateDewPoint(float temperature, float humidity) {
    float alpha = (DEW_POINT_CONST_A * temperature) / (DEW_POINT_CONST_B + temperature) + logf(humidity / 100.0f);
    float dew_point = (DEW_POINT_CONST_B * alpha) / (DEW_POINT_CONST_A - alpha);
    return dew_point;
}
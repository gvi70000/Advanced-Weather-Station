/**
 * @file HDC302x.c
 * @brief Implementation file for HDC302x temperature and humidity sensor library.
 *
 * Contains functions for initializing, reading data, and configuring the HDC302x sensor.
 */

#include "HDC302x.h"
#include "i2c.h"
#include <math.h>


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
 * @brief Calculates the closest matching offset byte for temperature or relative humidity.
 *
 * @param value The floating-point offset value (e.g., +8.2% RH or -7.8°C).
 * @param lsb The least significant bit (LSB) step for the offset (e.g., 0.1953125 for RH or 0.1708984375 for Temp).
 * @return uint8_t The calculated offset byte.
 */
static uint8_t calculateOffset(float value, float lsb) {
    uint8_t sign = (value < 0) ? HDC302X_SIGN_MASK : 0; // Set sign bit if negative
    if (value < 0) {
        value = -value; // Use absolute value for calculation
    }
    uint8_t offset = (uint8_t)(value / lsb + 0.5f); // Round to nearest value
    return sign | offset; // Combine sign bit and offset
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
    // Convert RH to a 16-bit value (scale: 0% -> 0x0000, 100% -> 0xFFFF)
    uint16_t rawRH = (uint16_t)(threshold->Humidity * HDC302X_RH_COEFF_INV);
    // Convert Temperature to a 16-bit value (scale: -40°C -> 0x0000, 125°C -> 0xFFFF)
    uint16_t rawTemp = (uint16_t)(threshold->Temperature * HDC302X_TEMP_COEFF1_INV + HDC302X_TEMP_COEFF3);
    // Extract 7 MSBs from RH and 9 MSBs from Temperature
    uint16_t msbRH = (rawRH >> 9) & 0x7F;      // 7 MSBs of RH
    uint16_t msbTemp = (rawTemp >> 7) & 0x1FF; // 9 MSBs of Temperature
    // Concatenate MSBs into a 16-bit threshold
    uint16_t encoded = (msbRH << 9) | msbTemp;
    // Swap bytes to return the value in little-endian format
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
    // Decode RH and Temperature
    uint8_t msbRH = (rawThreshold >> 9) & 0x7F;    // Extract 7 MSBs for RH
    uint16_t msbTemp = rawThreshold & 0x1FF;       // Extract 9 MSBs for Temp
    data->Humidity = (msbRH * 100.0f) / 127.0f;    // Convert RH to percentage
    data->Temperature = ((msbTemp * 175.0f) / 511.0f) - 45.0f; // Convert Temp to Celsius
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
    // Step 3: Configure default alert limits
    // Default limits: RH (0% - 100%) and Temp (-40°C to 125°C)
    HDC302x_Data_t highLowAlertClear = { .Temperature = 120.0f, .Humidity = 2.0f };
    if (HDC302x_SetAlertLimits(senID, highLowAlertClear, highLowAlertClear, highLowAlertClear, highLowAlertClear) != HAL_OK) {
        return HAL_ERROR; // Setting alert limits failed
    }
		if (HDC302x_GetAlertLimits(senID, &highLowAlertClear, &highLowAlertClear, &highLowAlertClear, &highLowAlertClear) != HAL_OK) {
			return HAL_ERROR; // Setting alert limits failed
		}
    // Step 4: Start auto-measurement mode with the lowest noise at 1 Hz
    if (HDC3020_SelectMeasurementMode(senID, HDC302X_CMD_AUTO_MEASUREMENT_1_PER_SECOND_LPM0) != HAL_OK) {
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
    uint8_t buffer[3]; // Configuration value (2 bytes) + CRC (1 byte)
    // Prepare the buffer
    buffer[0] = (uint8_t)(config->CFG_VAL >> 8); // MSB of configuration value
    buffer[1] = (uint8_t)(config->CFG_VAL & 0xFF); // LSB of configuration value
    buffer[2] = config->CFG_CRC; // Pre-calculated CRC value
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
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_HISTORY_MIN_TEMPERATURE, &rawMinTemp, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to read minimum temperature
    }
    // Read maximum temperature
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_HISTORY_MAX_TEMPERATURE, &rawMaxTemp, 1) != HAL_OK) {
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
    uint16_t threshold = 0; // Encoded thresholds
    uint8_t crc = 0;        // Calculated CRC
    // Encode and set high alert threshold
    threshold = EncodeThreshold(&highAlertValue);
    crc = CalculateCRC((uint8_t *)&threshold);
    if (HDC302x_WriteCommandWithData(senID, HDC302X_CMD_CONFIGURE_ALERT_THRESHOLD_SET_HIGH, threshold, crc) != HAL_OK) {
        return HAL_ERROR; // Failed to set high alert threshold
    }
    // Encode and set low alert threshold
    threshold = EncodeThreshold(&lowAlertValue);
    crc = CalculateCRC((uint8_t *)&threshold);
    if (HDC302x_WriteCommandWithData(senID, HDC302X_CMD_CONFIGURE_ALERT_THRESHOLD_SET_LOW, threshold, crc) != HAL_OK) {
        return HAL_ERROR; // Failed to set low alert threshold
    }
    // Encode and set high clear threshold
    threshold = EncodeThreshold(&highAlertClear);
    crc = CalculateCRC((uint8_t *)&threshold);
    if (HDC302x_WriteCommandWithData(senID, HDC302X_CMD_CONFIGURE_ALERT_THRESHOLD_CLEAR_HIGH, threshold, crc) != HAL_OK) {
        return HAL_ERROR; // Failed to set high clear threshold
    }
    // Encode and set low clear threshold
    threshold = EncodeThreshold(&lowAlertClear);
    crc = CalculateCRC((uint8_t *)&threshold);
    if (HDC302x_WriteCommandWithData(senID, HDC302X_CMD_CONFIGURE_ALERT_THRESHOLD_CLEAR_LOW, threshold, crc) != HAL_OK) {
        return HAL_ERROR; // Failed to set low clear threshold
    }
    return HAL_OK; // Successfully set all thresholds
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
    uint16_t rawThreshold; // Raw 16-bit threshold value

    // Read high alert threshold
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_ALERT_THRESHOLD_SET_HIGH, &rawThreshold, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to read high alert threshold
    }
    DecodeThreshold(rawThreshold, highAlertValue); // Decode high alert threshold

    // Read low alert threshold
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_ALERT_THRESHOLD_SET_LOW, &rawThreshold, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to read low alert threshold
    }
    DecodeThreshold(rawThreshold, lowAlertValue); // Decode low alert threshold

    // Read high alert clear threshold
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_ALERT_THRESHOLD_CLEAR_HIGH, &rawThreshold, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to read high alert clear threshold
    }
    DecodeThreshold(rawThreshold, highAlertClear); // Decode high alert clear threshold

    // Read low alert clear threshold
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_ALERT_THRESHOLD_CLEAR_LOW, &rawThreshold, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to read low alert clear threshold
    }
    DecodeThreshold(rawThreshold, lowAlertClear); // Decode low alert clear threshold

    return HAL_OK; // Successfully read and decoded all thresholds
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
    // Calculate RH and T offsets
    uint8_t RH_offset = calculateOffset(RH_Offset, RH_LSB); // RH offset with sign bit (Bit 15)
    uint8_t T_offset = calculateOffset(T_Offset, T_LSB);    // T offset with sign bit (Bit 7)
		// Combine RH and T offsets into a 16-bit value (big-endian format)
    uint16_t combinedOffset = (T_offset << 8) | RH_offset;
    // Write the combined offset value to the sensor
    return HDC302x_WriteCommandWithData(
        senID,
        HDC302X_CMD_PROGRAM_READ_OFFSET_VALUES, // Command to program offsets
        combinedOffset,
        CalculateCRC((uint8_t *)&combinedOffset) // Calculate CRC for data integrity
    );
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
HAL_StatusTypeDef HDC3020_VerifyOffset(uint8_t senID, float *rhOffset, float *tOffset) {
    uint16_t combinedOffset; // Combined 16-bit offset value
    // Read the combined offset value
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_PROGRAM_READ_OFFSET_VALUES, &combinedOffset, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to read offset
    }
    // Extract RH Offset (Bits 14-8) and decode
    uint8_t rawRHOffset = (combinedOffset >> 8) & 0x7F; // 7 bits for RH offset
    *rhOffset = (combinedOffset & 0x8000)  // Check RH sign bit (Bit 15)
                ? -(rawRHOffset * HDC302X_RH_COEFF_INV) // Negative offset
                : (rawRHOffset * HDC302X_RH_COEFF_INV); // Positive offset
    // Extract T Offset (Bits 6-0) and decode
    uint8_t rawTOffset = combinedOffset & 0x7F; // 7 bits for T offset
    *tOffset = (combinedOffset & 0x0080)  // Check T sign bit (Bit 7)
                ? -(rawTOffset * HDC302X_TEMP_COEFF1_INV) // Negative offset
                : (rawTOffset * HDC302X_TEMP_COEFF1_INV); // Positive offset
    return HAL_OK; // Successfully verified offsets
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
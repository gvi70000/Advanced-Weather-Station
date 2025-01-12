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
    uint16_t rawRH = (uint16_t)((threshold->Humidity / 100.0f) * 65535.0f);

    // Convert Temperature to a 16-bit value (scale: -40°C -> 0x0000, 125°C -> 0xFFFF)
    uint16_t rawTemp = (uint16_t)(((threshold->Temperature + 45.0f) / 175.0f) * 65535.0f);

    // Extract 7 MSBs from RH and 9 MSBs from Temperature
    uint16_t msbRH = (rawRH >> 9) & 0x7F;      // 7 MSBs of RH
    uint16_t msbTemp = (rawTemp >> 7) & 0x1FF; // 9 MSBs of Temperature

    // Concatenate MSBs into a 16-bit threshold
    uint16_t encoded = (msbRH << 9) | msbTemp;

    // Swap bytes to return the value in little-endian format
    return (encoded >> 8) | (encoded << 8);
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
    uint8_t recv_buffer[64]; // Buffer for results + CRCs (ensure enough space)
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
        uint16_t raw_result = (current_result[0] << 8) | current_result[1];
        // Validate CRC
        if (CalculateCRC(current_result) != current_result[2]) {
            return HAL_ERROR; // CRC mismatch
        }
        // Convert the validated result to little-endian format
        results[i] = raw_result;//(raw_result >> 8) | (raw_result << 8);
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
    HDC302x_Data_t highLowAlertClear = { .Temperature = 120.0f, .Humidity = 1.0f };
    if (HDC302x_SetAlertLimits(senID, highLowAlertClear, highLowAlertClear, highLowAlertClear, highLowAlertClear) != HAL_OK) {
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
 * @param sensor_index Sensor index (0 or 1).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC3020_ReadStatusRegister(uint8_t sensor_index) {
    // Read the status register using HDC302x_ReadMultipleResults with result_count = 1
		return HDC302x_ReadCmdResults(sensor_index, HDC302X_READ_STATUS, &HDC3020_Sensors[sensor_index].Status.Val.Value, 1);// Single result
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
 * @param senID Sensor ID (0 or 1).
 * @param data Pointer to store the read data.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC302x_ReadTemperatureAndHumidity(uint8_t sensor_index, HDC302x_Data_t *data) {
    uint16_t results[2]; // Raw temperature and humidity
    // Read raw temperature and humidity
    if (HDC302x_ReadCmdResults(sensor_index, HDC302X_CMD_MEASURE_READ, results, 2) != HAL_OK) {
        return HAL_ERROR; // Communication or CRC failure
    }
    // Convert raw data to physical values
    data->Temperature = (results[1] * HDC302X_TEMP_COEFF1) - HDC302X_TEMP_COEFF2; // Convert raw temp to °C
    data->Humidity = results[0] * HDC302X_RH_COEFF; // Convert raw humidity to %
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
 * for temperature and humidity from the sensor and stores them in the provided structures.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @param highAlertValue Pointer to store high alert threshold values for temperature and humidity.
 * @param lowAlertValue Pointer to store low alert threshold values for temperature and humidity.
 * @param highAlertClear Pointer to store high alert clear values for temperature and humidity.
 * @param lowAlertClear Pointer to store low alert clear values for temperature and humidity.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC302x_GetAlertLimits(uint8_t senID, HDC302x_Data_t *highAlertValue, HDC302x_Data_t *lowAlertValue, HDC302x_Data_t *highAlertClear, HDC302x_Data_t *lowAlertClear) {
    uint16_t thresholds[2]; // Raw values for temperature and humidity
    // Read high alert thresholds
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_ALERT_THRESHOLD_SET_HIGH, thresholds, 2) != HAL_OK) {
        return HAL_ERROR; // Failed to read high alert thresholds
    }
    highAlertValue->Temperature = (thresholds[0] * HDC302X_TEMP_COEFF1) - HDC302X_TEMP_COEFF2;
    highAlertValue->Humidity = thresholds[1] * HDC302X_RH_COEFF;

    // Read low alert thresholds
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_ALERT_THRESHOLD_SET_LOW, thresholds, 2) != HAL_OK) {
        return HAL_ERROR; // Failed to read low alert thresholds
    }
    lowAlertValue->Temperature = (thresholds[0] * HDC302X_TEMP_COEFF1) - HDC302X_TEMP_COEFF2;
    lowAlertValue->Humidity = thresholds[1] * HDC302X_RH_COEFF;

    // Read high alert clear thresholds
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_ALERT_THRESHOLD_CLEAR_HIGH, thresholds, 2) != HAL_OK) {
        return HAL_ERROR; // Failed to read high alert clear thresholds
    }
    highAlertClear->Temperature = (thresholds[0] * HDC302X_TEMP_COEFF1) - HDC302X_TEMP_COEFF2;
    highAlertClear->Humidity = thresholds[1] * HDC302X_RH_COEFF;

    // Read low alert clear thresholds
    if (HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_ALERT_THRESHOLD_CLEAR_LOW, thresholds, 2) != HAL_OK) {
        return HAL_ERROR; // Failed to read low alert clear thresholds
    }
    lowAlertClear->Temperature = (thresholds[0] * HDC302X_TEMP_COEFF1) - HDC302X_TEMP_COEFF2;
    lowAlertClear->Humidity = thresholds[1] * HDC302X_RH_COEFF;
    return HAL_OK; // All thresholds successfully read
}

/**
 * @brief Set the temperature and humidity offset values for the HDC3020 sensor.
 *
 * This function programs the combined temperature and humidity offset values into the sensor.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @param RH_Offset Offset for relative humidity (in percentage, e.g., +8.203125% or -7.8125%).
 * @param T_Offset Offset for temperature (in degrees Celsius, e.g., +7.177734375°C or -10.9375°C).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
/**
 * @brief Set the temperature and humidity offset values for the HDC3020 sensor.
 *
 * This function programs the combined temperature and humidity offset values into the sensor.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @param RH_Offset Offset for relative humidity (in percentage, e.g., +8.203125% or -7.8125%).
 * @param T_Offset Offset for temperature (in degrees Celsius, e.g., +7.177734375°C or -10.9375°C).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC3020_SetOffset(uint8_t senID, float RH_Offset, float T_Offset) {
    uint16_t combinedOffset = 0; // Combined 16-bit offset value

    // Calculate RH offset directly
    if (RH_Offset < 0) {
        combinedOffset |= 0x8000; // Set RH sign bit (Bit 15)
        RH_Offset = -RH_Offset;  // Convert to positive for calculation
    }
    if (RH_Offset >= 12.5f)    { combinedOffset |= (1 << 14); RH_Offset -= 12.5f; }
    if (RH_Offset >= 6.25f)    { combinedOffset |= (1 << 13); RH_Offset -= 6.25f; }
    if (RH_Offset >= 3.125f)   { combinedOffset |= (1 << 12); RH_Offset -= 3.125f; }
    if (RH_Offset >= 1.5625f)  { combinedOffset |= (1 << 11); RH_Offset -= 1.5625f; }
    if (RH_Offset >= 0.78125f) { combinedOffset |= (1 << 10); RH_Offset -= 0.78125f; }
    if (RH_Offset >= 0.390625f){ combinedOffset |= (1 << 9);  RH_Offset -= 0.390625f; }
    if (RH_Offset >= 0.1953125f){ combinedOffset |= (1 << 8); }

    // Calculate T offset directly
    if (T_Offset < 0) {
        combinedOffset |= 0x0080; // Set T sign bit (Bit 7)
        T_Offset = -T_Offset;    // Convert to positive for calculation
    }
    if (T_Offset >= 10.9375f)   { combinedOffset |= (1 << 6); T_Offset -= 10.9375f; }
    if (T_Offset >= 5.46875f)   { combinedOffset |= (1 << 5); T_Offset -= 5.46875f; }
    if (T_Offset >= 2.734375f)  { combinedOffset |= (1 << 4); T_Offset -= 2.734375f; }
    if (T_Offset >= 1.3671875f) { combinedOffset |= (1 << 3); T_Offset -= 1.3671875f; }
    if (T_Offset >= 0.68359375f){ combinedOffset |= (1 << 2); T_Offset -= 0.68359375f; }
    if (T_Offset >= 0.341796875f){ combinedOffset |= (1 << 1); T_Offset -= 0.341796875f; }
    if (T_Offset >= 0.1708984375f){ combinedOffset |= (1 << 0); }

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
 * This function sends a command to read the RH and temperature offset values and validates
 * the CRC to ensure data integrity.
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

    // Extract RH Offset (Bits 14–8) and decode
    uint8_t rawRHOffset = (combinedOffset >> 8) & 0x7F; // 7 bits for RH offset
    if (combinedOffset & 0x8000) { // RH sign bit (Bit 15)
        *rhOffset = -(rawRHOffset * 0.1953125f); // Apply negative scaling
    } else {
        *rhOffset = rawRHOffset * 0.1953125f; // Positive scaling
    }
    // Extract T Offset (Bits 6–0) and decode
    uint8_t rawTOffset = combinedOffset & 0x7F; // 7 bits for T offset
    if (combinedOffset & 0x0080) { // T sign bit (Bit 7)
        *tOffset = -(rawTOffset * 0.1708984375f); // Apply negative scaling
    } else {
        *tOffset = rawTOffset * 0.1708984375f; // Positive scaling
    }
    return HAL_OK; // Successfully verified offsets
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
    }

    // Enable and configure the heater
    uint16_t heaterConfig = config->HEATER_VAL; // Heater configuration value
    uint8_t heaterCRC = config->HCRC;           // Heater configuration CRC

    // Send the heater configuration command
    if (HDC302x_WriteCommandWithData(senID, HDC302X_CMD_HEATER_CONFIG, heaterConfig, heaterCRC) != HAL_OK) {
        return HAL_ERROR; // Failed to configure heater
    }

    // Enable the heater
    if (HDC302x_WriteCommand(senID, HDC302X_CMD_HEATER_ENABLE) != HAL_OK) {
        return HAL_ERROR; // Failed to enable heater
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
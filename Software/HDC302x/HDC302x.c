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
 * @brief Calculate CRC-8 checksum for the HDC302x.
 *
 * This function calculates the CRC-8/NRSC-5 checksum as described in the HDC302x datasheet.
 * The CRC is used for data integrity verification when communicating with the sensor.
 *
 * @param data Pointer to a two-byte data array to be processed.
 * @return The calculated 8-bit CRC checksum.
 */
static uint8_t CalculateCRC(uint8_t *data) {
    uint8_t crc = HDC302X_CRC_INIT; // Initial value
    for (uint8_t i = 0; i < 2; i++) { // Process 2 bytes
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & HDC302X_SIGN_MASK) ? (crc << 1) ^ HDC302X_CRC_POLY : (crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Encode temperature and humidity thresholds into a 16-bit value (little-endian format).
 *
 * This function encodes humidity and temperature values into a 16-bit representation,
 * as per the HDC302x datasheet. The encoding uses:
 * - **7 most significant bits (MSBs) for humidity**
 * - **9 MSBs for temperature**
 *
 * The final encoded value is returned in **little-endian** format.
 *
 * @param threshold Pointer to the `HDC302x_Data_t` structure containing temperature and humidity values.
 * @return Encoded 16-bit threshold value (little-endian format).
 */
static uint16_t EncodeThreshold(HDC302x_Data_t *threshold) {
    uint16_t rawRH = (uint16_t)(threshold->Humidity * HDC302X_RH_COEFF_INV);
    uint16_t rawTemp = (uint16_t)(threshold->Temperature * HDC302X_TEMP_COEFF1_INV + HDC302X_TEMP_COEFF3);
    uint16_t msbRH = (rawRH >> 9) & HDC302X_HUMIDITY_MSB_MASK;
    uint16_t msbTemp = (rawTemp >> 7) & HDC302X_TEMPERATURE_MSB_MASK;
    uint16_t encoded = (msbRH << 9) | msbTemp;

    // Swap bytes to match little-endian format
    return (encoded >> 8) | (encoded << 8);
}

/**
 * @brief Decode a 16-bit threshold value into temperature and humidity values.
 *
 * This function decodes the 16-bit threshold value stored in **big-endian** format.
 * The decoding process extracts:
 * - **7 MSBs for humidity**
 * - **9 MSBs for temperature**
 *
 * The extracted values are then converted to floating-point representations
 * and stored in the provided `HDC302x_Data_t` structure.
 *
 * @param rawThreshold The raw 16-bit threshold value (big-endian format).
 * @param data Pointer to an `HDC302x_Data_t` structure where decoded temperature and humidity values will be stored.
 */
static void DecodeThreshold(uint16_t rawThreshold, HDC302x_Data_t *data) {
    uint8_t msbRH = (rawThreshold >> 9) & HDC302X_HUMIDITY_MSB_MASK;
    uint16_t msbTemp = rawThreshold & HDC302X_TEMPERATURE_MSB_MASK;

    data->Humidity = (msbRH * 100.0f) / 127.0f;
    data->Temperature = ((msbTemp * 175.0f) / 511.0f) - 45.0f;
}

/**
 * @brief Write a 16-bit command to the HDC302x sensor.
 *
 * This function transmits a 16-bit command to the sensor using I2C.
 * The command must be pre-swapped to big-endian format before calling this function.
 *
 * @param sensor_index Index of the sensor in HDC3020_Sensors (0 for address 0x44, 1 for address 0x45).
 * @param command The 16-bit command to send (must be pre-swapped to big-endian format).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef HDC302x_WriteCommand(uint8_t sensor_index, uint16_t command) {
    return HAL_I2C_Master_Transmit(&hi2c2, HDC3020_Sensors[sensor_index].Address, (uint8_t *)&command, 2, I2C_TIMEOUT);
}

/**
 * @brief Write a 16-bit command with 16-bit data and CRC to the HDC302x sensor.
 *
 * This function sends a command, followed by a 16-bit data value, and a CRC byte
 * in a single I2C transaction. The command and data must be pre-swapped to big-endian format.
 *
 * @param sensor_index Index of the sensor in HDC3020_Sensors (0 for address 0x44, 1 for address 0x45).
 * @param command The 16-bit command to send (must be pre-swapped to big-endian format).
 * @param data The 16-bit data value to send (must be pre-swapped to big-endian format).
 * @param crc The CRC-8 checksum for the data being transmitted.
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
 * @brief Read multiple 16-bit results from the HDC302x sensor and return them in little-endian format.
 *
 * This function sends a read command to the sensor and retrieves multiple 16-bit results.
 * Each 16-bit result is followed by a CRC byte for data integrity verification.
 *
 * @param sensor_index Index of the sensor in HDC3020_Sensors (0 for address 0x44, 1 for address 0x45).
 * @param command The 16-bit read command to send (must be pre-swapped to big-endian format).
 * @param results Pointer to a buffer where the retrieved 16-bit results will be stored (converted to little-endian).
 * @param result_count Number of 16-bit results to read.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR if CRC verification fails or I2C communication fails.
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
 * This function initializes the HDC302x sensor by performing the following steps:
 * - Sets the I2C address based on the sensor ID.
 * - Performs a soft reset.
 * - Reads and verifies the manufacturer ID.
 * - Clears the status register.
 * - (Optional) Configures and stores the sensor settings in NVM.
 * - (Optional) Sets and verifies offset values for temperature and humidity.
 * - (Optional) Sets and stores alert thresholds in NVM.
 * - Starts auto-measurement at 1 Hz with the lowest noise mode.
 *
 * @note Some operations, such as NVM writes, are commented out to avoid repeated EEPROM programming.
 *
 * @param senID Sensor ID (0 for 0x44, 1 for 0x45).
 * @return HAL_StatusTypeDef HAL_OK if initialization is successful, HAL_ERROR otherwise.
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
		// Use this only once as it will store the setting in NVM, mandatory delay
//		if (HDC302x_SetConfiguration(senID, &HDC302X_CONFIG_1HZ_LOWEST_NOISE) != HAL_OK) {
//			return HAL_ERROR;
//		}
//		HAL_Delay(77);
		
		float o_rh = 1.5;
		float o_t = 5.3;
		if (HDC3020_SetOffset(senID, o_rh, o_t) != HAL_OK) {
			return HAL_ERROR;
		}
//		if (HDC302x_TransferOffsetsToNVM(senID) != HAL_OK) {
//			return HAL_ERROR; // Setting alert limits failed
//		}
//		HAL_Delay(100);
//		if (HDC3020_GetOffset(senID, &o_rh, &o_t) != HAL_OK) {
//			return HAL_ERROR;
//		}
    // Step 3: Configure default alert limits
    // Default limits: RH (0% - 100%) and Temp (-40°C to 125°C)
		
//    HDC302x_Data_t highAlertValue = { .Temperature = 120.0f, .Humidity = 2.0f };
//		HDC302x_Data_t lowAlertValue = { .Temperature = 120.0f, .Humidity = 2.0f };
//		HDC302x_Data_t highAlertClear = { .Temperature = 120.0f, .Humidity = 2.0f };
//		HDC302x_Data_t lowAlertClear = { .Temperature = 120.0f, .Humidity = 2.0f };
//		
//    if (HDC302x_SetAlertLimits(senID, highAlertValue, lowAlertValue, highAlertClear, lowAlertClear) != HAL_OK) {
//        return HAL_ERROR; // Setting alert limits failed
//    }
//		if (HDC302x_GetAlertLimits(senID, &highAlertValue, &lowAlertValue, &highAlertClear, &lowAlertClear) != HAL_OK) {
//			return HAL_ERROR; // Setting alert limits failed
//		}
//		if (HDC302x_TransferAlertLimitsToNVM(senID) != HAL_OK) {
//			return HAL_ERROR; // Setting alert limits failed
//		}
//		HAL_Delay(77);
		
    // Step 4: Start auto-measurement mode with the lowest noise at 1 Hz
    if (HDC3020_SelectMeasurementMode(senID, HDC302X_CMD_MEASURE_01_LPM0) != HAL_OK) {
        return HAL_ERROR; // Starting auto-measurement mode failed
    }
    return HAL_OK; // Initialization successful
}


/**
 * @brief Perform a soft reset on the HDC302x sensor.
 *
 * This function sends a soft reset command to the sensor, allowing it to restart 
 * and reinitialize its internal state. A delay is included to ensure the sensor 
 * has sufficient time to reset before further communication.
 *
 * @param senID Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @return HAL_StatusTypeDef HAL_OK if the reset is successful, HAL_ERROR otherwise.
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
 * @brief Configure the HDC302x sensor.
 *
 * This function writes a configuration value and its corresponding CRC to the sensor 
 * to set its operating mode. The configuration is stored in the sensor's volatile 
 * memory unless explicitly written to NVM.
 *
 * @note To save the configuration in non-volatile memory (NVM), use the 
 * `HDC302X_CMD_PROGRAM_READ_DEFAULT_STATE` command.
 *
 * @param senID Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @param config Pointer to the `HDC302x_Config_t` structure containing the configuration value and CRC.
 * @return HAL_StatusTypeDef HAL_OK if the configuration is set successfully, HAL_ERROR otherwise.
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
 * @brief Read the status register of the HDC3020 sensor.
 *
 * This function sends the appropriate command to the sensor and reads 
 * the status register value, storing it in the sensor's data structure.
 *
 * @param senID Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC3020_ReadStatusRegister(uint8_t senID) {
    // Read the status register using HDC302x_ReadMultipleResults with result_count = 1
		return HDC302x_ReadCmdResults(senID, HDC302X_READ_STATUS, &HDC3020_Sensors[senID].Status.Val.Value, 1);// Single result
}

/**
 * @brief Clear the status register of the HDC3020 sensor.
 *
 * This function sends the `HDC302X_CLEAR_STATUS` command to reset 
 * the status register and clears the stored status value in the 
 * corresponding sensor structure.
 *
 * @param sensor_index Sensor ID (0 for address 0x44, 1 for address 0x45).
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
 * @brief Read temperature and humidity data from the HDC302x sensor.
 *
 * This function reads the raw 16-bit temperature and humidity values from the sensor,
 * converts them into floating-point values using the appropriate scaling factors, 
 * and stores them in the provided `HDC302x_Data_t` structure.
 *
 * @param sensor_index Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @param data Pointer to an `HDC302x_Data_t` structure to store the converted temperature (°C) and humidity (%RH).
 * @return HAL_StatusTypeDef HAL_OK if the data is read and converted successfully, HAL_ERROR otherwise.
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
 * @brief Set the measurement mode for the HDC302x sensor.
 *
 * This function configures the sensor's measurement mode by sending the specified 
 * command. The measurement mode determines the sampling rate and power consumption.
 *
 * @param senID Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @param command The 16-bit command representing the desired measurement mode (must be pre-swapped to big-endian format).
 * @return HAL_StatusTypeDef HAL_OK if the mode is set successfully, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC3020_SelectMeasurementMode(uint8_t senID, uint16_t command) {
    // Return the result of the command transmission
    return HDC302x_WriteCommand(senID, command);
}

/**
 * @brief Retrieve the measurement history from the HDC302x sensor.
 *
 * This function reads the minimum and maximum recorded temperature and humidity values 
 * from the sensor and stores them in the provided `HDC302x_History_t` structure.
 *
 * @param senID Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @param history Pointer to an `HDC302x_History_t` structure to store the measurement history.
 * @return HAL_StatusTypeDef HAL_OK if the history data is successfully retrieved, HAL_ERROR otherwise.
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
 * @brief Store alert threshold limits in the HDC302x sensor's non-volatile memory (NVM).
 *
 * This function writes the currently configured alert limits to the sensor's NVM, ensuring 
 * they persist across power cycles.
 *
 * @note NVM writes should be used sparingly to avoid exceeding EEPROM endurance limits.
 *
 * @param senID Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @return HAL_StatusTypeDef HAL_OK if the operation is successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC302x_TransferAlertLimitsToNVM(uint8_t senID) {
    if (HDC302x_WriteCommand(senID, HDC302X_CMD_PROGRAM_ALERT_THRESHOLDS_TO_NVM) != HAL_OK) {
        return HAL_ERROR; // Return the error if the operation failed
    }
		return HAL_OK;
}

/**
 * @brief Set the alert limits for temperature and humidity on the HDC302x sensor.
 *
 * This function configures the high and low alert thresholds, as well as their 
 * respective clear values, for temperature and humidity. These values determine 
 * when alerts are triggered and reset.
 *
 * @param senID Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @param highAlertValue High alert threshold values for temperature (°C) and humidity (%RH).
 * @param lowAlertValue Low alert threshold values for temperature (°C) and humidity (%RH).
 * @param highAlertClear High alert clear values for temperature (°C) and humidity (%RH).
 * @param lowAlertClear Low alert clear values for temperature (°C) and humidity (%RH).
 * @return HAL_StatusTypeDef HAL_OK if the thresholds are successfully set, HAL_ERROR otherwise.
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
 * @brief Get the current alert limits for temperature and humidity from the HDC302x sensor.
 *
 * This function reads the high and low alert thresholds, as well as their 
 * respective clear values, for temperature and humidity from the sensor.
 * The retrieved values are decoded and stored in the provided structures.
 *
 * @param senID Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @param highAlertValue Pointer to store high alert threshold values for temperature (°C) and humidity (%RH).
 * @param lowAlertValue Pointer to store low alert threshold values for temperature (°C) and humidity (%RH).
 * @param highAlertClear Pointer to store high alert clear values for temperature (°C) and humidity (%RH).
 * @param lowAlertClear Pointer to store low alert clear values for temperature (°C) and humidity (%RH).
 * @return HAL_StatusTypeDef HAL_OK if the thresholds are successfully retrieved, HAL_ERROR otherwise.
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
 * @brief Store the programmed temperature and humidity offset values into the HDC302x sensor's non-volatile memory (NVM).
 *
 * This function writes the offset calibration values to the sensor’s EEPROM, ensuring they persist across power cycles.
 *
 * @note NVM writes should be used sparingly to avoid exceeding EEPROM endurance limits.
 *
 * @param senID Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @return HAL_StatusTypeDef HAL_OK if the operation is successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC302x_TransferOffsetsToNVM(uint8_t senID) {
    if (HDC302x_WriteCommand(senID, HDC302X_CMD_PROGRAM_READ_OFFSET_VALUES) != HAL_OK) {
        return HAL_ERROR; // Return the error if the operation failed
    }
		return HAL_OK;
}


/**
 * @brief Set the temperature and humidity offset values for the HDC302x sensor.
 *
 * This function encodes the offset values for temperature and humidity and programs them into the sensor.
 * The values are converted into an appropriate 16-bit format as per the HDC302x datasheet.
 *
 * @note The offsets must be written to NVM separately using `HDC302x_TransferOffsetsToNVM()` to persist after a reset.
 *
 * @param senID Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @param RH_Offset Offset for relative humidity in percentage (%RH), e.g., +8.2% or -7.8%.
 * @param T_Offset Offset for temperature in degrees Celsius (°C), e.g., +7.1°C or -10.9°C.
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
    uint16_t combinedOffset = ((hCode << 1) & 0x7F00) | ((tCode >> 6) & HDC302X_HUMIDITY_MSB_MASK) | CodeMask;

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
 * @brief Retrieve and decode the programmed temperature and humidity offset values from the HDC302x sensor.
 *
 * This function reads the offset values stored in the sensor, decodes them back into floating-point
 * temperature and humidity offsets, and stores them in the provided variables.
 *
 * @param senID Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @param rhOffset Pointer to store the retrieved humidity offset (%RH).
 * @param tOffset Pointer to store the retrieved temperature offset (°C).
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
    uint8_t RH_offset = (combinedOffset >> 8) & HDC302X_CRC_INIT;  // High byte
    uint8_t T_offset  = combinedOffset & HDC302X_CRC_INIT;         // Low byte

    printf("Extracted RH Offset Byte: 0x%02X, Extracted T Offset Byte: 0x%02X\n", RH_offset, T_offset);

    // Extract sign bits (Bit 7)
    uint8_t RH_negative = !(RH_offset & HDC302X_SIGN_MASK);
    uint8_t T_negative  = !(T_offset & HDC302X_SIGN_MASK);

    // Remove sign bits
    RH_offset &= HDC302X_HUMIDITY_MSB_MASK;
    T_offset  &= HDC302X_HUMIDITY_MSB_MASK;

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
 * If the heater configuration is `HDC302X_HEATER_OFF`, the heater will be disabled.
 * Otherwise, the function will send commands to enable and configure the heater.
 *
 * @param senID Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @param config Pointer to the `HDC302x_HeaterConfig_t` structure containing the heater configuration.
 *               Use `HDC302X_HEATER_OFF` to disable the heater.
 * @return HAL_StatusTypeDef HAL_OK if the operation is successful, HAL_ERROR otherwise.
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
 * @brief Read the Manufacturer ID from the HDC302x sensor.
 *
 * This function sends the command to read the 16-bit Manufacturer ID and retrieves the value.
 * The Manufacturer ID can be used to verify communication with the sensor.
 *
 * @param senID Sensor ID (0 for address 0x44, 1 for address 0x45).
 * @param manufacturerID Pointer to store the retrieved 16-bit Manufacturer ID.
 * @return HAL_StatusTypeDef HAL_OK if the Manufacturer ID is successfully read, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC302x_ReadManufacturerID(uint8_t senID, uint16_t *manufacturerID) {
    return HDC302x_ReadCmdResults(senID, HDC302X_CMD_READ_MANUFACTURER_ID, manufacturerID, 1);// Single result
}

/**
 * @brief Calculate the dew point based on temperature and humidity.
 *
 * This function calculates the dew point temperature using the **Magnus-Tetens** formula.
 *
 * @param temperature Temperature in degrees Celsius (°C).
 * @param humidity Relative humidity in percentage (%RH).
 * @return float Dew point temperature in degrees Celsius (°C).
 */
float CalculateDewPoint(float temperature, float humidity) {
    float alpha = (DEW_POINT_CONST_A * temperature) / (DEW_POINT_CONST_B + temperature) + logf(humidity / 100.0f);
    float dew_point = (DEW_POINT_CONST_B * alpha) / (DEW_POINT_CONST_A - alpha);
    return dew_point;
}
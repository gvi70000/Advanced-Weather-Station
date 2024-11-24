#include "ENS160.h"
#include "i2c.h"

// Initialize the ENS160 sensor
HAL_StatusTypeDef ENS160_Init(void) {
    uint16_t part_id = 0;

    // Step 1: Read the PART ID to verify the sensor
    if (ReadRegister(ENS160_I2C_ADDRESS, ENS160_PART_ID_ADDR, (uint8_t *)&part_id, ENS160_PART_ID_SIZE, &hi2c2) != HAL_OK) {
        return HAL_ERROR;
    }

    if (part_id != 0x0160) { // Ensure the PART ID matches the expected value
        return HAL_ERROR;
    }

    // Step 2: Set the operating mode to IDLE for initial configuration
    if (ENS160_SetOperatingMode(ENS160_OPMODE_IDLE) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 3: Configure the interrupt (CONFIG register at 0x11)
    // 0x63 = Active high interrupt, Push/Pull, Assert on new data ready in DATA registers
    uint8_t config_value = 0x63;
    if (WriteRegister(ENS160_I2C_ADDRESS, ENS160_CONFIG_ADDR, &config_value, 1, &hi2c2) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 4: Set the operating mode to STANDARD Gas Sensing Mode
    if (ENS160_SetOperatingMode(ENS160_OPMODE_STANDARD) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK; // Initialization successful
}

HAL_StatusTypeDef ENS160_UpdateEnvInputs(float temperatureC, float humidityPercent) {
    HAL_StatusTypeDef status;
    uint16_t temperatureKelvinScaled;
    uint16_t humidityScaled;

    // Convert temperature from Celsius to Kelvin, then scale by 64
    temperatureKelvinScaled = (uint16_t)((temperatureC + 273.15f) * 64.0f);

    // Convert relative humidity to required format (RH % × 512)
    humidityScaled = (uint16_t)(humidityPercent * 512.0f);

    // Write temperature to the sensor (LSB first, then MSB)
    status = ENS160_WriteRegister(ENS160_TEMP_IN_ADDR, (uint8_t *)&temperatureKelvinScaled, ENS160_TEMP_IN_SIZE);
    if (status != HAL_OK) {
        return status; // Return if write fails
    }

    // Write relative humidity to the sensor
    status = ENS160_WriteRegister(ENS160_RH_IN_ADDR, (uint8_t *)&humidityScaled, ENS160_RH_IN_SIZE);
    if (status != HAL_OK) {
        return status; // Return if write fails
    }

    return HAL_OK;
}

// Function to read all air quality data in one I2C transaction
HAL_StatusTypeDef ENS160_ReadAllData(ENS160_Data_t *data) {
    uint8_t raw_data[5]; // Buffer to hold AQI (1 byte) + TVOC (2 bytes) + eCO2 (2 bytes)

    // Read all relevant data starting from the AQI register
    HAL_StatusTypeDef status = ReadRegister(ENS160_I2C_ADDRESS, ENS160_DATA_AQI_ADDR, raw_data, sizeof(raw_data), &hi2c2);
    if (status != HAL_OK) {
        return status; // Return error if the read operation fails
    }

    // Parse raw data into structured fields
    data->AQI = raw_data[0];                       // AQI: 1 byte
    data->TVOC = (raw_data[1] | (raw_data[2] << 8)); // TVOC: 2 bytes (little-endian)
    data->eCO2 = (raw_data[3] | (raw_data[4] << 8)); // eCO2: 2 bytes (little-endian)

    return HAL_OK; // Return success
}

// Function to read all registers into a structure
HAL_StatusTypeDef ENS160_ReadAllRegisters(ENS160_Registers_t *registers) {
    return ReadRegister(ENS160_I2C_ADDRESS, ENS160_PART_ID_ADDR, (uint8_t *)registers, sizeof(ENS160_Registers_t), &hi2c2);
}

// Function to write to a specific register
HAL_StatusTypeDef ENS160_WriteRegister(uint8_t reg, uint8_t *pData, uint16_t size) {
    return WriteRegister(ENS160_I2C_ADDRESS, reg, pData, size, &hi2c2);
}

// Function to set the operating mode
HAL_StatusTypeDef ENS160_SetOperatingMode(ENS160_OPMODE_t mode) {
    return ENS160_WriteRegister(ENS160_OPMODE_ADDR, (uint8_t *)&mode, ENS160_OPMODE_SIZE);
}

// Function to read AQI value
HAL_StatusTypeDef ENS160_ReadAQI(ENS160_AQI_t *aqi) {
    return ReadRegister(ENS160_I2C_ADDRESS, ENS160_DATA_AQI_ADDR, (uint8_t *)aqi, ENS160_DATA_AQI_SIZE, &hi2c2);
}

// Function to read TVOC concentration
HAL_StatusTypeDef ENS160_ReadTVOC(uint16_t *tvoc) {
    return ReadRegister(ENS160_I2C_ADDRESS, ENS160_DATA_TVOC_ADDR, (uint8_t *)tvoc, ENS160_DATA_TVOC_SIZE, &hi2c2);
}

// Function to read CO2 concentration
HAL_StatusTypeDef ENS160_ReadCO2(uint16_t *eco2) {
    return ReadRegister(ENS160_I2C_ADDRESS, ENS160_DATA_ECO2_ADDR, (uint8_t *)eco2, ENS160_DATA_ECO2_SIZE, &hi2c2);
}

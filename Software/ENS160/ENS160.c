#include "ENS160.h"
#include "i2c.h"

// External declaration for I2C handle
extern I2C_HandleTypeDef hi2c2;

// Global instance of the register structure
static ENS160_Registers_t ENS160_REG;

/**
 * @brief Initialize the ENS160 sensor.
 * 
 * This function performs the initialization of the ENS160 sensor by:
 * - Verifying the sensor's PART ID.
 * - Setting the operating mode to IDLE.
 * - Configuring the interrupt register.
 * - Setting the operating mode to STANDARD Gas Sensing Mode.
 * 
 * @return HAL_StatusTypeDef HAL_OK if initialization is successful, HAL_ERROR otherwise.
 */
/**
 * @brief Initialize the ENS160 sensor.
 */
HAL_StatusTypeDef ENS160_Init(void) {
    // Step 1: Verify PART ID
    if (ReadRegister(ENS160_I2C_ADDRESS, ENS160_PART_ID_ADDR, (uint8_t *)&ENS160_REG.PART_ID, ENS160_PART_ID_SIZE, &hi2c2) != HAL_OK) {
        return HAL_ERROR;
    }

    if (ENS160_REG.PART_ID != ENS160_PART_ID) {
        return HAL_ERROR;
    }

    // Step 2: Set to IDLE mode for configuration
    if (ENS160_SetOperatingMode(ENS160_OPMODE_IDLE) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 3: Configure interrupt settings
    ENS160_REG.CONFIG.INTEN		= 1;
    ENS160_REG.CONFIG.INTDAT	= 1;
    ENS160_REG.CONFIG.INTGPR	= 0;
    ENS160_REG.CONFIG.INT_CFG	= 1;
    ENS160_REG.CONFIG.INTPOL	= 1;

    if (WriteRegister(ENS160_I2C_ADDRESS, ENS160_CONFIG_ADDR, (uint8_t *)&ENS160_REG.CONFIG, ENS160_CONFIG_SIZE, &hi2c2) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 4: Switch to STANDARD sensing mode
    if (ENS160_SetOperatingMode(ENS160_OPMODE_STANDARD) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Update temperature and humidity inputs for ENS160.
 */
HAL_StatusTypeDef ENS160_UpdateEnvInputs(float temperatureC, float humidityPercent) {
    // Convert temperature and humidity
    ENS160_REG.TEMP_IN = (uint16_t)((temperatureC + 273.15f) * 64.0f);
    ENS160_REG.RH_IN = (uint16_t)(humidityPercent * 512.0f);

    // Write temperature to TEMP_IN
    if (WriteRegister(ENS160_I2C_ADDRESS, ENS160_TEMP_IN_ADDR, (uint8_t *)&ENS160_REG.TEMP_IN, ENS160_TEMP_IN_SIZE, &hi2c2) != HAL_OK) {
        return HAL_ERROR;
    }

    // Write humidity to RH_IN
    if (WriteRegister(ENS160_I2C_ADDRESS, ENS160_RH_IN_ADDR, (uint8_t *)&ENS160_REG.RH_IN, ENS160_RH_IN_SIZE, &hi2c2) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Read all air quality data.
 */
HAL_StatusTypeDef ENS160_ReadAllData(ENS160_Data_t *data) {
    return ReadRegister(ENS160_I2C_ADDRESS, ENS160_DATA_AQI_ADDR, data->Buffer, sizeof(data->Buffer), &hi2c2);
}

/**
 * @brief Read all ENS160 registers.
 */
HAL_StatusTypeDef ENS160_ReadAllRegisters(ENS160_Registers_t *registers) {
    return ReadRegister(ENS160_I2C_ADDRESS, ENS160_PART_ID_ADDR, (uint8_t *)registers, sizeof(ENS160_Registers_t), &hi2c2);
}

/**
 * @brief Set operating mode for ENS160.
 */
HAL_StatusTypeDef ENS160_SetOperatingMode(ENS160_OPMODE_t mode) {
    return WriteRegister(ENS160_I2C_ADDRESS, ENS160_OPMODE_ADDR, (uint8_t *)&mode, ENS160_OPMODE_SIZE, &hi2c2);
}

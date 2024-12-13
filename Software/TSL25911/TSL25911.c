#include "TSL25911.h"
#include "i2c.h"

/** 
 * @brief External declaration for the I2C handle.
 * The handle is used for I2C communication with the TSL2591 sensor.
 */
extern I2C_HandleTypeDef hi2c2;

/**
 * @brief Static instance of TSL2591 register structure.
 * This structure holds the configuration and status data for the sensor.
 */
static TSL25911_REGISTERS_t TSL25911_Sensor;

// Write to a register of TSL2591
static inline HAL_StatusTypeDef TSL25911_WriteRegister(uint8_t reg, uint8_t* data, uint8_t len) {
    return WriteRegister(TSL25911_I2C_ADDR, reg, data, len, &hi2c2);
}

// Read from a register of TSL2591
static inline HAL_StatusTypeDef TSL25911_ReadRegister(uint8_t reg, uint8_t* data, uint8_t len) {
    return ReadRegister(TSL25911_I2C_ADDR, reg, data, len, &hi2c2);
}

/**
 * @brief Initializes the TSL25911 sensor.
 * 
 * This function follows the initialization sequence:
 * 1. Reads and validates the chip ID.
 * 2. Powers on the oscillator by setting the PON bit.
 * 3. Configures gain and integration time.
 * 4. Configures ALS interrupt thresholds.
 * 5. Enables ALS and ALS interrupts.
 * 
 * @return HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef TSL25911_Init(void) {
    // Step 1: Get and validate chip ID
    if (TSL25911_ReadID() != HAL_OK || TSL25911_Sensor.ID != TSL25911_ID) {
        return HAL_ERROR; // Invalid chip ID
    }

    // Step 2: Power ON the oscillator by setting the PON bit
    if (TSL25911_Enable(TSL25911_STATE_PON) != HAL_OK) {
        return HAL_ERROR; // Failed to set PON
    }

    // Step 3: Configure gain and integration time
    if (TSL25911_SetGainAndIntegrationTime(TSL25911_GAIN_LOW, TSL25911_INTEGRATION_600MS) != HAL_OK) {
        return HAL_ERROR; // Failed to set gain and integration time
    }

    // Step 5: Configure ALS interrupt thresholds
    if (TSL25911_SetInterruptThresholds(TSL25911_SET_MID, TSL25911_SET_MID) != HAL_OK) {
        return HAL_ERROR; // Failed to set interrupt thresholds
    }

    // Step 6: Enable ALS and ALS interrupts
		TSL25911_Sensor.ENABLE.Val.BitField.PON = 1;
		TSL25911_Sensor.ENABLE.Val.BitField.AEN = 1;
		TSL25911_Sensor.ENABLE.Val.BitField.AIEN = 1;

    if (TSL25911_Enable(TSL25911_STATE_PON_AEN_AIEN) != HAL_OK) {
        return HAL_ERROR; // Failed to set PON
    }

    return HAL_OK; // Initialization successful
}

/**
 * @brief Enables or disables the TSL2591 sensor.
 * 
 * @param state Desired state for the sensor (e.g., TSL25911_STATE_DISABLED or TSL25911_STATE_ENABLE).
 * @return HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef TSL25911_Enable(TSL25911_STATE_t state) {
    TSL25911_Sensor.ENABLE.Val.Value = state;
    return TSL25911_WriteRegister(TSL25911_REG_ENABLE, &TSL25911_Sensor.ENABLE.Val.Value, 1);
}

/**
 * @brief Resets the TSL2591 sensor.
 * 
 * This function sets the SRESET bit in the CONTROL register to reset the sensor.
 * @return HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef TSL25911_Reset(void) {
    TSL25911_Sensor.CTRL.Val.BitField.SRESET = 1;
    return TSL25911_WriteRegister(TSL25911_REG_CONTROL, &TSL25911_Sensor.CTRL.Val.Value, 1);
}

/**
 * @brief Sets the gain and integration time for the TSL2591 sensor.
 * 
 * @param gain Desired gain setting.
 * @param integrationTime Desired integration time setting.
 * @return HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef TSL25911_SetGainAndIntegrationTime(TSL25911_GAIN_t gain, TSL25911_INTEGRATION_t integrationTime) {
    TSL25911_Sensor.CTRL.Val.BitField.AGAIN = gain;
    TSL25911_Sensor.CTRL.Val.BitField.ATIME = integrationTime;
    return TSL25911_WriteRegister(TSL25911_REG_CONTROL, &TSL25911_Sensor.CTRL.Val.Value, 1);
}

/**
 * @brief Configures the ALS interrupt thresholds.
 * 
 * @param low Low threshold value.
 * @param high High threshold value.
 * @return HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef TSL25911_SetInterruptThresholds(uint16_t low, uint16_t high) {
    TSL25911_Sensor.ALS.LowThreshold.Value = low;
    TSL25911_Sensor.ALS.HighThreshold.Value = high;

    // Write thresholds to the sensor
    HAL_StatusTypeDef status = TSL25911_WriteRegister(TSL25911_REG_AILTL, TSL25911_Sensor.ALS.FullArray, 4);
    if (status != HAL_OK) return status;

    // Set No Persist Interrupt Thresholds
    return TSL25911_WriteRegister(TSL25911_REG_NPAILTL, TSL25911_Sensor.ALS.FullArray, 4);
}

/**
 * @brief Sets the persistence filter for ALS interrupts.
 * 
 * @param persistenceValue Desired persistence value.
 * @return HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef TSL25911_SetPersistence(TSL25911_PERSIST_t persistenceValue) {
    TSL25911_Sensor.PERSIST = persistenceValue;
    return TSL25911_WriteRegister(TSL25911_REG_PERSIST, &TSL25911_Sensor.PERSIST, 1);
}

/**
 * @brief Reads the device ID of the TSL2591 sensor.
 * 
 * @return HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef TSL25911_ReadID(void) {
    return TSL25911_ReadRegister(TSL25911_REG_ID, &TSL25911_Sensor.ID, 1);
}

/**
 * @brief Reads the status register of the TSL2591 sensor.
 * 
 * @return HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef TSL25911_GetStatus(void) {
    return TSL25911_ReadRegister(TSL25911_REG_STATUS, &TSL25911_Sensor.STATUS.Val.Value, 1);
}

/**
 * @brief Clears an interrupt on the TSL2591 sensor.
 * 
 * @param interrupt Type of interrupt to clear (special function command).
 * @return HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef TSL25911_ClearInterrupt(TSL25911_SpecialFunction_t interrupt) {
    return HAL_I2C_Master_Transmit(&hi2c2, TSL25911_I2C_ADDR, (uint8_t*)&interrupt, 1, HAL_MAX_DELAY);
}

/**
 * @brief Reads light data from the TSL2591 sensor.
 * 
 * This function performs the following steps:
 * 1. Reads the status register to check the validity of the data using TSL25911_GetStatus.
 * 2. Reads the raw light data directly into the mapped data structure using ReadRegister.
 * 3. Maps the raw data to the light data structure.
 * 4. Calculates the lux value based on the raw data, gain, and integration time.
 * 5. Clears the ALS interrupt if it is set.
 * 
 * @param lightData Pointer to a structure to store the light data.
 * @return HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef TSL25911_ReadLightData(TSL25911_LightData_t *lightData) {
    // Step 1: Read the status register to verify data validity using TSL25911_GetStatus
    if (TSL25911_GetStatus() != HAL_OK) {
        return HAL_ERROR; // Status read operation failed
    }

    if (!TSL25911_Sensor.STATUS.Val.BitField.AVALID) {
        return HAL_ERROR; // Data not valid
    }

    // Step 2: Read raw light data directly into the mapped data structure using ReadRegister
    if (TSL25911_ReadRegister(TSL25911_REG_C0DATA, TSL25911_Sensor.DATA.Array, sizeof(TSL25911_Sensor.DATA.Array)) != HAL_OK) {
        return HAL_ERROR; // I2C read operation failed
    }

    // Step 3: Map raw data to the light data structure
    lightData->FullSpectrum = TSL25911_Sensor.DATA.Channels.CH0.Value;
    lightData->Infrared = TSL25911_Sensor.DATA.Channels.CH1.Value;
    lightData->Visible = lightData->FullSpectrum - lightData->Infrared;

    // Step 4: Calculate lux based on integration time, gain, and raw data
    float atime, again, cpl;
    switch (TSL25911_Sensor.CTRL.Val.BitField.ATIME) {
        case TSL25911_INTEGRATION_100MS: atime = 100.0F; break;
        case TSL25911_INTEGRATION_200MS: atime = 200.0F; break;
        case TSL25911_INTEGRATION_300MS: atime = 300.0F; break;
        case TSL25911_INTEGRATION_400MS: atime = 400.0F; break;
        case TSL25911_INTEGRATION_500MS: atime = 500.0F; break;
        case TSL25911_INTEGRATION_600MS: atime = 600.0F; break;
        default: atime = 100.0F; break;
    }

    switch (TSL25911_Sensor.CTRL.Val.BitField.AGAIN) {
        case TSL25911_GAIN_LOW: again = 1.0F; break;
        case TSL25911_GAIN_MED: again = 25.0F; break;
        case TSL25911_GAIN_HIGH: again = 428.0F; break;
        case TSL25911_GAIN_MAX: again = 9876.0F; break;
        default: again = 1.0F; break;
    }

    if (lightData->FullSpectrum == 0xFFFF || lightData->Infrared == 0xFFFF) {
        return HAL_ERROR; // Overflow detected
    }

    cpl = (atime * again) / TSL25911_LUX_DF;
    lightData->Lux = (((float)lightData->Visible) * (1.0F - ((float)lightData->Infrared / (float)lightData->FullSpectrum))) / cpl;

    // Step 5: Clear the ALS interrupt if it is set
		if (TSL25911_ClearInterrupt(TSL25911_SPECIAL_FUNCTION_CLEAR_ALS_INT) != HAL_OK) {
			return HAL_ERROR; // Failed to clear interrupt
		}

    return HAL_OK;
}



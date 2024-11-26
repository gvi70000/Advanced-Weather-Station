#include "TSL25911.h"
#include "i2c.h"

// External declaration for I2C handle
extern I2C_HandleTypeDef hi2c2;

static TSL25911_REGISTERS_t tsl25911_registers;

/**
 * @brief Initializes the TSL25911 light sensor.
 * @return HAL_StatusTypeDef Returns HAL_OK on success or HAL_ERROR on failure.
 */
HAL_StatusTypeDef TSL25911_Init(void) {
    uint8_t id;

    // Step 1: Read and validate the device ID
    if (TSL25911_ReadID(&id) != HAL_OK || id != TSL25911_ID) {
        return HAL_ERROR; // Device not detected or invalid ID
    }
    tsl25911_registers.ID = id;

    // Step 2: Enable the sensor
    tsl25911_registers.EN.Val.Value = 0; // Clear the enable register
    tsl25911_registers.EN.Val.BitField.PON = 1; // Power ON
    tsl25911_registers.EN.Val.BitField.AEN = 1; // ALS Enable
    tsl25911_registers.EN.Val.BitField.NPIEN = 1; // Enable No Persist Interrupt
    if (TSL25911_Enable() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 3: Set gain and integration time
    if (TSL25911_SetGainAndIntegrationTime(TSL25911_GAIN_LOW, TSL25911_INTEGRATION_600MS) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 4: Configure ALS interrupt thresholds to a midpoint value
    if (TSL25911_SetInterruptThresholds(TSL25911_SET_MID, TSL25911_SET_MID) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 5: Configure No Persist ALS interrupt thresholds to wide range
    if (TSL25911_SetNoPersistInterruptThresholds(TSL25911_SET_MID, TSL25911_SET_MID) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Enables the TSL25911 light sensor.
 * @return HAL_StatusTypeDef Returns HAL_OK on success or HAL_ERROR on failure.
 */
HAL_StatusTypeDef TSL25911_Enable(void) {
    return WriteRegister(TSL25911_I2C_ADDR, TSL25911_REG_ENABLE, &tsl25911_registers.EN.Val.Value, 1, &hi2c2);
}

/**
 * @brief Disables the TSL25911 light sensor.
 * @return HAL_StatusTypeDef Returns HAL_OK on success or HAL_ERROR on failure.
 */
HAL_StatusTypeDef TSL25911_Disable(void) {
    tsl25911_registers.EN.Val.BitField.PON = 0;
    tsl25911_registers.EN.Val.BitField.AEN = 0;
    return WriteRegister(TSL25911_I2C_ADDR, TSL25911_REG_ENABLE, &tsl25911_registers.EN.Val.Value, 1, &hi2c2);
}

/**
 * @brief Configures the gain and integration time of the TSL25911 sensor.
 * @param[in] gain The desired gain setting (low, medium, high, or max).
 * @param[in] integrationTime The desired integration time (100ms to 600ms).
 * @return HAL_StatusTypeDef Returns HAL_OK on success or HAL_ERROR on failure.
 */
HAL_StatusTypeDef TSL25911_SetGainAndIntegrationTime(TSL25911_GAIN_t gain, TSL25911_INTEGRATION_t integrationTime) {
    tsl25911_registers.CTRL.Val.Value = 0; // Clear the control register
    tsl25911_registers.CTRL.Val.BitField.AGAIN = gain; // Set gain
    tsl25911_registers.CTRL.Val.BitField.ATIME = integrationTime; // Set integration time

    return WriteRegister(TSL25911_I2C_ADDR, TSL25911_REG_CONTROL, &tsl25911_registers.CTRL.Val.Value, 1, &hi2c2);
}

/**
 * @brief Reads the light data from the TSL25911 sensor.
 * @param[out] lightData Pointer to TSL25911_LightData_t structure to store the light data.
 * @return HAL_StatusTypeDef Returns HAL_OK on success or HAL_ERROR on failure.
 */
HAL_StatusTypeDef TSL25911_ReadLightData(TSL25911_LightData_t *lightData) {
    // Read 4 bytes directly into the DATA.Array field
    if (ReadRegister(TSL25911_I2C_ADDR, TSL25911_REG_C0DATA, tsl25911_registers.DATA.Array, 4, &hi2c2) != HAL_OK) {
        return HAL_ERROR; // Read operation failed
    }

    // Map the raw data to the light data structure
    lightData->FullSpectrum = tsl25911_registers.DATA.CH0.Value;
    lightData->Infrared = tsl25911_registers.DATA.CH1.Value;
    lightData->Visible = lightData->FullSpectrum - lightData->Infrared;
    lightData->Lux = TSL25911_CalculateLux(); // Calculate lux value

    return HAL_OK;
}

/**
 * @brief Reads the Package ID (PID) of the TSL25911 sensor directly into the register structure.
 * 
 * The PID is useful for identifying the sensor variant and ensuring the correct device is being used.
 * 
 * @return HAL_StatusTypeDef Returns HAL_OK on success or HAL_ERROR on failure.
 */
HAL_StatusTypeDef TSL25911_ReadPID(void) {
    // Read the PID register (0x00) directly into the PID field of the register structure
    if (ReadRegister(TSL25911_I2C_ADDR, TSL25911_REG_PID, &tsl25911_registers.PID.Val.Value, 1, &hi2c2) != HAL_OK) {
        return HAL_ERROR; // Failed to read PID
    }

    return HAL_OK;
}

/**
 * @brief Reads the device ID of the TSL25911 sensor.
 * @param[out] id Pointer to store the retrieved device ID.
 * @return HAL_StatusTypeDef Returns HAL_OK on success or HAL_ERROR on failure.
 */
HAL_StatusTypeDef TSL25911_ReadID(uint8_t *id) {
    if (ReadRegister(TSL25911_I2C_ADDR, TSL25911_REG_ID, id, 1, &hi2c2) != HAL_OK) {
        return HAL_ERROR;
    }
    tsl25911_registers.ID = *id;
    return HAL_OK;
}

/**
 * @brief Configures ALS interrupt thresholds.
 * @param[in] low The low threshold value.
 * @param[in] high The high threshold value.
 * @return HAL_StatusTypeDef Returns HAL_OK on success or HAL_ERROR on failure.
 */
HAL_StatusTypeDef TSL25911_SetInterruptThresholds(uint16_t low, uint16_t high) {
    tsl25911_registers.INTP.LowThreshold.Value = low;
    tsl25911_registers.INTP.HighThreshold.Value = high;

    return WriteRegister(TSL25911_I2C_ADDR, TSL25911_REG_AILTL, tsl25911_registers.INTP.FullArray, 4, &hi2c2);
}

/**
 * @brief Configures no-persist ALS interrupt thresholds.
 * @param[in] low The low threshold value.
 * @param[in] high The high threshold value.
 * @return HAL_StatusTypeDef Returns HAL_OK on success or HAL_ERROR on failure.
 */
HAL_StatusTypeDef TSL25911_SetNoPersistInterruptThresholds(uint16_t low, uint16_t high) {
    tsl25911_registers.INTNP.LowThreshold.Value = low;
    tsl25911_registers.INTNP.HighThreshold.Value = high;

    return WriteRegister(TSL25911_I2C_ADDR, TSL25911_REG_AILTL, tsl25911_registers.INTNP.FullArray, 4, &hi2c2);
}

/**
 * @brief Calculates the lux value from raw light data.
 * @return float The calculated lux value. Returns -1.0 if an overflow condition is detected.
 */
float TSL25911_CalculateLux(void) {
    uint16_t ch0 = tsl25911_registers.DATA.CH0.Value; // Full-spectrum channel
    uint16_t ch1 = tsl25911_registers.DATA.CH1.Value; // Infrared channel

    // Retrieve integration time and gain from the control register
    TSL25911_INTEGRATION_t integration = (TSL25911_INTEGRATION_t)tsl25911_registers.CTRL.Val.BitField.ATIME;
    TSL25911_GAIN_t gain = (TSL25911_GAIN_t)tsl25911_registers.CTRL.Val.BitField.AGAIN;

    // Convert integration time to milliseconds
    float atime = (256 - integration) * 2.73;

    // Convert gain to numeric value
    float again;
    switch (gain) {
        case TSL25911_GAIN_LOW: again = 1.0F; break;
        case TSL25911_GAIN_MED: again = 25.0F; break;
        case TSL25911_GAIN_HIGH: again = 428.0F; break;
        case TSL25911_GAIN_MAX: again = 9876.0F; break;
        default: again = 1.0F; break;
    }

    // Avoid invalid readings or division by zero
    if (ch0 == 0xFFFF || ch1 == 0xFFFF || ch0 == 0) {
        return -1.0F; // Invalid reading
    }

    // Calculate CPL
    float cpl = (atime * again) / TSL2591_LUX_DF;

    // Calculate lux
    float lux = (((float)ch0 - (float)ch1) * (1.0F - ((float)ch1 / (float)ch0))) / cpl;

    return lux < 0 ? 0 : lux; // Ensure no negative lux values
}
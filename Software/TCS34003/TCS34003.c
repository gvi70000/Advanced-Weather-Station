#include "TCS34003.h"
#include "i2c.h"

static TCS34003_REGISTERS_t TCS34003_Sensor; // Global variable for sensor configuration

// Write to a register of TSL2591
static HAL_StatusTypeDef TCS34003_WriteRegister(uint8_t reg, uint8_t* data, uint8_t len) {
    return WriteRegister(TCS34003_I2C_ADDR, reg, data, len, &hi2c1);
}

// Read from a register of TSL2591
static HAL_StatusTypeDef TCS34003_ReadRegister(uint8_t reg, uint8_t* data, uint8_t len) {
    return ReadRegister(TCS34003_I2C_ADDR, reg, data, len, &hi2c1);
}

/**
 * @brief Initializes the TCS3400 sensor.
 * Configures the sensor for minimum gain and sets the data-ready interrupt to trigger every second.
 * @retval HAL_StatusTypeDef HAL_OK if initialization is successful, otherwise an error code.
 */
HAL_StatusTypeDef TCS34003_Init(void) {
    HAL_StatusTypeDef status;

    // Enable PON (Power ON) and AEN (ADC Enable)
    status = TCS34003_SetEnable(0x0B); // PON=1, AEN=1
    if (status != HAL_OK) return status;

    // Set integration time to 256 cycles (712 ms)
    status = TCS34003_SetIntegrationTime(TCS34003_ATIME_256_CYCLES);
    if (status != HAL_OK) return status;

    // Set wait time to 85 cycles Wait Time = 236 ms (WLONG=0)
    status = TCS34003_SetWaitTime(TCS34003_WTIME_85_CYCLES);
    if (status != HAL_OK) return status;

    // Disable WLONG in CONFIG register for short wait time
    status = TCS34003_SetConfig(TCS34003_CONFIG_WLONG_DISABLE);
    if (status != HAL_OK) return status;

    // Set gain to minimum (1x)
    status = TCS34003_SetGain(TCS34003_AGAIN_1X);
    if (status != HAL_OK) return status;

    // Set interrupt persistence to generate interrupts after every RGBC cycle
    status = TCS34003_SetInterruptPersistence(TCS34003_APERS_EVERY_CYCLE);
    if (status != HAL_OK) return status;

    // Configure the interrupt thresholds for the Clear channel
    status = TCS34003_SetClearChannelThreshold(TCS34003_MID_VAL, TCS34003_MID_VAL);
    if (status != HAL_OK) return status;
    
//		// Enable PON (Power ON) and AEN (ADC Enable)
//    status = TCS34003_SetEnable(0x0D); // PON=1, AEN=1
//    if (status != HAL_OK) return status;
		
    // Clear any previous interrupts
    status = TCS34003_ClearInterrupts();
    if (status != HAL_OK) return status;

    // Enable interrupts by setting AIEN
    status = TCS34003_EnableInterrupts();
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/**
 * @brief Set the ENABLE register.
 * @param value The new ENABLE register value.
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetEnable(uint8_t value) {
    TCS34003_Sensor.ENABLE.Val.Value = value;
    return TCS34003_WriteRegister(TCS34003_REG_ENABLE, &TCS34003_Sensor.ENABLE.Val.Value, 1);
}

/**
 * @brief Set the ATIME register (RGBC Integration Time).
 * @param value Integration time (TCS34003_ATIME_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetIntegrationTime(TCS34003_ATIME_t value) {
    TCS34003_Sensor.ATIME = value;
    return TCS34003_WriteRegister(TCS34003_REG_ATIME, (uint8_t*)&TCS34003_Sensor.ATIME, 1);
}

/**
 * @brief Set the WTIME register (Wait Time).
 * @param value Wait time (TCS34003_WTIME_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetWaitTime(TCS34003_WTIME_t value) {
    TCS34003_Sensor.WTIME = value;
    return TCS34003_WriteRegister(TCS34003_REG_WTIME, (uint8_t*)&TCS34003_Sensor.WTIME, 1);
}

/**
 * @brief Set the Clear Channel Interrupt Threshold.
 * @param low Low threshold value.
 * @param high High threshold value.
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetClearChannelThreshold(uint16_t low, uint16_t high) {
    TCS34003_Sensor.CC_TH.LowThreshold.Value = low;
    TCS34003_Sensor.CC_TH.HighThreshold.Value = high;

    HAL_StatusTypeDef status;
    status = TCS34003_WriteRegister(TCS34003_REG_AILTL, (uint8_t*)&TCS34003_Sensor.CC_TH.LowThreshold.Bytes, 2);
    if (status != HAL_OK) return status;

    return TCS34003_WriteRegister(TCS34003_REG_AIHTL, (uint8_t*)&TCS34003_Sensor.CC_TH.HighThreshold.Bytes, 2);
}

/**
 * @brief Enables interrupts by setting the AIEN bit in the ENABLE register.
 * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef TCS34003_EnableInterrupts(void) {
    uint8_t enable_reg;

    // Read the current value of the ENABLE register
    HAL_StatusTypeDef status = TCS34003_ReadRegister(TCS34003_REG_ENABLE, &enable_reg, 1);
    if (status != HAL_OK) return status;

    // Set the AIEN bit (bit 4) to enable interrupts
    enable_reg |= 0x10;

    // Write the updated value back to the ENABLE register
    status = TCS34003_WriteRegister(TCS34003_REG_ENABLE, &enable_reg, 1);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/**
 * @brief Set the APERS register (Interrupt Persistence Filter).
 * @param value Persistence filter (TCS34003_APERS_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetInterruptPersistence(TCS34003_APERS_t value) {
    TCS34003_Sensor.APERS = value;
    return TCS34003_WriteRegister(TCS34003_REG_PERS, (uint8_t*)&TCS34003_Sensor.APERS, 1);
}

/**
 * @brief Set the CONFIG register.
 * @param value Configuration value (TCS34003_CONFIG_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetConfig(TCS34003_CONFIG_t value) {
    TCS34003_Sensor.CONFIG = value;
    return TCS34003_WriteRegister(TCS34003_REG_CONFIG, (uint8_t*)&TCS34003_Sensor.CONFIG, 1);
}

/**
 * @brief Set the CONTROL register (Gain Control).
 * @param value Gain control (TCS34003_AGAIN_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetGain(TCS34003_AGAIN_t value) {
    TCS34003_Sensor.AGAIN = value;
    return TCS34003_WriteRegister(TCS34003_REG_CONTROL, (uint8_t*)&TCS34003_Sensor.AGAIN, 1);
}

/**
 * @brief Set the AUX register (ALS Saturation Interrupt Enable).
 * @param value Saturation interrupt enable (TCS34003_AUX_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetAux(TCS34003_AUX_t value) {
    TCS34003_Sensor.AUX = value;
    return TCS34003_WriteRegister(TCS34003_REG_AUX, (uint8_t*)&TCS34003_Sensor.AUX, 1);
}

/**
 * @brief Set the IR register (IR Sensor Access).
 * @param value IR sensor access (TCS34003_IR_Access_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetIRAccess(TCS34003_IR_Access_t value) {
    TCS34003_Sensor.IR_Access = value;
    return TCS34003_WriteRegister(TCS34003_REG_IR, (uint8_t*)&TCS34003_Sensor.IR_Access, 1);
}

/**
 * @brief Clears all interrupts on the TCS3400 sensor.
 * This writes to the AICLEAR special function register to clear active interrupts.
 * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef TCS34003_ClearInterrupts(void) {
    uint8_t dummy = 0x00; // AICLEAR register does not require specific data
    return TCS34003_WriteRegister(TCS34003_REG_AICLEAR, &dummy, 1);
}

/**
 * @brief Reads the STATUS register from the TCS3400 sensor.
 * This updates the global TCS34003_Sensor structure with the latest STATUS register value.
 * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef TCS34003_GetStatus(void) {
    // Read the STATUS register (0x93)
    return TCS34003_ReadRegister(TCS34003_REG_STATUS, &TCS34003_Sensor.STATUS.Val.Value, 1);
}

/**
 * @brief Reads the light data (CLEAR, RED, GREEN, BLUE, IR) and calculates
 * Correlated Color Temperature (CCT), Luminance (lux), and Irradiance (W/m²).
 * @param[out] data Pointer to a TCS34003_LightData_t structure to store the light data and calculated values.
 * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef TCS34003_GetLightData(TCS34003_LightData_t* data) {
    HAL_StatusTypeDef status;

    // Step 1: Read STATUS register
    status = TCS34003_GetStatus();
    if (status != HAL_OK) {
        return status;
    }

    // Check if ALS data is valid (AVALID bit in STATUS register)
    if (!(TCS34003_Sensor.STATUS.Val.Value & 0x01)) { // AVALID = 1
        return HAL_ERROR; // ALS data not valid
    }

    // Step 2: Read RGBC data into FullArray
    status = TCS34003_ReadRegister(TCS34003_REG_CDATAL, data->RGBC.FullArray, sizeof(data->RGBC.FullArray));
    if (status != HAL_OK) {
        return status;
    }

    // Step 3: Enable IR Sensor access to read IR data
    status = TCS34003_SetIRAccess(TCS34003_IR_ACCESS_ENABLE);
    if (status != HAL_OK) {
        return status;
    }

    // Step 4: Read IR data into FullArray
    status = TCS34003_ReadRegister(TCS34003_REG_CDATAL, data->IR.FullArray, sizeof(data->IR.FullArray));
    if (status != HAL_OK) {
        return status;
    }

    // Step 5: Disable IR Sensor access
    status = TCS34003_SetIRAccess(TCS34003_IR_ACCESS_DISABLE);
    if (status != HAL_OK) {
        return status;
    }

    // Step 6: Calculate Correlated Color Temperature (CCT)
    float X = (TCS34003_CCT_COEFF_X_RED * data->RGBC.RedChannel) + (TCS34003_CCT_COEFF_X_GREEN * data->RGBC.GreenChannel) + (TCS34003_CCT_COEFF_X_BLUE * data->RGBC.BlueChannel);
    float Y = (TCS34003_CCT_COEFF_Y_RED * data->RGBC.RedChannel) + (TCS34003_CCT_COEFF_Y_GREEN * data->RGBC.GreenChannel) + (TCS34003_CCT_COEFF_Y_BLUE * data->RGBC.BlueChannel);
    float Z = (TCS34003_CCT_COEFF_Z_RED * data->RGBC.RedChannel) + (TCS34003_CCT_COEFF_Z_GREEN * data->RGBC.GreenChannel) + (TCS34003_CCT_COEFF_Z_BLUE * data->RGBC.BlueChannel);

    float n = X / (X + Y + Z);
		float n2 = n * n;
    data->CCT = (TCS34003_CCT_COEFF_C1 * (n2 * n)) + (TCS34003_CCT_COEFF_C2 * n2) + (TCS34003_CCT_COEFF_C3 * n) + TCS34003_CCT_COEFF_C4;

    // Step 7: Calculate Luminance (lux)
    data->Lux = (TCS34003_LUX_COEFF_CLEAR * data->RGBC.ClearChannel) +
                (TCS34003_LUX_COEFF_GREEN * data->RGBC.GreenChannel);

    // Step 8: Calculate Irradiance (W/m²)
    data->Irradiance = (data->RGBC.ClearChannel / TCS34003_RESPONSIVITY_CLEAR) * TCS34003_MICRO_TO_W_CONV;

    // Step 9: Clear any active interrupts
    status = TCS34003_ClearInterrupts();
    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}
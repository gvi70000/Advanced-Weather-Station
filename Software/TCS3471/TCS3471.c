#include "TCS3471.h"
#include "i2c.h"

// External declaration for I2C handle
extern I2C_HandleTypeDef hi2c1;

// Global variable to hold sensor data
TCS3471_Registers_t TCS3471_REG;

uint16_t c, r, g, b;
/** @brief Initialize the TCS3471 sensor */
HAL_StatusTypeDef TCS3471_Init(void) {
    HAL_StatusTypeDef status;
		uint8_t id;
		// Turn off the sensor
		status = TCS3471_Enable(ENABLE_NONE);
		if (status != HAL_OK) return status;
		HAL_Delay(5);
    // Step 1: Power on the sensor
    status = TCS3471_Enable(ENABLE_POWER_ON);
    if (status != HAL_OK) return status;
		HAL_Delay(3);
	
		status = TCS3471_GetID(&id);
		// Step 2: Set integration time for best accuracy
    status = TCS3471_SetIntegrationTime(TCS3471_ATIME_700ms);
    if (status != HAL_OK) return status;

    // Step 3: Set wait time
    // To complete the 1-second cycle (700+204=904ms)
    status = TCS3471_SetWaitTime(WTIME_204MS, TCS3471_WLONG_DISABLE);
    if (status != HAL_OK) return status;

    // Step 4: Set gain to 1x
    status = TCS3471_SetGain(TCS3471_AGAIN_1X);
    if (status != HAL_OK) return status;
	
		// Step 5: Set interrupt thresholds to the middle of the interval
    status = TCS3471_SetInterruptThreshold(TCS3471_MID_VAL, TCS3471_MID_VAL);
    if (status != HAL_OK) return status;
	
    // Step 6: Configure persistence (interrupts every time a condition is met)
    status = TCS3471_Persistance(APERS_1_CLEAR_OUT_OF_RANGE);
    if (status != HAL_OK) return status;
		
	  // Step 7: Enable interrupts
    status = TCS3471_Enable(ENABLE_ALL);
    if (status != HAL_OK) return status;
		
		// Step 8: Enable the sensor
		status = TCS3471_ClearInt();
		if (status != HAL_OK) return status;
		
    return HAL_OK;
}


/** @brief Get the ID of the TCS3471 sensor */
HAL_StatusTypeDef TCS3471_GetID(uint8_t *id) {
    return ReadRegister(TCS3471_I2C_ADDRESS, TCS3471_ID_REG, id, 1, &hi2c1);
}

/** @brief Power on the TCS3471 sensor */
HAL_StatusTypeDef TCS3471_Enable(TCS3471_EnableValues_t regVal) {
    TCS3471_REG.ENABLE.Val.Value = regVal; // Set POWER_ON bit
    return WriteRegister(TCS3471_I2C_ADDRESS, TCS3471_ENABLE_REG, &TCS3471_REG.ENABLE.Val.Value, 1, &hi2c1);
}

/** @brief Set the integration time for the TCS3471 sensor */
HAL_StatusTypeDef TCS3471_SetIntegrationTime(TCS3471_ATIME_t integration_time_ms) {
    TCS3471_REG.ATIME = integration_time_ms; // Update global register
    return WriteRegister(TCS3471_I2C_ADDRESS, TCS3471_ATIME_REG, &TCS3471_REG.ATIME, 1, &hi2c1);
}

/** @brief Set the wait time for the TCS3471 sensor */
HAL_StatusTypeDef TCS3471_SetWaitTime(TCS3471_WTIME_t wait_time_ms, TCS3471_WLONG_t multiply_12x) {
    HAL_StatusTypeDef status;
		TCS3471_REG.WTIME = wait_time_ms; // Update global register
    status = WriteRegister(TCS3471_I2C_ADDRESS, TCS3471_WTIME_REG, &TCS3471_REG.WTIME, 1, &hi2c1);
		if (status != HAL_OK) return status;
		TCS3471_REG.WLONG = multiply_12x;
		return WriteRegister(TCS3471_I2C_ADDRESS, TCS3471_CONFIG_REG, &TCS3471_REG.WLONG, 1, &hi2c1);
}

/** @brief Clear interrupts for the TCS3471 sensor */
HAL_StatusTypeDef TCS3471_ClearInt(void) {
	return HAL_I2C_Master_Transmit(&hi2c1, TCS3471_I2C_ADDRESS, (uint8_t*)(COMMAND_INTERRUPT_CLEAR), 1, 100);
}

/** @brief Set the low interrupt threshold for the TCS3471 sensor */
HAL_StatusTypeDef TCS3471_SetInterruptThreshold(uint16_t low_threshold, uint16_t high_threshold) {
		HAL_StatusTypeDef status;
    TCS3471_REG.INT_VAL.AIL = low_threshold; // Update global register
		TCS3471_REG.INT_VAL.AIH = high_threshold; // Update global register
    status = WriteRegister(TCS3471_I2C_ADDRESS, TCS3471_AILTL_REG, (uint8_t*)&TCS3471_REG.INT_VAL.AIL, 2, &hi2c1);
		if (status != HAL_OK) return status;
		return WriteRegister(TCS3471_I2C_ADDRESS, TCS3471_AIHTL_REG, (uint8_t*)&TCS3471_REG.INT_VAL.AIH, 2, &hi2c1);
}

/** @brief Set persistence for the TCS3471 sensor */
HAL_StatusTypeDef TCS3471_Persistance(TCS3471_PERS_t persistence) {
    TCS3471_REG.PERS = persistence; // Update global register
    return WriteRegister(TCS3471_I2C_ADDRESS, TCS3471_PERS_REG, (uint8_t*)&TCS3471_REG.PERS, 1, &hi2c1);
}

HAL_StatusTypeDef TCS3471_SetGain(TCS3471_AGAIN_t gain) {
    TCS3471_REG.GAIN = gain; // Update global register
    return WriteRegister(TCS3471_I2C_ADDRESS, TCS3471_CONTROL_REG, (uint8_t*)&TCS3471_REG.GAIN, 1, &hi2c1);
}

HAL_StatusTypeDef TCS3471_GetStatus(void) {
    return ReadRegister(TCS3471_I2C_ADDRESS, TCS3471_STATUS_REG, &TCS3471_REG.STATUS.Val.Value, 1, &hi2c1);
}

/**
 * @brief Reads CRGB data from the TCS3471 sensor and clears the interrupt.
 * 
 * @param crgb Pointer to a TCS3471_CRGB_t structure where the CRGB data will be stored.
 * @return HAL_StatusTypeDef HAL_OK on success, HAL_ERROR on failure.
 */
HAL_StatusTypeDef TCS3471_GetCRGB(TCS3471_CRGB_t *crgb) {
    HAL_StatusTypeDef status;
		
    // Step 1: Read CRGB data
    status = ReadRegister(TCS3471_I2C_ADDRESS, COMMAND_GET_ALL, (uint8_t*)crgb, sizeof(TCS3471_CRGB_t), &hi2c1);
    if (status != HAL_OK) {
        return status; // Return immediately if reading fails
    }

    // Step 2: Clear the interrupt
	status = HAL_I2C_Master_Transmit(&hi2c1, TCS3471_I2C_ADDRESS, (uint8_t*)(COMMAND_INTERRUPT_CLEAR), 1, 100);
    //status = WriteRegister(TCS3471_I2C_ADDRESS, COMMAND_INTERRUPT_CLEAR, &clear_command, 1, &hi2c1);
    if (status != HAL_OK) {
        return status; // Return if clearing the interrupt fails
    }

    return HAL_OK; // Both operations succeeded
}


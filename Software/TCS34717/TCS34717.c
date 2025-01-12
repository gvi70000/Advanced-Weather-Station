#include "TCS34717.h"
#include "i2c.h"

// External declaration for I2C handle
extern I2C_HandleTypeDef hi2c1;

// Global variable to hold sensor data
TCS34717_Registers_t TCS34717_REG;

uint16_t c, r, g, b;
/** @brief Initialize the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_Init(void) {
    HAL_StatusTypeDef status;
		//TCS34717_PowerOff();
		//HAL_Delay(5);
    // Step 1: Power on the sensor
    status = TCS34717_PowerOn();
    if (status != HAL_OK) return status;
		HAL_Delay(3);

	  // Step 2: Set integration time for best accuracy
    status = TCS34717_SetIntegrationTime(TCS34717_ATIME_700ms);
    if (status != HAL_OK) return status;

    // Step 3: Set wait time
    // To complete the 1-second cycle (700+204=904ms)
    status = TCS34717_SetWaitTime(TCS34717_WTIME_204ms);
    if (status != HAL_OK) return status;

    // Step 4: Set gain to 1x
    status = TCS34717_SetGain(TCS34717_AGAIN_1X);
    if (status != HAL_OK) return status;
	
		// Step 5: Set interrupt thresholds to the middle of the interval
    status = TCS34717_SetInterruptHigh(TCS34717_MID_VAL);
    if (status != HAL_OK) return status;
	
    status = TCS34717_SetInterruptLow(TCS34717_MID_VAL);
    if (status != HAL_OK) return status;
		
    // Step 6: Configure persistence (interrupts every time a condition is met)
    status = TCS34717_Persistance(APERS_1_CLEAR_OUT_OF_RANGE);
    if (status != HAL_OK) return status;
		
	  // Step 7: Enable interrupts
    status = TCS34717_EnableInterrupt();
    if (status != HAL_OK) return status;
		
		// Step 8: Enable the sensor
		status = TCS34717_ClearInt();
		if (status != HAL_OK) return status;
		
    // Step 9: Enable the sensor
    return TCS34717_Enable();
}


/** @brief Get the ID of the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_GetID(void) {
    return ReadRegister(TCS34717_I2C_ADDRESS, TCS34717_ID_REG, &TCS34717_REG.ID, 1, &hi2c1);
}

/** @brief Power on the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_PowerOn(void) {
    TCS34717_REG.ENABLE.Val.BitField.POWER_ON = 1; // Set POWER_ON bit
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS34717_ENABLE_REG, &TCS34717_REG.ENABLE.Val.Value, 1, &hi2c1);
}

/** @brief Power off the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_PowerOff(void) {
    TCS34717_REG.ENABLE.Val.BitField.POWER_ON = 0; // Set POWER_ON bit
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS34717_ENABLE_REG, &TCS34717_REG.ENABLE.Val.Value, 1, &hi2c1);
}

/** @brief Enable the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_Enable(void) {
    TCS34717_REG.ENABLE.Val.Value |= 0x03; // Set POWER_ON and RGBC_EN bits
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS34717_ENABLE_REG, &TCS34717_REG.ENABLE.Val.Value, 1, &hi2c1);
}

/** @brief Disable the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_Disable(void) {
    TCS34717_REG.ENABLE.Val.Value = 0x00; // Clear all ENABLE bits
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS34717_ENABLE_REG, &TCS34717_REG.ENABLE.Val.Value, 1, &hi2c1);
}

/** @brief Set the integration time for the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_SetIntegrationTime(TCS34717_ATIME_t integration_time_ms) {
    TCS34717_REG.ATIME = integration_time_ms; // Update global register
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS34717_ATIME_REG, &TCS34717_REG.ATIME, 1, &hi2c1);
}

/** @brief Set the wait time for the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_SetWaitTime(TCS34717_WTIME_t wait_time_ms) {
    TCS34717_REG.WTIME = wait_time_ms; // Update global register
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS34717_WTIME_REG, &TCS34717_REG.WTIME, 1, &hi2c1);
}

/** @brief Set the gain for the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_SetGain(TCS34717_AGAIN_t gain) {
    TCS34717_REG.CONTROL = gain; // Update global register
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS34717_CONTROL_REG, (uint8_t*)&TCS34717_REG.CONTROL, 1, &hi2c1);
}

/** @brief Enable interrupts for the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_EnableInterrupt(void) {
    TCS34717_REG.ENABLE.Val.Value |= 0x13; // Set INT_EN, POWER_ON, and RGBC_EN bits
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS34717_ENABLE_REG, &TCS34717_REG.ENABLE.Val.Value, 1, &hi2c1);
}

/** @brief Disable interrupts for the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_DisableInterrupt(void) {
    TCS34717_REG.ENABLE.Val.Value &= ~0x10; // Clear INT_EN bit
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS34717_ENABLE_REG, &TCS34717_REG.ENABLE.Val.Value, 1, &hi2c1);
}

/** @brief Clear interrupts for the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_ClearInt(void) {
	uint8_t dummy = 0x00;
	return WriteRegister(TCS34717_I2C_ADDRESS, TCS3471_INT_CLR, &dummy, 1, &hi2c1);
}

/** @brief Set the low interrupt threshold for the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_SetInterruptLow(uint16_t low_threshold) {
    TCS34717_REG.INT_VAL.AIL = low_threshold; // Update global register
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS34717_AILTL_REG, (uint8_t*)&TCS34717_REG.INT_VAL.AIL, 2, &hi2c1);
}

/** @brief Set the high interrupt threshold for the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_SetInterruptHigh(uint16_t high_threshold) {
    TCS34717_REG.INT_VAL.AIH = high_threshold; // Update global register
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS34717_AIHTL_REG, (uint8_t*)&TCS34717_REG.INT_VAL.AIH, 2, &hi2c1);
}

/** @brief Set persistence for the TCS34717 sensor */
HAL_StatusTypeDef TCS34717_Persistance(TCS34717_PERS_t persistence) {
    TCS34717_REG.PERS = persistence; // Update global register
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS34717_PERS_REG, (uint8_t*)&TCS34717_REG.PERS, 1, &hi2c1);
}

/**
 * @brief Reads CRGB data from the TCS34717 sensor and clears the interrupt.
 * 
 * @param crgb Pointer to a TCS34717_CRGB_t structure where the CRGB data will be stored.
 * @return HAL_StatusTypeDef HAL_OK on success, HAL_ERROR on failure.
 */
HAL_StatusTypeDef TCS34717_GetCRGB(TCS34717_CRGB_t *crgb) {
    HAL_StatusTypeDef status;
		
    // Step 1: Read CRGB data
    status = ReadRegister(TCS34717_I2C_ADDRESS, TCS3471_GET_ALL, (uint8_t*)crgb, sizeof(TCS34717_CRGB_t), &hi2c1);
    if (status != HAL_OK) {
        return status; // Return immediately if reading fails
    }

    // Step 2: Clear the interrupt
    uint8_t clear_command = 0x00; // No additional data needed for clearing
		HAL_I2C_Master_Transmit(&hi2c1, TCS34717_I2C_ADDRESS, (uint8_t*)(TCS3471_INT_CLR), 1, 100);
    status = WriteRegister(TCS34717_I2C_ADDRESS, TCS3471_INT_CLR, &clear_command, 1, &hi2c1);
    if (status != HAL_OK) {
        return status; // Return if clearing the interrupt fails
    }

    return HAL_OK; // Both operations succeeded
}

/**
 * @brief Reads all sensor registers into the TCS34717_REG structure.
 * 
 * @return HAL_StatusTypeDef HAL_OK on success, HAL_ERROR on failure.
 */
HAL_StatusTypeDef TCS34717_ReadRegisters(void) {
    HAL_StatusTypeDef status;

    // Read ENABLE register (0x00)
    status = ReadRegister(TCS34717_I2C_ADDRESS, TCS34717_ENABLE_REG, &TCS34717_REG.ENABLE.Val.Value, 1, &hi2c1);
    if (status != HAL_OK) return status;

    // Read ATIME register (0x01)
    status = ReadRegister(TCS34717_I2C_ADDRESS, TCS34717_ATIME_REG, (uint8_t*)&TCS34717_REG.ATIME, 1, &hi2c1);
    if (status != HAL_OK) return status;

    // Read WTIME register (0x03)
    status = ReadRegister(TCS34717_I2C_ADDRESS, TCS34717_WTIME_REG, (uint8_t*)&TCS34717_REG.WTIME, 1, &hi2c1);
    if (status != HAL_OK) return status;

    // Read interrupt thresholds (0x04-0x07)
    status = ReadRegister(TCS34717_I2C_ADDRESS, TCS34717_AILTL_REG, (uint8_t*)&TCS34717_REG.INT_VAL, 4, &hi2c1);
    if (status != HAL_OK) return status;

    // Read PERS register (0x0C)
    status = ReadRegister(TCS34717_I2C_ADDRESS, TCS34717_PERS_REG, (uint8_t*)&TCS34717_REG.PERS, 1, &hi2c1);
    if (status != HAL_OK) return status;

    // Read CONFIG register (0x0D)
    status = ReadRegister(TCS34717_I2C_ADDRESS, TCS34717_CONFIG_REG, (uint8_t*)&TCS34717_REG.CONFIG, 1, &hi2c1);
    if (status != HAL_OK) return status;

    // Read CONTROL register (0x0F)
    status = ReadRegister(TCS34717_I2C_ADDRESS, TCS34717_CONTROL_REG, (uint8_t*)&TCS34717_REG.CONTROL, 1, &hi2c1);
    if (status != HAL_OK) return status;

    // Read ID register (0x12)
    status = ReadRegister(TCS34717_I2C_ADDRESS, TCS34717_ID_REG, &TCS34717_REG.ID, 1, &hi2c1);
    if (status != HAL_OK) return status;

    // Read STATUS register (0x13)
    status = ReadRegister(TCS34717_I2C_ADDRESS, TCS34717_STATUS_REG, &TCS34717_REG.STATUS.Val.Value, 1, &hi2c1);
    if (status != HAL_OK) return status;

    return HAL_OK; // All reads successful
}
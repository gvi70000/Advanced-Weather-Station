#include "TCS34717.h"
#include "i2c.h"

// Function to set specific bits in a register
static HAL_StatusTypeDef TCS34717_SetBits(uint8_t addr, uint8_t bits) {
    uint8_t value;
    if (ReadRegister(TCS34717_I2C_ADDRESS, TCS3471_COMMAND_BIT | addr, &value, 1, &hi2c1) == HAL_OK) {
        value |= bits;
        return WriteRegister(TCS34717_I2C_ADDRESS, TCS3471_COMMAND_BIT | addr, &value, 1, &hi2c1);
    }
    return HAL_ERROR;
}

// Function to clear specific bits in a register
static HAL_StatusTypeDef TCS34717_ClearBits(uint8_t addr, uint8_t bits) {
    uint8_t value;
    if (ReadRegister(TCS34717_I2C_ADDRESS, TCS3471_COMMAND_BIT | addr, &value, 1, &hi2c1) == HAL_OK) {
        value &= ~bits;
        return WriteRegister(TCS34717_I2C_ADDRESS, TCS3471_COMMAND_BIT | addr, &value, 1, &hi2c1);
    }
    return HAL_ERROR;
}

// Function to initialize the sensor
void TCS34717_Init(void) {
    // Enable the sensor with RGBC and interrupt enabled
    TCS34717_Enable();

    // Set the gain to a lower value to prevent saturation in bright environments
    TCS34717_SetGain(TCS34717_AGAIN_1X); // 1X gain suitable for bright conditions

    // Set integration time to balance sensitivity and saturation
    TCS34717_SetIntegrationTime(TCS34717_ATIME_154ms); // Shorter integration time for faster response

    // Set wait time for reduced power consumption if necessary
    TCS34717_SetWaitTime(TCS34717_WTIME_2_4ms); // Minimal wait time for real-time responsiveness

    // Set interrupt thresholds considering outdoor lighting
    TCS34717_RGBC_Interrupt_Threshold_t thresholds;
    thresholds.AIL = 0x2000; // Low threshold for dim light
    thresholds.AIH = 0xFFF0; // High threshold to avoid saturation in bright light
		WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_AILTL_REG, thresholds.Buffer, 4, &hi2c1);
    // Set persistence for interrupt filtering
    TCS34717_SetPersistence(APERS_1_CLEAR_OUT_OF_RANGE); // Require 4 consecutive out-of-range samples
}

// Function to fetch RGBC data and populate the TCS34717_RGBC_Data_t structure
HAL_StatusTypeDef TCS34717_getCRGB(TCS34717_CRGB_t *rgbcData) {
    if (ReadRegister(TCS34717_I2C_ADDRESS, TCS347171_CDATA_REG, rgbcData->Buffer, 8, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

// Optimized function to set interrupt thresholds
HAL_StatusTypeDef TCS34717_SetInterruptThresholds(uint16_t low, uint16_t high) {
    TCS34717_RGBC_Interrupt_Threshold_t thresholds;
    thresholds.AIL = low;  // Set low threshold
    thresholds.AIH = high; // Set high threshold

    // Validate thresholds for outdoor brightness levels
    if (low > high || high > 0xFFFF) {
        return HAL_ERROR; // Invalid threshold range
    }

    return WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_AILTL_REG, thresholds.Buffer, 4, &hi2c1);
}

// Function to enable the sensor
HAL_StatusTypeDef TCS34717_Enable(void) {
    return TCS34717_SetBits(TCS347171_ENABLE_REG, 0x01 | 0x02 | 0x10); // POWER_ON, RGBC_EN, INT_EN bits
}

// Function to disable the sensor
HAL_StatusTypeDef TCS34717_Disable(void) {
    return TCS34717_ClearBits(TCS347171_ENABLE_REG, 0x01 | 0x02); // Clear POWER_ON and RGBC_EN bits
}

// Function to set the gain using the control register
HAL_StatusTypeDef TCS34717_SetGain(TCS34717_AGAIN_t gain) {
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_CONTROL_REG, (uint8_t *)&gain, 1, &hi2c1);
}

// Function to set the integration time (ATIME)
HAL_StatusTypeDef TCS34717_SetIntegrationTime(uint8_t time) {
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_ATIME_REG, &time, 1, &hi2c1);
}

// Function to set the wait time (WTIME)
HAL_StatusTypeDef TCS34717_SetWaitTime(uint8_t wait_time) {
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_WTIME_REG, &wait_time, 1, &hi2c1);
}

// Function to set the persistence register (PERS)
HAL_StatusTypeDef TCS34717_SetPersistence(TCS34717_APERS_t persistence) {
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_PERS_REG, (uint8_t *)&persistence, 1, &hi2c1);
}
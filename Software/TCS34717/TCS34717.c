#include "TCS34717.h"
#include "i2c.h"

// External declaration for I2C handle
extern I2C_HandleTypeDef hi2c1;

/** @brief Global structure to represent TCS34717 registers */
TCS34717_Registers_t TCS34717_REG;

/**
 * @brief Initialize the TCS34717 sensor.
 * 
 * Configures the sensor by enabling power, setting default gain,
 * integration time, wait time, and persistence for interrupt filtering.
 * 
 * @return HAL status.
 */
HAL_StatusTypeDef TCS34717_Init(void) {
    HAL_StatusTypeDef status;

    // Enable power and RGBC functionality
    TCS34717_REG.ENABLE.Val.Value = 0x03; // Power ON and RGBC Enable
    status = WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_ENABLE_REG,
                           &TCS34717_REG.ENABLE.Val.Value, 1, &hi2c1);
    if (status != HAL_OK) return status;

    // Set default integration time (e.g., 101ms)
    TCS34717_REG.ATIME = TCS34717_ATIME_101ms;
    status = WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_ATIME_REG,
                           (uint8_t*)&TCS34717_REG.ATIME, 1, &hi2c1);
    if (status != HAL_OK) return status;

    // Set default wait time (e.g., 204ms)
    TCS34717_REG.WTIME = TCS34717_WTIME_204ms;
    status = WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_WTIME_REG,
                           (uint8_t*)&TCS34717_REG.WTIME, 1, &hi2c1);
    if (status != HAL_OK) return status;

    // Set default gain (e.g., 16x)
    TCS34717_REG.CONTROL = TCS34717_AGAIN_1X;
    status = WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_CONTROL_REG,
                           (uint8_t*)&TCS34717_REG.CONTROL, 1, &hi2c1);
    if (status != HAL_OK) return status;

    // Set persistence for interrupt filtering
    TCS34717_SetPersistence(APERS_1_CLEAR_OUT_OF_RANGE); // Require 1 consecutive out-of-range sample

    return HAL_OK;
}

/**
 * @brief Read RGBC data from the TCS34717 sensor.
 * 
 * Reads clear, red, green, and blue channel data into the global
 * `TCS34717_REG.RGBC` structure.
 * 
 * @param rgbcData Pointer to a structure to store the RGBC data.
 * @return HAL status.
 */
HAL_StatusTypeDef TCS34717_ReadRGBCData(TCS34717_CRGB_t *rgbcData) {
    HAL_StatusTypeDef status;

    // Read RGBC data registers (0x14 - 0x1B)
    status = ReadRegister(TCS34717_I2C_ADDRESS, TCS347171_CDATA_REG,
                          TCS34717_REG.RGBC.ByteArray, 8, &hi2c1);
    if (status == HAL_OK && rgbcData != NULL) {
        *rgbcData = TCS34717_REG.RGBC; // Copy data to user-provided structure
    }
    return status;
}

/**
 * @brief Enable the TCS34717 sensor.
 */
void TCS34717_Enable(void) {
    TCS34717_REG.ENABLE.Val.Value |= 0x03; // Power ON and RGBC Enable
    WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_ENABLE_REG,
                  &TCS34717_REG.ENABLE.Val.Value, 1, &hi2c1);
}

/**
 * @brief Disable the TCS34717 sensor.
 */
void TCS34717_Disable(void) {
    TCS34717_REG.ENABLE.Val.Value &= ~0x03; // Clear Power ON and RGBC Enable bits
    WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_ENABLE_REG,
                  &TCS34717_REG.ENABLE.Val.Value, 1, &hi2c1);
}

/**
 * @brief Set the gain for the TCS34717 sensor.
 * 
 * @param gain Gain setting (TCS34717_AGAIN_t).
 */
void TCS34717_SetGain(TCS34717_AGAIN_t gain) {
    TCS34717_REG.CONTROL = gain;
    WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_CONTROL_REG,
                  (uint8_t*)&TCS34717_REG.CONTROL, 1, &hi2c1);
}

/**
 * @brief Set the integration time for the TCS34717 sensor.
 * 
 * @param time Integration time value (TCS34717_ATIME_t).
 */
void TCS34717_SetIntegrationTime(uint8_t time) {
    TCS34717_REG.ATIME = time;
    WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_ATIME_REG,
                  (uint8_t*)&TCS34717_REG.ATIME, 1, &hi2c1);
}

/**
 * @brief Set the wait time for the TCS34717 sensor.
 * 
 * @param wait_time Wait time value (TCS34717_WTIME_t).
 */
void TCS34717_SetWaitTime(uint8_t wait_time) {
    TCS34717_REG.WTIME = wait_time;
    WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_WTIME_REG,
                  (uint8_t*)&TCS34717_REG.WTIME, 1, &hi2c1);
}

/**
 * @brief Set the persistence value for the interrupt.
 * 
 * @param persistence Persistence setting (TCS34717_APERS_t).
 */
void TCS34717_SetPersistence(TCS34717_PERS_t persistence) {
    TCS34717_REG.PERS = persistence;
    WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_PERS_REG,
                  (uint8_t*)&TCS34717_REG.PERS, 1, &hi2c1);
}

/**
 * @brief Check if the RGBC data is valid.
 * 
 * @return 1 if valid, 0 otherwise.
 */
uint8_t TCS34717_IsDataValid(void) {
    ReadRegister(TCS34717_I2C_ADDRESS, TCS347171_STATUS_REG,
                 &TCS34717_REG.STATUS.Val.Value, 1, &hi2c1);
    return TCS34717_REG.STATUS.Val.BitField.AVALID;
}
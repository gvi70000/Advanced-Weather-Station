#include "TCS34717.h"
#include "i2c.h"

// Function to read data from a TCS34717 register
uint8_t TCS34717_Read(uint8_t addr, uint8_t *buffer, uint16_t num_bytes) {
    return (ReadRegister(TCS34717_I2C_ADDRESS, TCS3471_COMMAND_BIT | addr, buffer, num_bytes, &hi2c1) == HAL_OK) ? 1 : 0;
}

// Function to write data to a TCS34717 register
uint8_t TCS34717_Write(uint8_t addr, uint8_t *buffer, uint16_t num_bytes) {
    return (WriteRegister(TCS34717_I2C_ADDRESS, TCS3471_COMMAND_BIT | addr, buffer, num_bytes, &hi2c1) == HAL_OK) ? 1 : 0;
}

// Function to set specific bits in a register
uint8_t TCS34717_SetBits(uint8_t addr, uint8_t bits) {
    uint8_t value;
    if (TCS34717_Read(addr, &value, 1)) {
        value |= bits;
        return TCS34717_Write(addr, &value, 1);
    }
    return 0;
}

// Function to clear specific bits in a register
uint8_t TCS34717_ClearBits(uint8_t addr, uint8_t bits) {
    uint8_t value;
    if (TCS34717_Read(addr, &value, 1)) {
        value &= ~bits;
        return TCS34717_Write(addr, &value, 1);
    }
    return 0;
}

void TCS34717_Init(void) {
    // Enable the sensor with RGBC and interrupt enabled
    TCS34717_Enable();

    // Set the gain for high precision
    TCS34717_SetGain(TCS34717_AGAIN_1X); // Adjust for application; 16x is a good starting point

    // Set integration time for high precision
    TCS34717_SetIntegrationTime(TCS34717_ATIME_700ms); // Longest integration time for best precision

    // Set wait time (optional)
    TCS34717_SetWaitTime(TCS34717_WTIME_614ms); // Configure if wait functionality is needed

    // Set interrupt thresholds
    uint16_t lowThreshold = 32767;  // Example: Low threshold for RGBC clear channel
    uint16_t highThreshold = 32767; // Example: High threshold for RGBC clear channel
    uint8_t thresholds[4] = {
        lowThreshold & 0xFF,				// Low threshold low byte
        (lowThreshold >> 8) & 0xFF,	// Low threshold high byte
        highThreshold & 0xFF,				// High threshold low byte
        (highThreshold >> 8) & 0xFF	// High threshold high byte
    };
    TCS34717_Write(TCS347171_AILTL_REG, thresholds, 4);

    // Set persistence for interrupt filtering
    TCS34717_SetPersistence(APERS_1_CLEAR_OUT_OF_RANGE); // Require 1 consecutive out-of-range samples
}

// Function to fetch RGBC data and populate the TCS34717_RGBC_Data_t structure
HAL_StatusTypeDef TCS34717_ReadRGBCData(TCS34717_CRGB_t *rgbcData) {
    // Buffer to hold 8 bytes of RGBC data
    uint8_t buffer[8];

    // Read 8 bytes of RGBC data starting from the Clear channel low byte register
    if (TCS34717_Read(TCS347171_CDATA_REG, buffer, sizeof(buffer)) != HAL_OK) {
        return HAL_ERROR; // Read failed
    }

    // Combine bytes into 16-bit values and store in the structure
    rgbcData->CDATA = (buffer[1] << 8) | buffer[0]; // Clear channel
    rgbcData->RDATA = (buffer[3] << 8) | buffer[2]; // Red channel
    rgbcData->GDATA = (buffer[5] << 8) | buffer[4]; // Green channel
    rgbcData->BDATA = (buffer[7] << 8) | buffer[6]; // Blue channel

    return HAL_OK; // Successfully populated RGBC structure
}

// Function to enable the sensor
void TCS34717_Enable(void) {
    TCS34717_SetBits(TCS347171_ENABLE_REG, 0x01 | 0x02 | 0x10); // POWER_ON, RGBC_EN, INT_EN bits
}

// Function to disable the sensor
void TCS34717_Disable(void) {
    TCS34717_ClearBits(TCS347171_ENABLE_REG, 0x01 | 0x02); // Clear POWER_ON and RGBC_EN bits
}

// Function to set the gain using the control register
void TCS34717_SetGain(TCS34717_AGAIN_t gain) {
    uint8_t gain_value = (uint8_t)gain;
    TCS34717_Write(TCS347171_CONTROL_REG, &gain_value, 1);
}

// Function to set the integration time (ATIME)
void TCS34717_SetIntegrationTime(uint8_t time) {
    TCS34717_Write(TCS347171_ATIME_REG, &time, 1);
}

// Function to set the wait time (WTIME)
void TCS34717_SetWaitTime(uint8_t wait_time) {
    TCS34717_Write(TCS347171_WTIME_REG, &wait_time, 1);
}

// Function to check if the sensor data is valid
uint8_t TCS34717_IsDataValid(void) {
    uint8_t value;
    if (TCS34717_Read(TCS347171_STATUS_REG, &value, 1)) {
        return (value & 0x01); // Check AVALID bit (bit 0)
    }
    return 0;
}

// Function to set the persistence register (PERS)
void TCS34717_SetPersistence(TCS34717_APERS_t persistence) {
    uint8_t value = (uint8_t)persistence;
    TCS34717_Write(TCS347171_PERS_REG, &value, 1);
}
#include "TSL25911.h"
#include "i2c.h"

// External declaration for I2C handle
extern I2C_HandleTypeDef hi2c1;

// Function to initialize the TSL25911 sensor
HAL_StatusTypeDef TSL25911_Init(void) {
    uint8_t id;

    // Step 1: Read and validate the device ID
    if (TSL25911_ReadID(&id) != HAL_OK || id != 0x50) {
        return HAL_ERROR;  // Device not detected or invalid ID
    }

    // Step 2: Enable the sensor
    if (TSL25911_Enable() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 3: Set minimum gain to handle high light levels
    if (TSL25911_SetGain(TSL25911_GAIN_LOW) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 4: Set shorter integration time to prevent saturation
    if (TSL25911_SetIntegrationTime(TSL25911_INTEGRATION_100MS) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 5: Configure ALS interrupt thresholds
    uint16_t lowThreshold = 32767;   // Example: Low threshold in ADC counts
    uint16_t highThreshold = 32767; // Example: High threshold in ADC counts
    uint8_t lowThresholdBytes[2] = { lowThreshold & 0xFF, (lowThreshold >> 8) & 0xFF };
    uint8_t highThresholdBytes[2] = { highThreshold & 0xFF, (highThreshold >> 8) & 0xFF };

    if (WriteRegister(TSL25911_I2C_ADDR, TSL25911_REG_AILTL, lowThresholdBytes, 2, &hi2c1) != HAL_OK ||
        WriteRegister(TSL25911_I2C_ADDR, TSL25911_REG_AIHTL, highThresholdBytes, 2, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 6: Enable ALS interrupts
    uint8_t enable = TSL25911_ENABLE_POWERON | TSL25911_ENABLE_AEN | TSL25911_ENABLE_AIEN;
    if (WriteRegister(TSL25911_I2C_ADDR, TSL25911_REG_ENABLE, &enable, 1, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK; // Initialization successful
}

// Function to enable the TSL25911 sensor
HAL_StatusTypeDef TSL25911_Enable(void) {
    uint8_t enable = TSL25911_ENABLE_POWERON | TSL25911_ENABLE_AEN;
    return WriteRegister(TSL25911_I2C_ADDR, TSL25911_REG_ENABLE, &enable, 1, &hi2c1);
}

// Function to disable the TSL25911 sensor
HAL_StatusTypeDef TSL25911_Disable(void) {
    uint8_t disable = 0x00;  // Disable power and ALS
    return WriteRegister(TSL25911_I2C_ADDR, TSL25911_REG_ENABLE, &disable, 1, &hi2c1);
}

// Function to set gain
HAL_StatusTypeDef TSL25911_SetGain(TSL25911_GAIN_t gain) {
    uint8_t control;

    // Read current control register value
    if (ReadRegister(TSL25911_I2C_ADDR, TSL25911_REG_CONTROL, &control, 1, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }

    control &= 0xCF; // Clear existing gain bits
    control |= gain; // Set new gain

    return WriteRegister(TSL25911_I2C_ADDR, TSL25911_REG_CONTROL, &control, 1, &hi2c1);
}

// Function to set integration time
HAL_StatusTypeDef TSL25911_SetIntegrationTime(TSL25911_INTEGRATION_t integrationTime) {
    uint8_t control;

    // Read current control register value
    if (ReadRegister(TSL25911_I2C_ADDR, TSL25911_REG_CONTROL, &control, 1, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }

    control &= 0xF8; // Clear existing integration time bits
    control |= integrationTime; // Set new integration time

    return WriteRegister(TSL25911_I2C_ADDR, TSL25911_REG_CONTROL, &control, 1, &hi2c1);
}

// Function to read light data (IR, Full Spectrum, Visible, and Lux)
HAL_StatusTypeDef TSL25911_ReadLightData(TSL25911_LightData_t *lightData) {
    uint32_t luminosity;
    uint8_t buffer[4];

    // Read 32-bit light data from the sensor (CHAN0 and CHAN1)
    if (ReadRegister(TSL25911_I2C_ADDR, TSL25911_REG_C0DATA, buffer, 4, &hi2c1) != HAL_OK) {
        return HAL_ERROR; // Read operation failed
    }

    // Combine bytes into 16-bit channel data
    lightData->FullSpectrum = (buffer[1] << 8) | buffer[0]; // CHAN0 (Full Spectrum)
    lightData->Infrared = (buffer[3] << 8) | buffer[2];     // CHAN1 (Infrared)

    // Calculate visible light as the difference between full spectrum and infrared
    lightData->Visible = lightData->FullSpectrum - lightData->Infrared;

    // Calculate lux using the Adafruit formula or your own calibration coefficients
		if (lightData->FullSpectrum == 0xFFFF || lightData->Infrared == 0xFFFF) {
        lightData->Lux = 0; // Saturated sensor data
    } else {
				lightData->Lux = (lightData->FullSpectrum - lightData->Infrared) * 0.5f;
		}
    return HAL_OK; // Light data read successfully
}

// Function to read the ID register
HAL_StatusTypeDef TSL25911_ReadID(uint8_t *id) {
    return ReadRegister(TSL25911_I2C_ADDR, TSL25911_REG_ID, id, 1, &hi2c1);
}

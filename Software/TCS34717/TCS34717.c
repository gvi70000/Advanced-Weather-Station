#include "TCS34717.h"
#include "i2c.h"

static TCS34717_Registers_t TCS34717_REGS; // Static register structure to hold sensor state

// Function to set specific bits in a register
static HAL_StatusTypeDef TCS34717_SetBits(uint8_t addr, uint8_t bits) {
    uint8_t value;
    if (ReadRegister(TCS34717_I2C_ADDRESS, TCS3471_COMMAND_BIT | addr, &value, 1, &hi2c1) == HAL_OK) {
        value |= bits;
        HAL_StatusTypeDef status = WriteRegister(TCS34717_I2C_ADDRESS, TCS3471_COMMAND_BIT | addr, &value, 1, &hi2c1);
        if (status == HAL_OK) {
            // Update the corresponding field in TCS34717_REGS
            if (addr == TCS347171_ENABLE_REG) {
                TCS34717_REGS.ENABLE.Val.Value |= bits;
            }
        }
        return status;
    }
    return HAL_ERROR;
}

// Function to clear specific bits in a register
static HAL_StatusTypeDef TCS34717_ClearBits(uint8_t addr, uint8_t bits) {
    uint8_t value;
    if (ReadRegister(TCS34717_I2C_ADDRESS, TCS3471_COMMAND_BIT | addr, &value, 1, &hi2c1) == HAL_OK) {
        value &= ~bits;
        HAL_StatusTypeDef status = WriteRegister(TCS34717_I2C_ADDRESS, TCS3471_COMMAND_BIT | addr, &value, 1, &hi2c1);
        if (status == HAL_OK) {
            // Update the corresponding field in TCS34717_REGS
            if (addr == TCS347171_ENABLE_REG) {
                TCS34717_REGS.ENABLE.Val.Value &= ~bits;
            }
        }
        return status;
    }
    return HAL_ERROR;
}

// Function to initialize the sensor
void TCS34717_Init(void) {
    // Enable the sensor
    TCS34717_Enable();

    // Set the gain to a lower value to prevent saturation in bright environments
    TCS34717_SetGain(TCS34717_AGAIN_1X);
    TCS34717_REGS.CONTROL = TCS34717_AGAIN_1X;

    // Set integration time to balance sensitivity and saturation
    TCS34717_SetIntegrationTime(TCS34717_ATIME_700ms);
    TCS34717_REGS.ATIME = TCS34717_ATIME_700ms;

    // Set wait time for reduced power consumption if necessary
    TCS34717_SetWaitTime(TCS34717_WTIME_614ms);
    TCS34717_REGS.WTIME = TCS34717_WTIME_614ms;

    // Set interrupt thresholds
    TCS34717_SetInterruptThresholds(TCS347171_LIM, TCS347171_LIM);

    // Set persistence for interrupt filtering
    TCS34717_SetPersistence(APERS_1_CLEAR_OUT_OF_RANGE);
    TCS34717_REGS.PERS = APERS_1_CLEAR_OUT_OF_RANGE;
}

// Function to fetch RGBC data and populate the TCS34717_RGBC_Data_t structure
HAL_StatusTypeDef TCS34717_getCRGB(TCS34717_CRGB_t *rgbcData) {
    if (ReadRegister(TCS34717_I2C_ADDRESS, TCS347171_CDATA_REG, TCS34717_REGS.RGBC_DATA.Buffer, 8, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }
    // Populate the provided structure
    *rgbcData = TCS34717_REGS.RGBC_DATA;
    return HAL_OK;
}

// Function to set interrupt thresholds
HAL_StatusTypeDef TCS34717_SetInterruptThresholds(uint16_t low, uint16_t high) {
    TCS34717_REGS.INT.AIL = low;
    TCS34717_REGS.INT.AIH = high;

    // Write to the hardware
    return WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_AILTL_REG, TCS34717_REGS.INT.Buffer, 4, &hi2c1);
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
    HAL_StatusTypeDef status = WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_CONTROL_REG, (uint8_t *)&gain, 1, &hi2c1);
    if (status == HAL_OK) {
        TCS34717_REGS.CONTROL = gain;
    }
    return status;
}

// Function to set the integration time (ATIME)
HAL_StatusTypeDef TCS34717_SetIntegrationTime(uint8_t time) {
    HAL_StatusTypeDef status = WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_ATIME_REG, &time, 1, &hi2c1);
    if (status == HAL_OK) {
        TCS34717_REGS.ATIME = (TCS34717_ATIME_t)time;
    }
    return status;
}

// Function to set the wait time (WTIME)
HAL_StatusTypeDef TCS34717_SetWaitTime(uint8_t wait_time) {
    HAL_StatusTypeDef status = WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_WTIME_REG, &wait_time, 1, &hi2c1);
    if (status == HAL_OK) {
        TCS34717_REGS.WTIME = (TCS34717_WTIME_t)wait_time;
    }
    return status;
}

// Function to set the persistence register (PERS)
HAL_StatusTypeDef TCS34717_SetPersistence(TCS34717_PERS_t persistence) {
    HAL_StatusTypeDef status = WriteRegister(TCS34717_I2C_ADDRESS, TCS347171_PERS_REG, (uint8_t *)&persistence, 1, &hi2c1);
    if (status == HAL_OK) {
        TCS34717_REGS.PERS = persistence;
    }
    return status;
}
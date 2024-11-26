#include "BMP581.h"
#include "i2c.h"
#include "gpio.h"

// Global instance of BMP581 registers
static BMP581_registers_t bmp581_regs;

// Helper function to convert 3 bytes to a 32-bit integer
static inline int32_t get_32bit(const uint8_t *data_ptr) {
    return (int32_t)(*(uint32_t*)data_ptr & 0x00FFFFFF);  // Masking to keep 3 bytes
}

// Function to initialize the BMP581 sensor
/**
 * @brief BMP581_Init function implementation.
 * @details Detailed implementation for BMP581_Init.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Init(void) {
    // Step 1: Send reset command
    if (BMP581_SendCommand(BMP581_CMD_RESET) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(5);  // Minimal delay as per the datasheet.

    // Step 2: Read and verify the CHIP ID
/**
 * @brief chip_id function implementation.
 * @details Detailed implementation for chip_id.
 * @param None
 * @return HAL status.
 */
    uint8_t chip_id = BMP581_Get_CHIP_ID();
    if (chip_id != BMP581_CHIP_ID_PRIM) {
        return HAL_ERROR;  // Invalid CHIP_ID
    }

    // Step 3: Read the STATUS register to check NVM readiness and errors
    if (BMP581_Get_Status() != HAL_OK) {
        return HAL_ERROR;  // Failed to read STATUS register
    }
    if (bmp581_regs.STATUS.Val.BitField.status_nvm_rdy != 1 || bmp581_regs.STATUS.Val.BitField.status_nvm_err != 0) {
        return HAL_ERROR;  // NVM not ready or there's an NVM error
    }

    // Step 4: Check Power-On Reset (POR) completion
    if (BMP581_Get_INTStatus() != HAL_OK) {
        return HAL_ERROR;  // Failed to read INT_STATUS register
    }
    if (bmp581_regs.INT_STATUS.Val.BitField.por != 1) {
        return HAL_ERROR;  // Power-on reset not complete
    }

    // Step 5: Configure the sensor settings for continuous high-accuracy mode
    bmp581_regs.OSR_CONFIG.Val.BitField.osr_t = BMP581_OSR_128X;  // Oversampling for temperature
    bmp581_regs.OSR_CONFIG.Val.BitField.osr_p = BMP581_OSR_128X;  // Oversampling for pressure
    bmp581_regs.OSR_CONFIG.Val.BitField.press_en = 1;  // Enable pressure measurement
    if (BMP581_Set_OSRConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 6: Configure the IIR filters
    bmp581_regs.DSP_IIR_CONFIG.Val.BitField.set_iir_t = BMP581_IIR_COEFF_127;  // IIR filter for temperature
    bmp581_regs.DSP_IIR_CONFIG.Val.BitField.set_iir_p = BMP581_IIR_COEFF_127;  // IIR filter for pressure
    if (BMP581_Set_DSPIIRConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 7: Set the output data rate (ODR) and power mode
    bmp581_regs.ODR_CONFIG.Val.BitField.odr = BMP581_ODR_1_HZ;  // Set ODR to 1Hz
    bmp581_regs.ODR_CONFIG.Val.BitField.pwr_mode = BMP581_PWRMODE_NORMAL;  // Set to normal power mode
    bmp581_regs.ODR_CONFIG.Val.BitField.deep_dis = BMP581_DEEP_DISABLED;  // Disable deep standby
    if (BMP581_Set_ODRConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 8: Configure interrupts
    bmp581_regs.INT_CONFIG.Val.BitField.int_mode = BMP581_INT_MODE_PULSED;
    bmp581_regs.INT_CONFIG.Val.BitField.int_pol = BMP581_INT_POL_ACTIVE_HIGH;
    bmp581_regs.INT_CONFIG.Val.BitField.int_od = BMP581_INT_PIN_PUSH_PULL;
    bmp581_regs.INT_CONFIG.Val.BitField.int_en = BMP581_INT_ENABLE;
    if (BMP581_Set_INTConfig() != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 9: Enable data-ready interrupt
    bmp581_regs.INT_SOURCE.Val.BitField.drdy_data_reg_en = BMP581_INT_ENABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.fifo_full_en = BMP581_INT_DISABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.fifo_ths_en = BMP581_INT_DISABLE;
    bmp581_regs.INT_SOURCE.Val.BitField.oor_p_en = BMP581_INT_DISABLE;
    if (BMP581_Set_INTSource() != HAL_OK) {
        return HAL_ERROR;
    }

    // Initialization complete
    return HAL_OK;
}

// Function to send commands to BMP581 sensor
/**
 * @brief BMP581_SendCommand function implementation.
 * @details Detailed implementation for BMP581_SendCommand.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_SendCommand(BMP581_cmd_t cmd) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_CMD, (uint8_t*)&cmd, ONE, &hi2c2);
}

// REG 0x01 - CHIP ID
/**
 * @brief BMP581_Get_CHIP_ID function implementation.
 * @details Detailed implementation for BMP581_Get_CHIP_ID.
 * @param None
 * @return HAL status.
 */
uint8_t BMP581_Get_CHIP_ID(void) {
/**
 * @brief id function implementation.
 * @details Detailed implementation for id.
 * @param None
 * @return HAL status.
 */
    uint8_t id = 0;
    if (ReadRegister(BMP581_ADDRESS, BMP581_REG_CHIP_ID, &id, ONE, &hi2c2) == HAL_OK) {
        return id;
    }
    return 0;
}

// REG 0x02 - CHIP Revision ID
/**
 * @brief BMP581_Get_CHIP_REV function implementation.
 * @details Detailed implementation for BMP581_Get_CHIP_REV.
 * @param None
 * @return HAL status.
 */
uint8_t BMP581_Get_CHIP_REV(void) {
/**
 * @brief rev function implementation.
 * @details Detailed implementation for rev.
 * @param None
 * @return HAL status.
 */
    uint8_t rev = 0;
    if (ReadRegister(BMP581_ADDRESS, BMP581_REG_REV_ID, &rev, ONE, &hi2c2) == HAL_OK) {
        return rev;
    }
    return 0;
}

// REG 0x11 - CHIP STATUS
/**
 * @brief BMP581_Get_CHIPStatus function implementation.
 * @details Detailed implementation for BMP581_Get_CHIPStatus.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_CHIPStatus(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_CHIP_STATUS, (uint8_t*)&bmp581_regs.CHIP_STATUS.Val, ONE, &hi2c2);
}

// REG 0x13 - DRIVE CONFIG
/**
 * @brief BMP581_Set_DriveConfig function implementation.
 * @details Detailed implementation for BMP581_Set_DriveConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_DriveConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_DRIVE_CONFIG, (uint8_t*)&bmp581_regs.DRIVE_CONFIG.Val, ONE, &hi2c2);
}

/**
 * @brief BMP581_Get_DriveConfig function implementation.
 * @details Detailed implementation for BMP581_Get_DriveConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_DriveConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_DRIVE_CONFIG, (uint8_t*)&bmp581_regs.DRIVE_CONFIG.Val, ONE, &hi2c2);
}

// REG 0x14 - Interrupt Configuration
/**
 * @brief BMP581_Set_INTConfig function implementation.
 * @details Detailed implementation for BMP581_Set_INTConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_INTConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_INT_CONFIG, (uint8_t*)&bmp581_regs.INT_CONFIG.Val, ONE, &hi2c2);
}

/**
 * @brief BMP581_Get_INTConfig function implementation.
 * @details Detailed implementation for BMP581_Get_INTConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_INTConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_INT_CONFIG, (uint8_t*)&bmp581_regs.INT_CONFIG.Val, ONE, &hi2c2);
}

// REG 0x15 - Interrupt Source Configuration
/**
 * @brief BMP581_Set_INTSource function implementation.
 * @details Detailed implementation for BMP581_Set_INTSource.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_INTSource(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_INT_SOURCE, (uint8_t*)&bmp581_regs.INT_SOURCE.Val, ONE, &hi2c2);
}

/**
 * @brief BMP581_Get_INTSource function implementation.
 * @details Detailed implementation for BMP581_Get_INTSource.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_INTSource(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_INT_SOURCE, (uint8_t*)&bmp581_regs.INT_SOURCE.Val, ONE, &hi2c2);
}

// REG 0x16 - FIFO Configuration
/**
 * @brief BMP581_Set_FIFOConfig function implementation.
 * @details Detailed implementation for BMP581_Set_FIFOConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_FIFOConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_FIFO_CONFIG, (uint8_t*)&bmp581_regs.FIFO_CONFIG.Val, ONE, &hi2c2);
}

/**
 * @brief BMP581_Get_FIFOConfig function implementation.
 * @details Detailed implementation for BMP581_Get_FIFOConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_FIFO_CONFIG, (uint8_t*)&bmp581_regs.FIFO_CONFIG.Val, ONE, &hi2c2);
}

// REG 0x17 - FIFO Count
/**
 * @brief BMP581_Get_FIFOCount function implementation.
 * @details Detailed implementation for BMP581_Get_FIFOCount.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOCount(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_FIFO_COUNT, (uint8_t*)&bmp581_regs.FIFO_COUNT.Val, ONE, &hi2c2);
}

// REG 0x18 - FIFO Frame Selection
/**
 * @brief BMP581_Set_FIFOFrameSel function implementation.
 * @details Detailed implementation for BMP581_Set_FIFOFrameSel.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_FIFOFrameSel(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_FIFO_SEL, (uint8_t*)&bmp581_regs.FIFO_SEL.Val, ONE, &hi2c2);
}

/**
 * @brief BMP581_Get_FIFOFrameSel function implementation.
 * @details Detailed implementation for BMP581_Get_FIFOFrameSel.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOFrameSel(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_FIFO_SEL, (uint8_t*)&bmp581_regs.FIFO_SEL.Val, ONE, &hi2c2);
}

// REG 0x1D - 0x1F Temperature and 0x20 - 0x22 Pressure Data
/**
 * @brief BMP581_Get_TempPressData function implementation.
 * @details Detailed implementation for BMP581_Get_TempPressData.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_TempPressData(BMP581_sensor_data_t *data) {
/**
 * @brief reg_data[SIX] function implementation.
 * @details Detailed implementation for reg_data[SIX].
 * @param None
 * @return HAL status.
 */
    uint8_t reg_data[SIX] __attribute__((aligned(4)));  // Ensure 32-bit alignment
    if (ReadRegister(BMP581_ADDRESS, BMP581_REG_TEMP_DATA_XLSB, reg_data, SIX, &hi2c2) == HAL_OK) {
        // Convert raw temperature data
        int32_t raw_data = get_32bit(&reg_data[0]);  // Aligned access
        data->temperature = (float)raw_data / TEMP_COEFF;

        // Convert raw pressure data
        raw_data = get_32bit(&reg_data[3]);  // Aligned access
        data->pressure = (float)raw_data / PRESS_COEFF;
        return HAL_OK;
    }
    return HAL_ERROR;
}

// REG 0x27 - Interrupt Status
/**
 * @brief BMP581_Get_INTStatus function implementation.
 * @details Detailed implementation for BMP581_Get_INTStatus.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_INTStatus(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_INT_STATUS, (uint8_t*)&bmp581_regs.INT_STATUS.Val, ONE, &hi2c2);
}

// REG 0x28 - Status Register
/**
 * @brief BMP581_Get_Status function implementation.
 * @details Detailed implementation for BMP581_Get_Status.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_Status(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_STATUS, (uint8_t*)&bmp581_regs.STATUS.Val, ONE, &hi2c2);
}

// REG 0x29 - FIFO Data
/**
 * @brief BMP581_Get_FIFOData function implementation.
 * @details Detailed implementation for BMP581_Get_FIFOData.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_FIFOData(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_FIFO_DATA, (uint8_t*)&bmp581_regs.FIFO_DATA, ONE, &hi2c2);
}

// REG 0x2B - NVM Address
/**
 * @brief BMP581_Set_NVMAddr function implementation.
 * @details Detailed implementation for BMP581_Set_NVMAddr.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_NVMAddr(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_NVM_ADDR, (uint8_t*)&bmp581_regs.NVM_ADDR.Val, ONE, &hi2c2);
}

/**
 * @brief BMP581_Get_NVMAddr function implementation.
 * @details Detailed implementation for BMP581_Get_NVMAddr.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_NVMAddr(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_NVM_ADDR, (uint8_t*)&bmp581_regs.NVM_ADDR.Val, ONE, &hi2c2);
}

// REG 0x2C - 0x2D - NVM Data
/**
 * @brief BMP581_Set_NVMData function implementation.
 * @details Detailed implementation for BMP581_Set_NVMData.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_NVMData(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_NVM_DATA_LSB, (uint8_t*)&bmp581_regs.NVM_DATA, TWO, &hi2c2);
}

/**
 * @brief BMP581_Get_NVMData function implementation.
 * @details Detailed implementation for BMP581_Get_NVMData.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_NVMData(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_NVM_DATA_LSB, (uint8_t*)&bmp581_regs.NVM_DATA, TWO, &hi2c2);
}

// REG 0x30 - DSP Configuration
/**
 * @brief BMP581_Set_DSPConfig function implementation.
 * @details Detailed implementation for BMP581_Set_DSPConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_DSPConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_DSP_CONFIG, (uint8_t*)&bmp581_regs.DSP_CONFIG.Val, ONE, &hi2c2);
}

/**
 * @brief BMP581_Get_DSPConfig function implementation.
 * @details Detailed implementation for BMP581_Get_DSPConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_DSPConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_DSP_CONFIG, (uint8_t*)&bmp581_regs.DSP_CONFIG.Val, ONE, &hi2c2);
}

// REG 0x31 - DSP IIR Configuration
/**
 * @brief BMP581_Set_DSPIIRConfig function implementation.
 * @details Detailed implementation for BMP581_Set_DSPIIRConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_DSPIIRConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_DSP_IIR, (uint8_t*)&bmp581_regs.DSP_IIR_CONFIG.Val, ONE, &hi2c2);
}

/**
 * @brief BMP581_Get_DSPIIRConfig function implementation.
 * @details Detailed implementation for BMP581_Get_DSPIIRConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_DSPIIRConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_DSP_IIR, (uint8_t*)&bmp581_regs.DSP_IIR_CONFIG.Val, ONE, &hi2c2);
}

// REG 0x32 - 0x33 - OOR Pressure Threshold
/**
 * @brief BMP581_Set_OOR_PRESS_THR function implementation.
 * @details Detailed implementation for BMP581_Set_OOR_PRESS_THR.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OOR_PRESS_THR(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_OOR_THR_P_LSB, (uint8_t*)&bmp581_regs.OOR_PRESS_THR, TWO, &hi2c2);
}

/**
 * @brief BMP581_Get_OOR_PRESS_THR function implementation.
 * @details Detailed implementation for BMP581_Get_OOR_PRESS_THR.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OOR_PRESS_THR(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_OOR_THR_P_LSB, (uint8_t*)&bmp581_regs.OOR_PRESS_THR, TWO, &hi2c2);
}

// REG 0x34 - OOR Range
/**
 * @brief BMP581_Set_OORRange function implementation.
 * @details Detailed implementation for BMP581_Set_OORRange.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OORRange(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_OOR_RANGE, (uint8_t*)&bmp581_regs.OOR_PRESS_RNG, ONE, &hi2c2);
}

/**
 * @brief BMP581_Get_OORRange function implementation.
 * @details Detailed implementation for BMP581_Get_OORRange.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OORRange(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_OOR_RANGE, (uint8_t*)&bmp581_regs.OOR_PRESS_RNG, ONE, &hi2c2);
}

// REG 0x35 - OOR Configuration
/**
 * @brief BMP581_Set_OORConfig function implementation.
 * @details Detailed implementation for BMP581_Set_OORConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OORConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_OOR_CONFIG, (uint8_t*)&bmp581_regs.OOR_CONFIG.Val, ONE, &hi2c2);
}

/**
 * @brief BMP581_Get_OORConfig function implementation.
 * @details Detailed implementation for BMP581_Get_OORConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OORConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_OOR_CONFIG, (uint8_t*)&bmp581_regs.OOR_CONFIG.Val, ONE, &hi2c2);
}

// REG 0x36 - OSR Configuration
/**
 * @brief BMP581_Set_OSRConfig function implementation.
 * @details Detailed implementation for BMP581_Set_OSRConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_OSRConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_OSR_CONFIG, (uint8_t*)&bmp581_regs.OSR_CONFIG.Val, ONE, &hi2c2);
}

/**
 * @brief BMP581_Get_OSRConfig function implementation.
 * @details Detailed implementation for BMP581_Get_OSRConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OSRConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_OSR_CONFIG, (uint8_t*)&bmp581_regs.OSR_CONFIG.Val, ONE, &hi2c2);
}

// REG 0x37 - ODR Configuration
/**
 * @brief BMP581_Set_ODRConfig function implementation.
 * @details Detailed implementation for BMP581_Set_ODRConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Set_ODRConfig(void) {
    return WriteRegister(BMP581_ADDRESS, BMP581_REG_ODR_CONFIG, (uint8_t*)&bmp581_regs.ODR_CONFIG.Val, ONE, &hi2c2);
}

/**
 * @brief BMP581_Get_ODRConfig function implementation.
 * @details Detailed implementation for BMP581_Get_ODRConfig.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_ODRConfig(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_ODR_CONFIG, (uint8_t*)&bmp581_regs.ODR_CONFIG.Val, ONE, &hi2c2);
}

// REG 0x38 - Effective OSR
/**
 * @brief BMP581_Get_OSREff function implementation.
 * @details Detailed implementation for BMP581_Get_OSREff.
 * @param None
 * @return HAL status.
 */
HAL_StatusTypeDef BMP581_Get_OSREff(void) {
    return ReadRegister(BMP581_ADDRESS, BMP581_REG_OSR_EFF, (uint8_t*)&bmp581_regs.OSR_EFF.Val, ONE, &hi2c2);
}

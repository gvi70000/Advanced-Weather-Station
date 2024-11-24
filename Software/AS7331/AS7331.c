#include "AS7331.h"
#include "i2c.h"

// Global instance to hold AS7331 register values
AS7331_Registers_t AS7331_Registers;

// Function to initialize the AS7331 sensor
HAL_StatusTypeDef AS7331_Init(void) {
    uint8_t osr, status;

    // Step 1: Verify communication by reading OSR and STATUS registers
    if (AS7331_ReadOSRStatus(&osr, &status) != HAL_OK) {
        return HAL_ERROR; // Communication failed
    }

    // Step 2: Set the sensor to minimum gain and shorter integration time Minimum gain (e.g., 1x)
    if (AS7331_SetGain(CREG1_GAIN_1X) != HAL_OK) {
        return HAL_ERROR;
    }

    // Shorter integration time (e.g., 200ms)
    if (AS7331_SetIntegrationTime(CREG1_TIME_256MS) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 3: Enable continuous measurement mode with interrupts
    AS7331_Registers.CREG3.Val.BitField.MMODE = CREG3_MMODE_CONT;    // Continuous measurement mode
    AS7331_Registers.CREG3.Val.BitField.RDYOD = CREG3_RDYOD_OPEN_DRAIN; // Open-drain ready output
    if (WriteRegister(AS7331_I2C_ADDR, AS7331_REG_CREG3, &AS7331_Registers.CREG3.Val.Value, 1, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 4: Enable data-ready interrupt in CREG2
    AS7331_Registers.CREG2.Val.BitField.EN_TM = CREG2_EN_TM_ENABLED; // Enable interrupt for measurement ready
    if (WriteRegister(AS7331_I2C_ADDR, AS7331_REG_CREG2, &AS7331_Registers.CREG2.Val.Value, 1, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }

    // Step 5: Update OSR register to start measurements
    AS7331_Registers.OSR.Val.BitField.DOS = OSR_DOS_MEASUREMENT; // Start measurement mode
    if (WriteRegister(AS7331_I2C_ADDR, AS7331_REG_OSR, &AS7331_Registers.OSR.Val.Value, 1, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK; // Initialization successful
}

// Function to read UVA, UVB, and UVC data
HAL_StatusTypeDef AS7331_ReadUVData(AS7331_UVData_t *uvData) {
    AS7331_OutputResultRegisterBank_t outputData;

    // Update the output result registers
    if (AS7331_UpdateOutputResultRegisters(&outputData) != HAL_OK) {
        return HAL_ERROR;
    }

    // Map the output results to UVA, UVB, and UVC
    uvData->UVA = outputData.MRES1;
    uvData->UVB = outputData.MRES2;
    uvData->UVC = outputData.MRES3;

    return HAL_OK;
}

// Function to read and update the output result registers
HAL_StatusTypeDef AS7331_UpdateOutputResultRegisters(AS7331_OutputResultRegisterBank_t *outputRegisters) {
    uint8_t buffer[2];

    if (ReadRegister(AS7331_I2C_ADDR, 0x01, buffer, 2, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }
    outputRegisters->TEMP = (buffer[1] << 8) | buffer[0];

    if (ReadRegister(AS7331_I2C_ADDR, 0x02, buffer, 2, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }
    outputRegisters->MRES1 = (buffer[1] << 8) | buffer[0];

    if (ReadRegister(AS7331_I2C_ADDR, 0x03, buffer, 2, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }
    outputRegisters->MRES2 = (buffer[1] << 8) | buffer[0];

    if (ReadRegister(AS7331_I2C_ADDR, 0x04, buffer, 2, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }
    outputRegisters->MRES3 = (buffer[1] << 8) | buffer[0];

    if (ReadRegister(AS7331_I2C_ADDR, 0x05, buffer, 2, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }
    outputRegisters->OUTCONVL = (buffer[1] << 8) | buffer[0];

    if (ReadRegister(AS7331_I2C_ADDR, 0x06, buffer, 2, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }
    outputRegisters->OUTCONVH = (buffer[1] << 8) | buffer[0];

    return HAL_OK;
}

// Function to set the integration time in the CREG1 register
HAL_StatusTypeDef AS7331_SetIntegrationTime(AS7331_CREG1_TIME_t time) {
    AS7331_Registers.CREG1.Val.BitField.TIME = time;
    return WriteRegister(AS7331_I2C_ADDR, AS7331_REG_CREG1, &AS7331_Registers.CREG1.Val.Value, 1, &hi2c1);
}

// Function to set the gain in the CREG1 register
HAL_StatusTypeDef AS7331_SetGain(AS7331_CREG1_GAIN_t gain) {
    AS7331_Registers.CREG1.Val.BitField.GAIN = gain;
    return WriteRegister(AS7331_I2C_ADDR, AS7331_REG_CREG1, &AS7331_Registers.CREG1.Val.Value, 1, &hi2c1);
}

// Function to read the OSR and STATUS register as part of a single read operation
HAL_StatusTypeDef AS7331_ReadOSRStatus(uint8_t *osr, uint8_t *status) {
    uint8_t buffer[2];

    if (ReadRegister(AS7331_I2C_ADDR, AS7331_REG_OSR, buffer, 2, &hi2c1) != HAL_OK) {
        return HAL_ERROR;
    }

    *osr = buffer[0];
    *status = buffer[1];

    return HAL_OK;
}

// Function to write the entire AS7331_Registers structure back to the device
HAL_StatusTypeDef AS7331_WriteRegisters(void) {
    uint8_t *buffer = (uint8_t *)&AS7331_Registers;
    uint8_t regAddr = 0x00; // Starting register address

    // Write each register byte by byte
    for (uint8_t i = 0; i < sizeof(AS7331_Registers); i++) {
        if (WriteRegister(AS7331_I2C_ADDR, regAddr++, &buffer[i], 1, &hi2c1) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}



#include "AS7331.h" 
#include "i2c.h"

// Global instance to hold AS7331 register values
static AS7331_DataIn_t AS7331_Sensor;
static AS7331_OSR_STATUS_t OSR_STATUS;

/**
 * @brief Write 1 byte of data to a register of the AS7331 sensor.
 *
 * @param reg Register address to write to.
 * @param data Data byte to be written.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef AS7331_WriteRegister(uint8_t reg, uint8_t data) {
    return WriteRegister(AS7331_I2C_ADDR, reg, &data, 1, &hi2c2); // Always write 1 byte
}

/**
 * @brief Read 2 bytes of data from a register of the AS7331 sensor.
 *
 * @param reg Register address to read from.
 * @param data Pointer to the data buffer to store the read data (must be at least 2 bytes).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef AS7331_ReadRegister(uint8_t reg, uint8_t* data) {
    return ReadRegister(AS7331_I2C_ADDR, reg, data, 2, &hi2c2); // Always read 2 bytes
}

static void setDefault(void) {
	AS7331_Sensor.OSR.Val.BitField.DOS = MODE_CONFIGURATION;
	AS7331_Sensor.OSR.Val.BitField.SW_RES = SW_RESET_ON;
	AS7331_Sensor.OSR.Val.BitField.PD = POWER_ON;
	AS7331_Sensor.OSR.Val.BitField.SS = STOP_MEASUREMENT;
	
	AS7331_Sensor.CREG1.Val.BitField.TIME = TIME_1024MS;
	AS7331_Sensor.CREG1.Val.BitField.GAIN = GAIN_16X;
	
	AS7331_Sensor.CREG2.Val.BitField.DIV = DIVIDER_2;
	AS7331_Sensor.CREG2.Val.BitField.EN_DIV = DIVIDER_ENABLED;
	AS7331_Sensor.CREG2.Val.BitField.EN_TM = TIMER_DISABLED;
	
	AS7331_Sensor.CREG3.Val.BitField.CCLK = CLK_1MHZ;
	AS7331_Sensor.CREG3.Val.BitField.RDYOD = RDY_PIN_PUSH_PULL;
	AS7331_Sensor.CREG3.Val.BitField.SB = STANDBY_OFF;
	AS7331_Sensor.CREG3.Val.BitField.MMODE = MEASURE_MODE_CONT;
	
	AS7331_Sensor.BREAK = 255; //255 * 8us = 2040us
	AS7331_Sensor.EDGES = 1; // 1 edge
}
/**
 * @brief Initialize the AS7331 sensor for 1-second interrupts at the highest precision.
 *
 * This function configures the AS7331 sensor with the following settings:
 * - Highest precision with maximum integration time and gain.
 * - Interrupts raised every second.
 * - Configures measurement modes and other relevant registers.
 *
 * @return HAL_StatusTypeDef HAL_OK if initialization was successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_Init(void) {
    // Step 1: Perform a software reset
    if (AS7331_SoftReset(SW_RESET_ON) != HAL_OK) return HAL_ERROR;
    HAL_Delay(100); // Allow reset to complete
    if (AS7331_SoftReset(SW_RESET_OFF) != HAL_OK) return HAL_ERROR;
    // Step 2: Set the sensor to configuration mode and power on
    AS7331_Sensor.OSR.Val.BitField.DOS = MODE_CONFIGURATION;
		if (AS7331_SetPower(POWER_ON) != HAL_OK) return HAL_ERROR;

	
		//AS7331_Sensor.OSR.Val.Value = 0x83;
    // Step 3: Validate the chip ID
    uint8_t id;
    if (AS7331_GetChipID(&id) != HAL_OK || id != AS7331_ID) return HAL_ERROR;

    // Step 4: Set integration time and gain
    if (AS7331_SetIntegrationTime(TIME_1024MS) != HAL_OK) return HAL_ERROR;
    if (AS7331_SetGain(GAIN_1X) != HAL_OK) return HAL_ERROR;

    // Step 5: Configure measurement mode to continuous
    if (AS7331_SetMeasurementMode(MEASURE_MODE_CONT) != HAL_OK) return HAL_ERROR;

    // Step 6: Configure the READY pin for push-pull interrupts
    if (AS7331_SetReadyOutputMode(RDY_PIN_PUSH_PULL) != HAL_OK) return HAL_ERROR;

    // Step 7: Set the clock frequency
    if (AS7331_SetClockFrequency(CLK_1MHZ) != HAL_OK) return HAL_ERROR;

    // Step 8: Calculate and set break time for continuous measurement
    if (AS7331_SetBreakTime(127) != HAL_OK) return HAL_ERROR;

    // Step 9: Set operational state to measurement mode
    if (AS7331_SetOperationalState(MODE_MEASUREMENT) != HAL_OK) return HAL_ERROR;
    if (AS7331_StartStopMeasurement(START_MEASUREMENT) != HAL_OK) return HAL_ERROR;

    // Step 10: Verify sensor status
    if (AS7331_GetOSR_Status() != HAL_OK) return HAL_ERROR;
    if (ReadRegister(AS7331_I2C_ADDR, AS7331_REG_AGEN, &AS7331_Sensor.CREG3.Val.Value, 1, &hi2c2) != HAL_OK) {
        return HAL_ERROR; // Return error if the read fails
    }
    return HAL_OK;
}

/**
 * @brief Get the chip ID and store it in the static AS7331_ID structure.
 *
 * The chip ID is stored in the `DEVID` field of the AGEN register.
 *
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_GetChipID(uint8_t *chipID) {
    uint8_t data; // AGEN register is 1 byte
    // Read the AGEN register
    if (ReadRegister(AS7331_I2C_ADDR, AS7331_REG_AGEN, chipID, 1, &hi2c2) != HAL_OK) {
        return HAL_ERROR; // Return error if the read fails
    }
    return HAL_OK;
}

/**
 * @brief Set the operational state in the OSR register.
 *
 * @param operationalState Operational state to set (DOS field).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_SetOperationalState(AS7331_DOS_t operationalState) {
    AS7331_Sensor.OSR.Val.BitField.DOS = operationalState;
    return AS7331_WriteRegister(AS7331_REG_OSR, AS7331_Sensor.OSR.Val.Value);
}

/**
 * @brief Perform a software reset by modifying the SW_RES field in the OSR register.
 *
 * @param swReset Software reset state to set.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_SoftReset(AS7331_SW_RST_t swReset) {
    AS7331_Sensor.OSR.Val.BitField.SW_RES = swReset;
    return AS7331_WriteRegister(AS7331_REG_OSR, AS7331_Sensor.OSR.Val.Value);
}

/**
 * @brief Set the power state in the OSR register.
 *
 * @param setPower Power state to set (PD field).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_SetPower(AS7331_PWR_t setPower) {
    AS7331_Sensor.OSR.Val.BitField.PD = setPower;
    return AS7331_WriteRegister(AS7331_REG_OSR, AS7331_Sensor.OSR.Val.Value);
}

/**
 * @brief Start or stop measurement by modifying the SS field in the OSR register.
 *
 * @param startStopMeasure Start/Stop measurement state to set.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_StartStopMeasurement(AS7331_SS_t startStopMeasure) {
    AS7331_Sensor.OSR.Val.BitField.SS = startStopMeasure;
    return AS7331_WriteRegister(AS7331_REG_OSR, AS7331_Sensor.OSR.Val.Value);
}

/**
 * @brief Set the integration time in the CREG1 register.
 *
 * @param time Integration time to set (TIME field).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_SetIntegrationTime(AS7331_TIME_t time) {
    AS7331_Sensor.CREG1.Val.BitField.TIME = time;
    return AS7331_WriteRegister(AS7331_REG_CREG1, AS7331_Sensor.CREG1.Val.Value);
}

/**
 * @brief Set the gain in the CREG1 register.
 *
 * @param gain Gain to set (GAIN field).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_SetGain(AS7331_GAIN_t gain) {
    AS7331_Sensor.CREG1.Val.BitField.GAIN = gain;
    return AS7331_WriteRegister(AS7331_REG_CREG1, AS7331_Sensor.CREG1.Val.Value);
}

/**
 * @brief Set the digital divider value in the CREG2 register.
 *
 * @param div Divider value to set (DIV field).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_SetDivider(AS7331_DIV_t divider) {
    AS7331_Sensor.CREG2.Val.BitField.DIV = divider;
    return AS7331_WriteRegister(AS7331_REG_CREG2, AS7331_Sensor.CREG2.Val.Value);
}

/**
 * @brief Enable or disable the digital divider in the CREG2 register.
 *
 * @param enable Enable/Disable state to set (EN_DIV field).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_EnableDivider(AS7331_EN_DIV_t enableDivider) {
    AS7331_Sensor.CREG2.Val.BitField.EN_DIV = enableDivider;
    return AS7331_WriteRegister(AS7331_REG_CREG2, AS7331_Sensor.CREG2.Val.Value);
}

/**
 * @brief Enable or disable internal measurement of the conversion time in the CREG2 register.
 *
 * When disabled (set to 0) in combination with SYND mode, the internal measurement
 * of the conversion time is disabled, and no temperature measurement takes place.
 *
 * @param enable Set to 1 to enable internal measurement, or 0 to disable it.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_EnableInternalMeasurement(AS7331_EN_TM_t enableTimer) {
    AS7331_Sensor.CREG2.Val.BitField.EN_TM = enableTimer;
    return AS7331_WriteRegister(AS7331_REG_CREG2, AS7331_Sensor.CREG2.Val.Value);
}

/**
 * @brief Set the measurement mode in the CREG3 register.
 *
 * @param mode Measurement mode to set (MMODE field).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_SetMeasurementMode(AS7331_MMODE_t mode) {
    AS7331_Sensor.CREG3.Val.BitField.MMODE = mode;
    return AS7331_WriteRegister(AS7331_REG_CREG3, AS7331_Sensor.CREG3.Val.Value);
}

/**
 * @brief Set the standby mode in the CREG3 register.
 *
 * @param standby Standby mode to set (SB field).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_SetStandbyMode(AS7331_SB_t standby) {
    AS7331_Sensor.CREG3.Val.BitField.SB = standby;
    return AS7331_WriteRegister(AS7331_REG_CREG3, AS7331_Sensor.CREG3.Val.Value);
}

/**
 * @brief Set the READY pin output mode in the CREG3 register.
 *
 * @param rdyod READY pin output mode to set (RDYOD field).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_SetReadyOutputMode(AS7331_CREG3_RDYOD_t rdyod) {
    AS7331_Sensor.CREG3.Val.BitField.RDYOD = rdyod;
    return AS7331_WriteRegister(AS7331_REG_CREG3, AS7331_Sensor.CREG3.Val.Value);
}

/**
 * @brief Set the internal clock frequency in the CREG3 register.
 *
 * @param cclk Clock frequency to set (CCLK field).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_SetClockFrequency(AS7331_CLK_t cclk) {
    AS7331_Sensor.CREG3.Val.BitField.CCLK = cclk;
    return AS7331_WriteRegister(AS7331_REG_CREG3, AS7331_Sensor.CREG3.Val.Value);
}

/**
 * @brief Set the break time (TBREAK) between two measurements in the BREAK register.
 *
 * The break time is specified in steps of 8 µs, with a maximum of 2040 µs.
 *
 * @param breakTime Break time to set (0 to 255, where value * 8 µs = break time).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_SetBreakTime(uint8_t breakTime) {
    // Store the break time in the global AS7331_Sensor structure
    AS7331_Sensor.BREAK = breakTime;
    // Write the value to the BREAK register
    return AS7331_WriteRegister(AS7331_REG_BREAK, AS7331_Sensor.BREAK);
}

/**
 * @brief Set the number of SYN falling edges in the EDGES register.
 *
 * This specifies the number of synchronization edges for the measurement process.
 *
 * @param edges Number of SYN falling edges (0 to 255).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_SetEdges(uint8_t edges) {
    // Store the edge count in the global AS7331_Sensor structure
    AS7331_Sensor.EDGES = edges;
    // Write the value to the EDGES register
    return AS7331_WriteRegister(AS7331_REG_EDGES, AS7331_Sensor.EDGES);
}

/**
 * @brief Set the OPTREG:INIT_IDX field in the OPTREG register.
 *
 * The recommended write value is 0b0111001X, where X is the INIT_IDX field (bit 0).
 * Default after power-on reset or software reset.
 *
 * @param initIdx INIT_IDX field value to set (0 or 1).
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_SetOptReg(uint8_t initIdx) {
    // Store the OPTREG value in the global AS7331_Sensor structure
    AS7331_Sensor.OPTREG = (0b01110010 | (initIdx & 0x01)); // 0111001X where X = INIT_IDX
    // Write the value to the OPTREG register
    return AS7331_WriteRegister(AS7331_REG_OPTREG, AS7331_Sensor.OPTREG);
}

/**
 * @brief Read the Operational State Register (OSR) and Status Register values.
 *
 * This function reads 2 bytes from the OSR and STATUS registers and stores
 * them as a combined 16-bit value in the global `OSR_STATUS` structure.
 * The least significant byte (LSB) contains the OSR value, and the most
 * significant byte (MSB) contains the STATUS value.
 *
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_GetOSR_Status(void) {
    uint8_t data[2]; // OSR and STATUS together are 16 bits (2 bytes)
    
    // Read 2 bytes starting from the OSR register address
    HAL_StatusTypeDef status = AS7331_ReadRegister(AS7331_REG_OSR, data);
    if (status == HAL_OK) {
        // Combine LSB (OSR) and MSB (STATUS) into a 16-bit value
        OSR_STATUS.Val.Value = ((uint16_t)data[1] << 8) | data[0];
    }
    
    return status;
}

/**
 * @brief Get the temperature measurement result.
 *
 * The temperature value is 16 bits, with the 12 least significant bits (LSB)
 * containing the actual temperature value. The 4 most significant bits (MSB)
 * are reserved and should be masked out.
 *
 * @param result Pointer to a variable to store the 12-bit temperature result.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_GetTemperature(uint16_t *result) {
    uint8_t data[2]; // Temperature register is 16 bits (2 bytes)
    HAL_StatusTypeDef status = AS7331_ReadRegister(AS7331_REG_TEMP, data);
    if (status == HAL_OK) {
        // Combine LSB and MSB into a 16-bit value
        uint16_t raw_value = ((uint16_t)data[1] << 8) | data[0];
        // Mask out the 4 most significant bits (MSB) to get the 12-bit temperature
        *result = raw_value & 0x0FFF;
    }
    return status;
}

/**
 * @brief Read the measurement result for Channel A.
 *
 * @param result Pointer to a variable to store the measurement result.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_getChannelA(uint16_t *result) {
    uint8_t data[2]; // MRES1 is 16 bits
    HAL_StatusTypeDef status = AS7331_ReadRegister(AS7331_REG_AGEN, data);
    if (status == HAL_OK) {
        *result = ((uint16_t)data[1] << 8) | data[0]; // Combine MSB and LSB
    }
    return status;
}

/**
 * @brief Read the measurement result for Channel B.
 *
 * @param result Pointer to a variable to store the measurement result.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_getChannelB(uint16_t *result) {
    uint8_t data[2]; // MRES2 is 16 bits
    HAL_StatusTypeDef status = AS7331_ReadRegister(AS7331_REG_MRES2, data);
    if (status == HAL_OK) {
        *result = ((uint16_t)data[1] << 8) | data[0]; // Combine MSB and LSB
    }
    return status;
}

/**
 * @brief Read the measurement result for Channel C.
 *
 * @param result Pointer to a variable to store the measurement result.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_getChannelC(uint16_t *result) {
    uint8_t data[2]; // MRES3 is 16 bits
    HAL_StatusTypeDef status = AS7331_ReadRegister(AS7331_REG_MRES3, data);
    if (status == HAL_OK) {
        *result = ((uint16_t)data[1] << 8) | data[0]; // Combine MSB and LSB
    }
    return status;
}

/**
 * @brief Read the conversion time measurement (OUTCONVL and OUTCONVH).
 *
 * OUTCONVL contains the least significant byte (LSB) and middle byte.
 * OUTCONVH contains the most significant byte (MSB) and one empty byte (00h).
 *
 * @param conversionTime Pointer to a variable to store the full 24-bit conversion time.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_ReadConversionTime(uint32_t *conversionTime) {
    uint8_t outconv_l[2]; // OUTCONVL contains LSB and middle byte
    uint8_t outconv_h[2]; // OUTCONVH contains MSB and one empty byte (00h)

    // Read OUTCONVL (2 bytes: LSB and middle byte)
    HAL_StatusTypeDef status_l = AS7331_ReadRegister(AS7331_REG_OUTCONVL, outconv_l);
    if (status_l != HAL_OK) {
        return status_l; // Return error if reading OUTCONVL fails
    }

    // Read OUTCONVH (2 bytes: MSB and empty byte)
    HAL_StatusTypeDef status_h = AS7331_ReadRegister(AS7331_REG_CREG1, outconv_h);
    if (status_h != HAL_OK) {
        return status_h; // Return error if reading OUTCONVH fails
    }

    // Combine LSB, middle byte, and MSB into a 24-bit value
    *conversionTime = ((uint32_t)outconv_h[0] << 16) | ((uint32_t)outconv_l[1] << 8) | (uint32_t)outconv_l[0];

    return HAL_OK;
}

/**
 * @brief Read the temperature, UVA, UVB, and UVC values into an AS7331_DataOut_t structure.
 *
 * This function checks the NDATA bit in the STATUS register (stored in the global OSR_STATUS structure)
 * to ensure data is ready before reading the UV data. It reads 8 bytes from the output registers in one call:
 * - Temperature (16 bits, LSB first)
 * - UVA (16 bits, LSB first)
 * - UVB (16 bits, LSB first)
 * - UVC (16 bits, LSB first)
 *
 * @param uvData Pointer to an AS7331_DataOut_t structure to store the UV data.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef AS7331_ReadUVData(AS7331_DataOut_t *uvData) {
    uint8_t data[8]; // Total 8 bytes for TEMP, UVA, UVB, and UVC

    // Step 1: Update the global OSR_STATUS structure by reading the OSR and STATUS registers
    if (AS7331_GetOSR_Status() != HAL_OK) {
        return HAL_ERROR; // Return error if STATUS register read fails
    }

    // Step 2: Check NDATA bit in STATUS register (bit 3)
    if (OSR_STATUS.Val.BitField.STATUS.Val.BitField.NDATA == 0) {
        return HAL_ERROR; // Data not ready, return error
    }

    // Step 3: Read UV data from the TEMP register and subsequent registers
    if (ReadRegister(AS7331_I2C_ADDR, AS7331_REG_TEMP, data, 8, &hi2c2) != HAL_OK) {
        return HAL_ERROR; // Return error if UV data read fails
    }

    // Step 4: Parse the data into the AS7331_DataOut_t structure
    uvData->TEMP_C100 = (int16_t)((((uint16_t)data[1] << 8) | data[0]) * AS7331_TEMP1 - AS7331_TEMP2); // Convert TEMP to °C * 100
    uvData->UVA = ((uint16_t)data[3] << 8) | data[2];                              // UVA (16 bits)
    uvData->UVB = ((uint16_t)data[5] << 8) | data[4];                              // UVB (16 bits)
    uvData->UVC = ((uint16_t)data[7] << 8) | data[6];                              // UVC (16 bits)

    return HAL_OK;
}
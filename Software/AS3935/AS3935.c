#include "as3935.h"
#include "i2c.h"

AS3935_REGS_t AS3935_REGS;

/**
 * @brief Initializes the AS3935 lightning sensor.
 * 
 * Configures the AS3935 sensor with default settings:
 * - Sets the noise floor level.
 * - Configures AFE gain for sensitivity.
 * - Sets the lightning event threshold.
 * - Calibrates the oscillators.
 * - Optionally enables/disables antenna frequency display.
 * 
 * @retval HAL_OK Initialization successful.
 * @retval HAL_ERROR Initialization failed (e.g., wake-up or calibration issues).
 */
HAL_StatusTypeDef AS3935_Init(void) {
    // Wake up the sensor
    if (!AS3935_WakeUp()) {
        return HAL_ERROR; // Wakeup failed
    }

    // Set default Noise Floor Level (0-7)
    ReadRegister(AS3935_I2C_ADDRESS, THRESHOLD, &AS3935_REGS.NOISE.Val.Value, 1, &hi2c2);
    AS3935_REGS.NOISE.Val.BitField.NF_LEV = 4; // Example: Set to level 4
    WriteRegister(AS3935_I2C_ADDRESS, THRESHOLD, &AS3935_REGS.NOISE.Val.Value, 1, &hi2c2);

    // Set AFE gain (e.g., indoor mode)
    ReadRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &AS3935_REGS.POWER.Val.Value, 1, &hi2c2);
    AS3935_REGS.POWER.Val.BitField.GAIN = 1; // Example: Gain = 1
    WriteRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &AS3935_REGS.POWER.Val.Value, 1, &hi2c2);

    // Set default lightning event threshold
    ReadRegister(AS3935_I2C_ADDRESS, LIGHTNING_REG, &AS3935_REGS.STATISTICS.Val.Value, 1, &hi2c2);
    AS3935_REGS.STATISTICS.Val.BitField.SREJ = 1; // Example: Minimum threshold
    WriteRegister(AS3935_I2C_ADDRESS, LIGHTNING_REG, &AS3935_REGS.STATISTICS.Val.Value, 1, &hi2c2);

    // Recalibrate oscillators
    if (!AS3935_CalibrateOscillators()) {
        return HAL_ERROR; // Calibration failed
    }

    // Optional antenna frequency display
    ReadRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &AS3935_REGS.IRQ.Val.Value, 1, &hi2c2);
    AS3935_REGS.IRQ.Val.BitField.DISP_LCO = 0; // Disable antenna display
    WriteRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &AS3935_REGS.IRQ.Val.Value, 1, &hi2c2);

    return HAL_OK; // Initialization successful
}

/**
 * @brief Powers down the AS3935 lightning sensor.
 * 
 * Sets the power-down bit in the AFE_GAIN register to save power.
 */
void AS3935_PowerDown(void) {
    // Set power-down bit
    ReadRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &AS3935_REGS.POWER.Val.Value, 1, &hi2c2);
    AS3935_REGS.POWER.Val.BitField.POWER = 1; // Set power-down bit
    WriteRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &AS3935_REGS.POWER.Val.Value, 1, &hi2c2);
}

/**
 * @brief Wakes up the AS3935 lightning sensor from power-down mode.
 * 
 * Clears the power-down bit in the AFE_GAIN register and recalibrates
 * the oscillators.
 * 
 * @retval true  Wake-up successful.
 * @retval false Wake-up failed (e.g., calibration error).
 */
bool AS3935_WakeUp(void) {
    // Clear power-down bit
    ReadRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &AS3935_REGS.POWER.Val.Value, 1, &hi2c2);
    AS3935_REGS.POWER.Val.BitField.POWER = 0; // Clear power-down bit
    WriteRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &AS3935_REGS.POWER.Val.Value, 1, &hi2c2);

    HAL_Delay(2); // Wait 2ms for wake-up

    // Calibrate TRCO
    AS3935_SendDirectCommand(CALIB_RCO);
    HAL_Delay(2);

    // Verify calibration status
    ReadRegister(AS3935_I2C_ADDRESS, CALIB_TRCO, &AS3935_REGS.TRCO.Val.Value, 1, &hi2c2);
    return AS3935_REGS.TRCO.Val.BitField.TRCO_CALIB_DONE;
}

/**
 * @brief Recalibrates the AS3935 sensor after it has been powered down.
 * 
 * This function sends the CALIB_RCO command to recalibrate the internal oscillators
 * and briefly displays the TRCO on the IRQ pin for validation.
 */
void AS3935_RecalibrateAfterPowerDown(void) {
    // Send calibration command to recalibrate internal oscillators
    AS3935_SendDirectCommand(CALIB_RCO);
    HAL_Delay(2); // Allow time for calibration to complete

    // Enable TRCO display on IRQ pin for validation
    ReadRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &AS3935_REGS.IRQ.Val.Value, 1, &hi2c2);
    AS3935_REGS.IRQ.Val.BitField.DISP_TRCO = 1;
    WriteRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &AS3935_REGS.IRQ.Val.Value, 1, &hi2c2);
    HAL_Delay(2); // Allow time for TRCO display to stabilize

    // Disable TRCO display on IRQ pin
    AS3935_REGS.IRQ.Val.BitField.DISP_TRCO = 0;
    WriteRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &AS3935_REGS.IRQ.Val.Value, 1, &hi2c2);
}

/**
 * @brief Calibrates the oscillators of the AS3935 lightning sensor.
 * 
 * Sends a direct command to calibrate the internal TRCO and SRCO oscillators,
 * then verifies the calibration status.
 * 
 * @retval true  Calibration successful.
 * @retval false Calibration failed.
 */
bool AS3935_CalibrateOscillators(void) {
    // Send calibration command
    AS3935_SendDirectCommand(CALIB_RCO);
    HAL_Delay(2);

    // Read calibration status
    ReadRegister(AS3935_I2C_ADDRESS, CALIB_TRCO, &AS3935_REGS.TRCO.Val.Value, 1, &hi2c2);
    ReadRegister(AS3935_I2C_ADDRESS, CALIB_SRCO, &AS3935_REGS.SRCO.Val.Value, 1, &hi2c2);

    return AS3935_REGS.TRCO.Val.BitField.TRCO_CALIB_DONE && AS3935_REGS.SRCO.Val.BitField.SRCO_CALIB_DONE;
}

/**
 * @brief Controls the display of an oscillator on the IRQ pin.
 * 
 * Enables or disables the display of the specified oscillator (TRCO, SRCO, or LCO) on the IRQ pin.
 * 
 * @param enable     Set to true to enable the display, false to disable.
 * @param oscillator The oscillator to display (5 = TRCO, 6 = SRCO, 7 = LCO).
 */
void AS3935_DisplayOscillator(bool enable, uint8_t oscillator) {
    ReadRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &AS3935_REGS.IRQ.Val.Value, 1, &hi2c2);
    
    if (enable) {
        AS3935_REGS.IRQ.Val.Value |= (1 << oscillator); // Enable display for the oscillator
    } else {
        AS3935_REGS.IRQ.Val.Value &= ~(1 << oscillator); // Disable display for the oscillator
    }

    WriteRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &AS3935_REGS.IRQ.Val.Value, 1, &hi2c2);
}

/**
 * @brief Sets the internal tuning capacitance of the AS3935 sensor.
 * 
 * Adjusts the tuning capacitance to optimize the antenna tuning.
 * 
 * @param capacitance The desired tuning capacitance (0-15, representing steps of 8pF).
 */
void AS3935_SetTuningCapacitance(uint8_t capacitance) {
    ReadRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &AS3935_REGS.IRQ.Val.Value, 1, &hi2c2);
    
    // Set the capacitance value (lower 4 bits)
    AS3935_REGS.IRQ.Val.BitField.TUN_CAP = capacitance & 0x0F;
    
    WriteRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &AS3935_REGS.IRQ.Val.Value, 1, &hi2c2);
}

/**
 * @brief Sets the frequency division ratio for antenna tuning.
 * 
 * Configures the division ratio for antenna frequency to match the desired tuning.
 * 
 * @param divisionRatio The desired frequency division ratio (0-3).
 */
void AS3935_SetFrequencyDivision(uint8_t divisionRatio) {
    ReadRegister(AS3935_I2C_ADDRESS, INT_MASK_ANT, &AS3935_REGS.INT_FREQ.Val.Value, 1, &hi2c2);
    
    // Set the division ratio (bits 6-7)
    AS3935_REGS.INT_FREQ.Val.BitField.LCO_FDIV = divisionRatio & 0x03;
    
    WriteRegister(AS3935_I2C_ADDRESS, INT_MASK_ANT, &AS3935_REGS.INT_FREQ.Val.Value, 1, &hi2c2);
}

/**
 * @brief Enables or disables the display of the antenna frequency on the IRQ pin.
 * 
 * @param enable Set to true to enable antenna frequency display, false to disable.
 */
void AS3935_EnableAntennaFrequencyDisplay(bool enable) {
    ReadRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &AS3935_REGS.IRQ.Val.Value, 1, &hi2c2);
    
    if (enable) {
        AS3935_REGS.IRQ.Val.BitField.DISP_LCO = 1; // Enable antenna frequency display
    } else {
        AS3935_REGS.IRQ.Val.BitField.DISP_LCO = 0; // Disable antenna frequency display
    }

    WriteRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &AS3935_REGS.IRQ.Val.Value, 1, &hi2c2);
}

/**
 * @brief Estimates the distance to a detected storm.
 * 
 * Reads the DISTANCE register and extracts the distance value.
 * 
 * @return AS3935_Distance_t Distance to the storm (in km).
 */
AS3935_Distance_t AS3935_GetDistanceToStorm(void) {
    ReadRegister(AS3935_I2C_ADDRESS, DISTANCE, &AS3935_REGS.DISTANCE, 1, &hi2c2);
    return (AS3935_Distance_t)(AS3935_REGS.DISTANCE & AS3935_DISTANCE_MASK);
}

/**
 * @brief Retrieves the lightning energy detected by the AS3935 sensor.
 * 
 * Combines the energy registers (LSB, MSB, MMSB) into a 20-bit integer.
 * 
 * @return uint32_t Lightning energy value (20-bit).
 */
uint32_t AS3935_GetLightningEnergy(void) {
    // Read all energy bytes in one I2C transaction
    ReadRegister(AS3935_I2C_ADDRESS, ENERGY_LIGHT_LSB, AS3935_REGS.ENERGY.ByteArray, 3, &hi2c2);

    // Mask the MMSB to ensure only the lower 20 bits are valid
    AS3935_REGS.ENERGY.MMSB &= AS3935_LIGHTNING_ENERGY_MASK;

    return AS3935_REGS.ENERGY.Value;
}

/**
 * @brief Sets the noise floor level of the AS3935 sensor.
 * 
 * The noise floor level affects the sensor's sensitivity to noise.
 * Higher levels make the sensor less sensitive to noise.
 * 
 * @param level Noise floor level (0-7).
 */
void AS3935_SetNoiseFloorLevel(uint8_t level) {
    ReadRegister(AS3935_I2C_ADDRESS, THRESHOLD, &AS3935_REGS.NOISE.Val.Value, 1, &hi2c2);
    AS3935_REGS.NOISE.Val.BitField.NF_LEV = level & 0x07; // Mask to valid range
    WriteRegister(AS3935_I2C_ADDRESS, THRESHOLD, &AS3935_REGS.NOISE.Val.Value, 1, &hi2c2);
}

/**
 * @brief Reads the current noise floor level of the AS3935 sensor.
 * 
 * @return uint8_t Current noise floor level (0-7).
 */
uint8_t AS3935_ReadNoiseFloorLevel(void) {
    ReadRegister(AS3935_I2C_ADDRESS, THRESHOLD, &AS3935_REGS.NOISE.Val.Value, 1, &hi2c2);
    return AS3935_REGS.NOISE.Val.BitField.NF_LEV;
}

/**
 * @brief Masks or unmasks disturbers in the AS3935 sensor.
 * 
 * @param mask Set to true to mask disturbers, false to unmask them.
 */
void AS3935_MaskDisturber(bool mask) {
    ReadRegister(AS3935_I2C_ADDRESS, INT_MASK_ANT, &AS3935_REGS.INT_FREQ.Val.Value, 1, &hi2c2);
    AS3935_REGS.INT_FREQ.Val.BitField.MASK_DIST = mask ? 1 : 0;
    WriteRegister(AS3935_I2C_ADDRESS, INT_MASK_ANT, &AS3935_REGS.INT_FREQ.Val.Value, 1, &hi2c2);
}

/**
 * @brief Reads the interrupt status from the AS3935 sensor.
 * 
 * @return uint8_t Interrupt status (lower 4 bits of INT_MASK_ANT register).
 */
uint8_t AS3935_ReadInterruptStatus(void) {
    ReadRegister(AS3935_I2C_ADDRESS, INT_MASK_ANT, &AS3935_REGS.INT_FREQ.Val.Value, 1, &hi2c2);
    return AS3935_REGS.INT_FREQ.Val.BitField.INT;
}

/**
 * @brief Sets the lightning event threshold in the AS3935 sensor.
 * 
 * @param threshold Lightning event threshold (0-3).
 */
void AS3935_SetLightningEventThreshold(uint8_t threshold) {
    ReadRegister(AS3935_I2C_ADDRESS, LIGHTNING_REG, &AS3935_REGS.STATISTICS.Val.Value, 1, &hi2c2);
    AS3935_REGS.STATISTICS.Val.BitField.SREJ = threshold & 0x03; // Mask to valid range
    WriteRegister(AS3935_I2C_ADDRESS, LIGHTNING_REG, &AS3935_REGS.STATISTICS.Val.Value, 1, &hi2c2);
}

/**
 * @brief Clears the lightning detection statistics in the AS3935 sensor.
 * 
 * Sets and clears the CL_STAT bit in the LIGHTNING_REG register.
 */
void AS3935_ClearStatistics(void) {
    ReadRegister(AS3935_I2C_ADDRESS, LIGHTNING_REG, &AS3935_REGS.STATISTICS.Val.Value, 1, &hi2c2);
    AS3935_REGS.STATISTICS.Val.BitField.CL_STAT = 1; // Set CL_STAT bit
    WriteRegister(AS3935_I2C_ADDRESS, LIGHTNING_REG, &AS3935_REGS.STATISTICS.Val.Value, 1, &hi2c2);
    HAL_Delay(1); // Ensure statistics are cleared
    AS3935_REGS.STATISTICS.Val.BitField.CL_STAT = 0; // Clear CL_STAT bit
    WriteRegister(AS3935_I2C_ADDRESS, LIGHTNING_REG, &AS3935_REGS.STATISTICS.Val.Value, 1, &hi2c2);
}

/**
 * @brief Sends a direct command to the AS3935 sensor.
 * 
 * Direct commands are used for specific tasks like calibration or reset.
 * 
 * @param command The direct command to send to the sensor.
 */
void AS3935_SendDirectCommand(uint8_t command) {
    WriteRegister(AS3935_I2C_ADDRESS, command, NULL, 0, &hi2c2);
}

/**
 * @brief Sets the AFE (Analog Front End) gain of the AS3935 sensor.
 * 
 * The AFE gain determines the sensitivity of the sensor to lightning events.
 * 
 * @param gain The desired AFE gain value (0-31).
 */
void AS3935_SetAFEGain(uint8_t gain) {
    // Read the current value of the AFE_GAIN register
    ReadRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &AS3935_REGS.POWER.Val.Value, 1, &hi2c2);
    
    // Update the gain value while preserving other bits
    AS3935_REGS.POWER.Val.BitField.GAIN = gain & 0x1F; // Mask to valid range (5 bits)
    
    // Write the updated value back to the register
    WriteRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &AS3935_REGS.POWER.Val.Value, 1, &hi2c2);
}

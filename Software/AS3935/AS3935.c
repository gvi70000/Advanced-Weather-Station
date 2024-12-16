#include "as3935.h"
#include "i2c.h"

static AS3935_REGS_t AS3935_Sensor;

/**
 * @brief Write data to a register of the AS3935 sensor.
 *
 * @param reg Register address to write to.
 * @param data Pointer to the data buffer to be written.
 * @param len Length of data to write.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef AS3935_WriteRegister(uint8_t reg, uint8_t* data, uint8_t len) {
    return WriteRegister(AS3935_I2C_ADDRESS, reg, data, len, &hi2c2);
}

/**
 * @brief Read data from a register of the AS3935 sensor.
 *
 * @param reg Register address to read from.
 * @param data Pointer to the data buffer to store the read data.
 * @param len Length of data to read.
 * @return HAL_StatusTypeDef HAL_OK if successful, HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef AS3935_ReadRegister(uint8_t reg, uint8_t* data, uint8_t len) {
    return ReadRegister(AS3935_I2C_ADDRESS, reg, data, len, &hi2c2);
}

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
    AS3935_ReadRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1);
    AS3935_Sensor.NOISE.Val.BitField.NF_LEV = 4; // Example: Set to level 4
    AS3935_WriteRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1);

    // Set AFE gain (e.g., indoor mode)
    AS3935_ReadRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);
    AS3935_Sensor.POWER.Val.BitField.GAIN = 1; // Example: Gain = 1
    AS3935_WriteRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);

    // Set default lightning event threshold
    AS3935_ReadRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1);
    AS3935_Sensor.STATISTICS.Val.BitField.SREJ = 1; // Example: Minimum threshold
    AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1);

    // Recalibrate oscillators
    if (!AS3935_CalibrateOscillators()) {
        return HAL_ERROR; // Calibration failed
    }

    // Optional antenna frequency display
    AS3935_ReadRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
    AS3935_Sensor.IRQ.Val.BitField.DISP_LCO = 0; // Disable antenna display
    AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);

    return HAL_OK; // Initialization successful
}

/**
 * @brief Powers down the AS3935 lightning sensor.
 * 
 * Sets the power-down bit in the AFE_GAIN register to save power.
 */
void AS3935_PowerDown(void) {
    // Set power-down bit
    AS3935_ReadRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);
    AS3935_Sensor.POWER.Val.BitField.POWER = 1; // Set power-down bit
    AS3935_WriteRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);
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
    AS3935_ReadRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);
    AS3935_Sensor.POWER.Val.BitField.POWER = 0; // Clear power-down bit
    AS3935_WriteRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);

    HAL_Delay(2); // Wait 2ms for wake-up

    // Calibrate TRCO
    AS3935_SendDirectCommand(CALIB_RCO);
    HAL_Delay(2);

    // Verify calibration status
    AS3935_ReadRegister(CALIB_TRCO, &AS3935_Sensor.TRCO.Val.Value, 1);
    return AS3935_Sensor.TRCO.Val.BitField.TRCO_CALIB_DONE;
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
    AS3935_ReadRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
    AS3935_Sensor.IRQ.Val.BitField.DISP_TRCO = 1;
    AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
    HAL_Delay(2); // Allow time for TRCO display to stabilize

    // Disable TRCO display on IRQ pin
    AS3935_Sensor.IRQ.Val.BitField.DISP_TRCO = 0;
    AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
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
    AS3935_ReadRegister(CALIB_TRCO, &AS3935_Sensor.TRCO.Val.Value, 1);
    AS3935_ReadRegister(CALIB_SRCO, &AS3935_Sensor.SRCO.Val.Value, 1);

    return AS3935_Sensor.TRCO.Val.BitField.TRCO_CALIB_DONE && AS3935_Sensor.SRCO.Val.BitField.SRCO_CALIB_DONE;
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
    AS3935_ReadRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
    
    if (enable) {
        AS3935_Sensor.IRQ.Val.Value |= (1 << oscillator); // Enable display for the oscillator
    } else {
        AS3935_Sensor.IRQ.Val.Value &= ~(1 << oscillator); // Disable display for the oscillator
    }

    AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
}

/**
 * @brief Sets the internal tuning capacitance of the AS3935 sensor.
 * 
 * Adjusts the tuning capacitance to optimize the antenna tuning.
 * 
 * @param capacitance The desired tuning capacitance (0-15, representing steps of 8pF).
 */
void AS3935_SetTuningCapacitance(uint8_t capacitance) {
    AS3935_ReadRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
    
    // Set the capacitance value (lower 4 bits)
    AS3935_Sensor.IRQ.Val.BitField.TUN_CAP = capacitance & 0x0F;
    
    AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
}

/**
 * @brief Sets the frequency division ratio for antenna tuning.
 * 
 * Configures the division ratio for antenna frequency to match the desired tuning.
 * 
 * @param divisionRatio The desired frequency division ratio (0-3).
 */
void AS3935_SetFrequencyDivision(uint8_t divisionRatio) {
    AS3935_ReadRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);
    
    // Set the division ratio (bits 6-7)
    AS3935_Sensor.INT_FREQ.Val.BitField.LCO_FDIV = divisionRatio & 0x03;
    
    AS3935_WriteRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);
}

/**
 * @brief Enables or disables the display of the antenna frequency on the IRQ pin.
 * 
 * @param enable Set to true to enable antenna frequency display, false to disable.
 */
void AS3935_EnableAntennaFrequencyDisplay(bool enable) {
    AS3935_ReadRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
    
    if (enable) {
        AS3935_Sensor.IRQ.Val.BitField.DISP_LCO = 1; // Enable antenna frequency display
    } else {
        AS3935_Sensor.IRQ.Val.BitField.DISP_LCO = 0; // Disable antenna frequency display
    }

    AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
}

/**
 * @brief Estimates the distance to a detected storm.
 * 
 * Reads the DISTANCE register and extracts the distance value.
 * 
 * @return AS3935_Distance_t Distance to the storm (in km).
 */
AS3935_Distance_t AS3935_GetDistanceToStorm(void) {
    AS3935_ReadRegister(DISTANCE, &AS3935_Sensor.DISTANCE, 1);
    return (AS3935_Distance_t)(AS3935_Sensor.DISTANCE & AS3935_DISTANCE_MASK);
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
    AS3935_ReadRegister(ENERGY_LIGHT_LSB, AS3935_Sensor.ENERGY.ByteArray, 3);

    // Mask the MMSB to ensure only the lower 20 bits are valid
    AS3935_Sensor.ENERGY.MMSB &= AS3935_LIGHTNING_ENERGY_MASK;

    return AS3935_Sensor.ENERGY.Value;
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
    AS3935_ReadRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1);
    AS3935_Sensor.NOISE.Val.BitField.NF_LEV = level & 0x07; // Mask to valid range
    AS3935_WriteRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1);
}

/**
 * @brief Reads the current noise floor level of the AS3935 sensor.
 * 
 * @return uint8_t Current noise floor level (0-7).
 */
uint8_t AS3935_ReadNoiseFloorLevel(void) {
    AS3935_ReadRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1);
    return AS3935_Sensor.NOISE.Val.BitField.NF_LEV;
}

/**
 * @brief Masks or unmasks disturbers in the AS3935 sensor.
 * 
 * @param mask Set to true to mask disturbers, false to unmask them.
 */
void AS3935_MaskDisturber(bool mask) {
    AS3935_ReadRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);
    AS3935_Sensor.INT_FREQ.Val.BitField.MASK_DIST = mask ? 1 : 0;
    AS3935_WriteRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);
}

/**
 * @brief Reads the interrupt status from the AS3935 sensor.
 * 
 * @return uint8_t Interrupt status (lower 4 bits of INT_MASK_ANT register).
 */
uint8_t AS3935_ReadInterruptStatus(void) {
    AS3935_ReadRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);
    return AS3935_Sensor.INT_FREQ.Val.BitField.INT;
}

/**
 * @brief Sets the lightning event threshold in the AS3935 sensor.
 * 
 * @param threshold Lightning event threshold (0-3).
 */
void AS3935_SetLightningEventThreshold(uint8_t threshold) {
    AS3935_ReadRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1);
    AS3935_Sensor.STATISTICS.Val.BitField.SREJ = threshold & 0x03; // Mask to valid range
    AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1);
}

/**
 * @brief Clears the lightning detection statistics in the AS3935 sensor.
 * 
 * Sets and clears the CL_STAT bit in the LIGHTNING_REG register.
 */
void AS3935_ClearStatistics(void) {
    AS3935_ReadRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1);
    AS3935_Sensor.STATISTICS.Val.BitField.CL_STAT = 1; // Set CL_STAT bit
    AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1);
    HAL_Delay(1); // Ensure statistics are cleared
    AS3935_Sensor.STATISTICS.Val.BitField.CL_STAT = 0; // Clear CL_STAT bit
    AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1);
}

/**
 * @brief Sends a direct command to the AS3935 sensor.
 * 
 * Direct commands are used for specific tasks like calibration or reset.
 * 
 * @param command The direct command to send to the sensor.
 */
void AS3935_SendDirectCommand(uint8_t command) {
    AS3935_WriteRegister(command, NULL, 0);
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
    AS3935_ReadRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);
    
    // Update the gain value while preserving other bits
    AS3935_Sensor.POWER.Val.BitField.GAIN = gain & 0x1F; // Mask to valid range (5 bits)
    
    // Write the updated value back to the register
    AS3935_WriteRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);
}

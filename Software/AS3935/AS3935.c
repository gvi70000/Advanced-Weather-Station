#include "as3935.h"
#include "i2c.h"
#include <stdio.h>
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
 * @brief Sends a direct command to the AS3935 lightning sensor.
 * 
 * Direct commands are used for specific operations, such as oscillator calibration or reset.
 * The command is written to the `AS3935_DIRECT_COMMAND` address to trigger the desired action.
 * 
 * @param command The direct command to send (refer to AS3935 command definitions).
 * @retval HAL_OK     Command sent successfully.
 * @retval HAL_ERROR  Communication failure.
 */
static inline HAL_StatusTypeDef AS3935_SendDirectCommand(uint8_t command) {
    // Write the direct command to the AS3935_DIRECT_COMMAND register
    return AS3935_WriteRegister(AS3935_DIRECT_COMMAND, &command, 1);
}

/**
 * @brief Initializes the AS3935 lightning sensor with default settings.
 * 
 * This function configures the AS3935 sensor using predefined defaults:
 * - Wakes up the sensor.
 * - Configures noise floor level and detection efficiency.
 * - Configures power state and AFE gain.
 * - Sets lightning statistics.
 * - Configures interrupts and frequency.
 * - Calibrates oscillators.
 * 
 * @retval HAL_OK     Initialization successful.
 * @retval HAL_ERROR  Initialization failed at any step.
 */
HAL_StatusTypeDef AS3935_Init(void) {
    // Wake up the sensor
    if (AS3935_WakeUp() != HAL_OK) {
        return HAL_ERROR; // Wake-up failed
    }
    // Configure detection efficiency and noise floor (WDTH = 50%, NF_LEV = 4)
    if (AS3935_ConfigureDetectionAndNoise(EFFICIENCY_100, NOISE_FLOOR_390uVrms) != HAL_OK) {
        return HAL_ERROR; // Failed to configure detection and noise
    }
    // Configure power state to "Power Up" and AFE gain to "Outdoor Mode"
    if (AS3935_ConfigurePowerAndGain(AS3935_POWER_UP, AFE_GAIN_OUTDOOR) != HAL_OK) {
        return HAL_ERROR; // Failed to configure power and gain
    }
    // Configure lightning statistics (SREJ = 1, MIN_NUM_LIGH = 1, do not clear statistics)
    if (AS3935_ConfigureLightningStatistics(SREJ_LEVEL_0, MIN_NUM_LIGHTNING_1, CLEAR_STAT_DISABLED) != HAL_OK) {
        return HAL_ERROR; // Failed to configure lightning statistics
    }
    // Configure interrupts and frequency division ratio (INT = INT_L, mask disturbers, divide frequency by 16)
    if (AS3935_ConfigureInterruptsAndFrequency(INT_L, MASK_DIST_ENABLED, LCO_FDIV_RATIO_16) != HAL_OK) {
        return HAL_ERROR; // Failed to configure interrupts and frequency
    }
    // Calibrate oscillators (TRCO and SRCO)
    if (AS3935_CalibrateOscillators() != HAL_OK) {
        return HAL_ERROR; // Oscillator calibration failed
    }
    return HAL_OK; // Initialization successful
}

/**
 * @brief Wakes up the AS3935 lightning sensor from power-down mode.
 * 
 * Clears the power-down bit and recalibrates the oscillators.
 * 
 * @retval HAL_OK Wake-up successful.
 * @retval HAL_ERROR Wake-up failed.
 */
HAL_StatusTypeDef AS3935_WakeUp(void) {
    // Clear power-down bit and set recommended outdoor gain
    AS3935_Sensor.POWER.Val.BitField.PWD = AS3935_POWER_UP;
    AS3935_Sensor.POWER.Val.BitField.GAIN = AFE_GAIN_OUTDOOR;
    if (AS3935_WriteRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(2); // Wait 2ms for wake-up
    // Calibrate TRCO
    if (AS3935_SendDirectCommand(CALIB_RCO) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(4);

    // Verify calibration status
//    if (AS3935_ReadRegister(CALIB_TRCO, &AS3935_Sensor.TRCO.Val.Value, 1) != HAL_OK || !AS3935_Sensor.TRCO.Val.BitField.TRCO_CALIB_DONE) {
//        return HAL_ERROR; // Calibration failed
//    }
    return HAL_OK;
}

/**
 * @brief Configures the power state and AFE (Analog Front End) gain of the AS3935 sensor.
 * 
 * This function allows simultaneous configuration of the power state and AFE gain
 * by modifying the AFE_GAIN register. It combines the functionality of setting the 
 * power-down bit and adjusting the gain in a single call.
 * 
 * @param pwr  Power state (use AS3935_PowerState_t).
 *             - `POWER_DOWN`: Power down the sensor.
 *             - `POWER_UP`: Power up the sensor.
 * @param gain Gain setting for the AFE (use AS3935_AFE_Gain_t, range 0-31).
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_ConfigurePowerAndGain(AS3935_PowerState_t pwr, AS3935_AFE_Gain_t gain) {
    // Configure the power state
    AS3935_Sensor.POWER.Val.BitField.PWD = pwr;
    // Configure the AFE gain
    AS3935_Sensor.POWER.Val.BitField.GAIN = gain;
    // Write the updated value to the AFE_GAIN register
    return AS3935_WriteRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);
}

/**
 * @brief Configures the detection efficiency and noise floor level of the AS3935 sensor.
 * 
 * This function allows simultaneous configuration of the Watchdog Threshold (WDTH) 
 * and the Noise Floor Level (NF_LEV) by modifying the THRESHOLD register. 
 * 
 * @param wdth  Desired Watchdog Threshold value (use AS3935_WDTH_t).
 *              Values range from 0 to 15 (corresponding to binary 0000 to 1111).
 * @param level Desired Noise Floor Level (use AS3935_NoiseFloorLevel_t).
 *              Values range from 0 to 7, where higher values filter more noise.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_ConfigureDetectionAndNoise(AS3935_WDTH_t wdth, AS3935_NoiseFloorLevel_t level) {
    // Configure the Watchdog Threshold (WDTH)
    AS3935_Sensor.NOISE.Val.BitField.WDTH = wdth;
    // Configure the Noise Floor Level (NF_LEV)
    AS3935_Sensor.NOISE.Val.BitField.NF_LEV = level;
    // Write the updated noise register value to the THRESHOLD register
    return AS3935_WriteRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1);
}

/**
 * @brief Configures the spike rejection level, minimum number of lightning events, and lightning detection statistics.
 * 
 * This function sets the spike rejection threshold (SREJ) and the minimum number of lightning events 
 * (MIN_NUM_LIGH) in register 0x02. Additionally, it allows clearing the lightning detection statistics 
 * by setting the `CL_STAT` bit. If statistics are cleared, the bit is reset to `0` after the operation.
 * 
 * @param srej      Spike rejection level (use AS3935_SREJ_t).
 *                  - Values range from `SREJ_LEVEL_0` (highest detection efficiency) to `SREJ_LEVEL_11` (lowest efficiency).
 * @param minNum    Minimum number of lightning events for validation (use AS3935_MinLightning_t).
 *                  - Values:
 *                    - `MIN_NUM_LIGHTNING_1`: Minimum of 1 lightning event.
 *                    - `MIN_NUM_LIGHTNING_5`: Minimum of 5 lightning events.
 *                    - `MIN_NUM_LIGHTNING_9`: Minimum of 9 lightning events.
 *                    - `MIN_NUM_LIGHTNING_16`: Minimum of 16 lightning events.
 * @param clearStat Lightning detection statistics clearing configuration (use AS3935_ClearStat_t).
 *                  - `CLEAR_STAT_ENABLED`: Clears the statistics.
 *                  - `CLEAR_STAT_DISABLED`: Leaves the statistics unchanged.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure or invalid values.
 */
HAL_StatusTypeDef AS3935_ConfigureLightningStatistics(AS3935_SREJ_t srej, AS3935_MinLightning_t minNum, AS3935_ClearStat_t clearStat) {
    // Configure the Spike Rejection Level (SREJ)
    AS3935_Sensor.STATISTICS.Val.BitField.SREJ = srej;
    // Configure the Minimum Number of Lightning Events (MIN_NUM_LIGH)
    AS3935_Sensor.STATISTICS.Val.BitField.MIN_NUM_LIGH = minNum;
    // Set the Clear Statistics (CL_STAT) bit
    AS3935_Sensor.STATISTICS.Val.BitField.CL_STAT = clearStat;
    // Write the updated configuration to register 0x02
    if (AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1) != HAL_OK) {
        return HAL_ERROR; // Communication error
    }
    // If clearing statistics, reset the CL_STAT bit after the operation
    if (clearStat == CLEAR_STAT_ENABLED) {
        AS3935_Sensor.STATISTICS.Val.BitField.CL_STAT = CLEAR_STAT_DISABLED;
        if (AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1) != HAL_OK) {
            return HAL_ERROR; // Communication error
        }
    }
    return HAL_OK;
}

/**
 * @brief Configures interrupt types, disturber masking, and frequency division ratio for the AS3935 sensor.
 * 
 * This function sets the interrupt type(s), configures whether disturber events are masked, 
 * and sets the frequency division ratio for the antenna. The configuration is written to 
 * register 0x03 (INT_MASK_ANT).
 * 
 * @param interrupt  Desired interrupt type(s) (use AS3935_INT_t).
 *                   Multiple interrupt types can be combined using bitwise OR (e.g., INT_NH | INT_L).
 *                   - INT_NH: Noise level too high.
 *                   - INT_D: Disturber detected.
 *                   - INT_L: Lightning event detected.
 * @param maskDist   Configures whether disturber events are masked (use AS3935_MaskDist_t).
 *                   - MASK_DIST_DISABLED: Do not mask disturber events (INT_D is enabled).
 *                   - MASK_DIST_ENABLED: Mask disturber events (INT_D is disabled).
 * @param fdivRatio  Desired frequency division ratio for the antenna (use AS3935_LCO_FDiv_t).
 *                   - LCO_FDIV_RATIO_16: Divide frequency by 16.
 *                   - LCO_FDIV_RATIO_32: Divide frequency by 32.
 *                   - LCO_FDIV_RATIO_64: Divide frequency by 64.
 *                   - LCO_FDIV_RATIO_128: Divide frequency by 128.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure or invalid values.
 */
HAL_StatusTypeDef AS3935_ConfigureInterruptsAndFrequency(AS3935_INT_t interrupt, AS3935_MaskDist_t maskDist, AS3935_LCO_FDiv_t fdivRatio) { 
    // Configure the interrupt type (INT)
    AS3935_Sensor.INT_FREQ.Val.BitField.INT = interrupt;
    // Configure the MASK_DIST bit
    AS3935_Sensor.INT_FREQ.Val.BitField.MASK_DIST = maskDist;
    // Configure the frequency division ratio (LCO_FDIV)
    AS3935_Sensor.INT_FREQ.Val.BitField.LCO_FDIV = fdivRatio;
    // Write the updated value to register 0x03
    return AS3935_WriteRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);
}

/**
 * @brief Reads lightning energy and distance estimation from registers 0x04 to 0x07.
 * 
 * This function reads the lightning energy (20 bits) and the distance estimation (6 bits) 
 * from the AS3935 sensor. The values are parsed into the `AS3935_Energy_t` structure, 
 * which combines the two data points into an easy-to-use representation.
 * 
 * @param energy Pointer to an `AS3935_Energy_t` structure to store the read values.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_ReadLightningEnergyAndDistance(AS3935_Energy_t *energy) {
    // Read registers 0x04 to 0x07 into the byte array of the structure
    if (AS3935_ReadRegister(ENERGY_LIGHT_LSB, energy->ByteArray, 4) != HAL_OK) {
        return HAL_ERROR; // Communication error
    }
    // Extract the lightning energy (20 bits) and distance estimation (6 bits)
    energy->LightningEnergy = (energy->LSB) | ((uint32_t)(energy->MSB) << 8) | ((uint32_t)(energy->MMSB) << 16);
    energy->DistanceEstimation = energy->Distance;
    return HAL_OK;
}

/**
 * @brief Configures the tuning capacitors and oscillator display settings for the AS3935 sensor.
 * 
 * This function sets the internal tuning capacitor value (TUN_CAP) for antenna optimization and 
 * configures which oscillator(s) (TRCO, SRCO, LCO) are displayed on the IRQ pin for diagnostic purposes.
 * 
 * @param tuningCap Desired tuning capacitor value (use AS3935_TUNE_CAP_t).
 *                  - Values range from `TUN_CAP_0PF` (0 pF) to `TUN_CAP_120PF` (120 pF).
 * @param oscDisplay Desired oscillator display configuration (use AS3935_OSCDisplay_t).
 *                   - Options include `OSC_DISPLAY_DISABLED`, `OSC_DISPLAY_TRCO_ONLY`, etc.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure or invalid values.
 */
HAL_StatusTypeDef AS3935_SetIRQConfig(AS3935_TUNE_CAP_t tuningCap, AS3935_OSCDisplay_t oscDisplay) {
    // Configure the tuning capacitors
    AS3935_Sensor.IRQ.Val.BitField.TUN_CAP = tuningCap;
    // Configure the oscillator display settings
    AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = oscDisplay;
    // Write the updated value to register 0x08
    return AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
}

/**
 * @brief Reads the calibration status of the Timer RC Oscillator (TRCO) into `AS3935_Sensor.TRCO`.
 * 
 * This function reads the TRCO calibration status from register 0x3A and stores it in the 
 * `AS3935_Sensor.TRCO` structure. It provides detailed information about whether the calibration 
 * was successful or not.
 * 
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_ReadTRCOCalibrationStatus(void) {
    // Read register 0x3A into AS3935_Sensor.TRCO
    return AS3935_ReadRegister(CALIB_TRCO, &AS3935_Sensor.TRCO.Val.Value, 1);
}

/**
 * @brief Reads the calibration status of the System RC Oscillator (SRCO) into `AS3935_Sensor.SRCO`.
 * 
 * This function reads the SRCO calibration status from register 0x3B and stores it in the 
 * `AS3935_Sensor.SRCO` structure. It provides detailed information about whether the calibration 
 * was successful or not.
 * 
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_ReadSRCOCalibrationStatus(void) {
    // Read register 0x3B into AS3935_Sensor.SRCO
    return AS3935_ReadRegister(CALIB_SRCO, &AS3935_Sensor.SRCO.Val.Value, 1);
}

/**
 * @brief Calibrates the oscillators (TRCO and SRCO) of the AS3935 lightning sensor.
 * 
 * This function sends the calibration command, verifies the calibration status for TRCO and SRCO,
 * sets TRCO on the IRQ line for measurement, and checks for completion of the calibration process.
 * 
 * @retval HAL_OK     Calibration successful.
 * @retval HAL_ERROR  Calibration failed or communication error.
 */
HAL_StatusTypeDef AS3935_CalibrateOscillators(void) {
    HAL_StatusTypeDef status;

    // Send the calibration command
    status = AS3935_SendDirectCommand(CALIB_RCO);
    if (status != HAL_OK) {
        return HAL_ERROR; // Command failed
    }

    // Wait for calibration to complete
    HAL_Delay(2);

    // Enable TRCO display on IRQ pin for measurement
    status = AS3935_ReadRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
    if (status != HAL_OK) {
        return HAL_ERROR; // Failed to read IRQ register
    }
    AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = OSC_DISPLAY_TRCO_ONLY;
    status = AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
    if (status != HAL_OK) {
        return HAL_ERROR; // Failed to enable TRCO display
    }

    // Continuously poll the TRCO and SRCO calibration status until both are done
    for (uint8_t attempt = 0; attempt < 10; ++attempt) {
        // Verify TRCO calibration status
        status = AS3935_ReadTRCOCalibrationStatus();
        if (status != HAL_OK) {
            return HAL_ERROR; // Communication error
        }

        // Verify SRCO calibration status
        status = AS3935_ReadSRCOCalibrationStatus();
        if (status != HAL_OK) {
            return HAL_ERROR; // Communication error
        }

        // Check if both TRCO and SRCO are calibrated
        if (AS3935_Sensor.TRCO.Val.BitField.TRCO_CALIB_DONE && AS3935_Sensor.SRCO.Val.BitField.SRCO_CALIB_DONE) {
            // Disable TRCO display on IRQ pin after successful calibration
            AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = 0;
            status = AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
            return status == HAL_OK ? HAL_OK : HAL_ERROR; // Calibration successful if write succeeds
        }

        // Delay before polling again
        HAL_Delay(1);
    }

    // Disable TRCO display on IRQ pin if calibration failed
    AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = 0;
    status = AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);

    return HAL_ERROR; // Calibration failed
}

/**
 * @brief Recalibrates the AS3935 sensor after it has been powered down.
 * 
 * This function sends the CALIB_RCO command to recalibrate the internal oscillators
 * (TRCO and SRCO) and briefly displays the TRCO oscillator on the IRQ pin for validation.
 * It uses the latest functions, enums, and structures for improved readability and reliability.
 * 
 * @retval HAL_OK     Recalibration successful.
 * @retval HAL_ERROR  Communication failure or calibration error.
 */
HAL_StatusTypeDef AS3935_RecalibrateAfterPowerDown(void) {
    // Send calibration command to recalibrate internal oscillators
    if (AS3935_SendDirectCommand(CALIB_RCO) != HAL_OK) {
        return HAL_ERROR; // Calibration command failed
    }
    // Allow time for calibration to complete
    HAL_Delay(2);
    // Enable TRCO display on IRQ pin for validation
    if (AS3935_ReadRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to read IRQ register
    }
    // Set the oscillator display configuration to display TRCO only
    if (AS3935_SetIRQConfig(AS3935_Sensor.IRQ.Val.BitField.TUN_CAP, OSC_DISPLAY_TRCO_ONLY) != HAL_OK) {
        return HAL_ERROR; // Failed to configure TRCO display
    }
    // Allow time for TRCO display to stabilize
    HAL_Delay(2);
    // Disable TRCO display on IRQ pin
    if (AS3935_SetIRQConfig(AS3935_Sensor.IRQ.Val.BitField.TUN_CAP, OSC_DISPLAY_DISABLED) != HAL_OK) {
        return HAL_ERROR; // Failed to disable TRCO display
    }
    return HAL_OK; // Recalibration successful
}

/**
 * @brief Loops through all possible tuning capacitor values for the AS3935 sensor.
 * 
 * This function iterates through all `TUN_CAP` values, setting each one sequentially and 
 * allowing observation of the sensor's behavior at each step. It automatically covers the 
 * full range defined in the `AS3935_TUNE_CAP_t` enum.
 * 
 * @param delayMs Delay in milliseconds between each tuning step (for evaluation purposes).
 *                Use 0 for no delay.
 */
void AS3935_TuneCapacitorsWithLoop(uint32_t delayMs) {
    // Loop through all tuning capacitor values
    for (AS3935_TUNE_CAP_t tuningCap = TUN_CAP_0PF; tuningCap <= TUN_CAP_120PF; tuningCap++) {
        // Read the current IRQ register value
        if (AS3935_ReadRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1) != HAL_OK) {
            printf("Failed to read register for TUN_CAP %d pF.\n", tuningCap * 8);
            continue; // Skip to the next value
        }
        // Set the tuning capacitor value
        AS3935_Sensor.IRQ.Val.BitField.TUN_CAP = tuningCap;
        // Write the updated value back to the IRQ register
        if (AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1) != HAL_OK) {
            printf("Failed to set TUN_CAP to %d pF.\n", tuningCap * 8);
            continue; // Skip to the next value
        }
        // Display the current tuning capacitor value
        printf("Tuning capacitors set to %d pF.\n", tuningCap * 8);
        // Optional delay for testing or observation
        if (delayMs > 0) {
            HAL_Delay(delayMs);
        }
    }
}
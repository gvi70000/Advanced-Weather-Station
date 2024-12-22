#include "AS3935.h"
#include "i2c.h"
#include "gpio.h"
#include "tim.h"
#include <stdio.h>
#include <stdlib.h>
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
 * @param commandReg The direct command to send (refer to AS3935 command definitions).
 * @retval HAL_OK     Command sent successfully.
 * @retval HAL_ERROR  Communication failure.
 */
static inline HAL_StatusTypeDef AS3935_SendDirectCommand(uint8_t commandReg) {
    // Write the direct command (0x96) to the specified register
		return AS3935_WriteRegister(commandReg, (uint8_t*)AS3935_DIRECT_COMMAND, 1);
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
    // Clear power-down bit and set recommended outdoor gain
    AS3935_Sensor.POWER.Val.BitField.PWD = AS3935_POWER_UP;
    AS3935_Sensor.POWER.Val.BitField.GAIN = AFE_GAIN_INDOOR;
    if (AS3935_WriteRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(AS3935_CMD_DELAY); // Wait 2ms for wake-up
		
		AS3935_ReadAllRegisters();

		if (AS3935_setTuningCapacitor(TUN_CAP_64PF) != HAL_OK) {
        return HAL_ERROR;
    }
	
		AS3935_Sensor.NOISE.Val.BitField.NF_LEV = NOISE_FLOOR_390uVrms;
		AS3935_Sensor.NOISE.Val.BitField.WDTH = EFFICIENCY_100;
    if (AS3935_WriteRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to configure detection and noise
    }
		AS3935_Sensor.STATISTICS.Val.BitField.SREJ = SREJ_LEVEL_0;
		AS3935_Sensor.STATISTICS.Val.BitField.MIN_NUM_LIGH = MIN_NUM_LIGHTNING_1;
    if (AS3935_ClearStatistics(CLEAR_STAT_ENABLED) != HAL_OK) {
        return HAL_ERROR; // Failed to configure lightning statistics
    }
		AS3935_Sensor.INT_FREQ.Val.BitField.INT = INT_L;
		AS3935_Sensor.INT_FREQ.Val.BitField.MASK_DIST = MASK_DIST_DISABLED;
		AS3935_Sensor.INT_FREQ.Val.BitField.LCO_FDIV = LCO_FDIV_RATIO_16;
    // Configure interrupts and frequency division ratio (INT = INT_L, show disturbers, divide frequency by 16)
    if (AS3935_WriteRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to configure interrupts and frequency
    }
		HAL_Delay(AS3935_CMD_DELAY); // Wait 250ms for calibration
    // Send calibrate command
    if (AS3935_SendDirectCommand(CALIB_RCO) != HAL_OK) {
        return HAL_ERROR;
    }
		if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_SRCO_ONLY) != HAL_OK) {
        return HAL_ERROR; // Failed to configure interrupts and frequency
    }
    HAL_Delay(250); // Wait 250ms for calibration
		if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_DISABLED) != HAL_OK) {
        return HAL_ERROR; // Failed to configure interrupts and frequency
    }
    // Verify calibration status
		// ToDo! Check what is wrong with the frequency
//    if (AS3935_ReadRegister(CALIB_SRCO, &AS3935_Sensor.SRCO.Val.Value, 1) != HAL_OK || !AS3935_Sensor.SRCO.Val.BitField.SRCO_CALIB_DONE) {
//        return HAL_ERROR; // Calibration failed
//    }
    return HAL_OK; // Initialization successful
}

/**
 * @brief Reads all the AS3935 sensor registers into the AS3935_Sensor structure.
 *
 * @retval HAL_OK     Operation was successful, and all registers were read.
 * @retval HAL_ERROR  Reading one or more registers failed.
 */
HAL_StatusTypeDef AS3935_ReadAllRegisters(void) {
    HAL_StatusTypeDef status;

    // Read register 0x00 (POWER)
    status = AS3935_ReadRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    // Read register 0x01 (NOISE)
    status = AS3935_ReadRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    // Read register 0x02 (STATISTICS)
    status = AS3935_ReadRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    // Read register 0x03 (INT_FREQ)
    status = AS3935_ReadRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    // Read registers 0x04 to 0x07 (ENERGY)
    status = AS3935_ReadRegister(ENERGY_LIGHT_LSB, AS3935_Sensor.ENERGY.ByteArray, 4);
    if (status != HAL_OK) return HAL_ERROR;

    // Read register 0x08 (IRQ)
    status = AS3935_ReadRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    // Read register 0x3A (TRCO)
    status = AS3935_ReadRegister(CALIB_TRCO, &AS3935_Sensor.TRCO.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    // Read register 0x3B (SRCO)
    status = AS3935_ReadRegister(CALIB_SRCO, &AS3935_Sensor.SRCO.Val.Value, 1);
    if (status != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

/**
 * @brief Configures the power state of the AS3935 sensor.
 * 
 * @param pwr The desired power state (e.g., power-down or normal operation).
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetPower(AS3935_PowerState_t powerState) {
    AS3935_Sensor.POWER.Val.BitField.PWD = powerState;
    return AS3935_WriteRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);
}

/**
 * @brief Sets the AFE gain of the AS3935 sensor.
 * 
 * @param gain The desired AFE gain level.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetGain(AS3935_AFE_Gain_t gain) {
    AS3935_Sensor.POWER.Val.BitField.GAIN = gain;
    return AS3935_WriteRegister(AFE_GAIN, &AS3935_Sensor.POWER.Val.Value, 1);
}

/**
 * @brief Configures the watchdog threshold of the AS3935 sensor.
 * 
 * @param wdth The desired watchdog threshold.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetWatchdog(AS3935_WDTH_t watchdogThreshold) {
    AS3935_Sensor.NOISE.Val.BitField.WDTH = watchdogThreshold;
    return AS3935_WriteRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1);
}

/**
 * @brief Sets the noise floor level of the AS3935 sensor.
 * 
 * @param level The desired noise floor level.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetNoise(AS3935_NoiseFloorLevel_t noiseLevel) {
    AS3935_Sensor.NOISE.Val.BitField.NF_LEV = noiseLevel;
    return AS3935_WriteRegister(THRESHOLD, &AS3935_Sensor.NOISE.Val.Value, 1);
}

/**
 * @brief Configures the spike rejection level of the AS3935 sensor.
 * 
 * @param srej The desired spike rejection level.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetSpikeRejectionLevel(AS3935_SREJ_t spikeRejectionLevel) {
    AS3935_Sensor.STATISTICS.Val.BitField.SREJ = spikeRejectionLevel;
    return AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1);
}

/**
 * @brief Sets the minimum number of lightning events required for validation.
 * 
 * @param minNum The minimum number of lightning events.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetLightningNo(AS3935_MinLightning_t noLightningEvents) {
    AS3935_Sensor.STATISTICS.Val.BitField.MIN_NUM_LIGH = noLightningEvents;
    return AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1);
}

/**
 * @brief Clears the lightning statistics in the AS3935 sensor.
 * 
 * @param clearStat The clear statistics command.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_ClearStatistics(AS3935_ClearStat_t clearStat) {
    AS3935_Sensor.STATISTICS.Val.BitField.CL_STAT = clearStat;
    if (AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (clearStat == CLEAR_STAT_ENABLED) {
        AS3935_Sensor.STATISTICS.Val.BitField.CL_STAT = CLEAR_STAT_DISABLED;
        if (AS3935_WriteRegister(LIGHTNING_REG, &AS3935_Sensor.STATISTICS.Val.Value, 1) != HAL_OK) {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

/**
 * @brief Sets the type of interrupt generated by the AS3935 sensor.
 * 
 * @param interrupt The desired interrupt type.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetInterruptType(AS3935_INT_t interrupt) {
    AS3935_Sensor.INT_FREQ.Val.BitField.INT = interrupt;
    return AS3935_WriteRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);
}

/**
 * @brief Configures the disturber mask in the AS3935 sensor.
 * 
 * @param maskDist The desired disturber mask state.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetdDisturberMask(AS3935_MaskDist_t maskDist) {
    AS3935_Sensor.INT_FREQ.Val.BitField.MASK_DIST = maskDist;
    return AS3935_WriteRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);
}

/**
 * @brief Sets the frequency division ratio for the LCO signal.
 * 
 * @param fdivRatio The desired frequency division ratio.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_setFrequencyDivisionRatio(AS3935_LCO_FDiv_t fdivRatio) {
    AS3935_Sensor.INT_FREQ.Val.BitField.LCO_FDIV = fdivRatio;
    return AS3935_WriteRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1);
}

/**
 * @brief Reads lightning energy and distance estimation from registers 0x04 to 0x07.
 * 
 * This function reads the lightning energy (20 bits) and the distance estimation (6 bits) 
 * from the AS3935 sensor. The values are parsed into the `AS3935_Energy_t` structure, 
 * which combines the two data points into an easy-to-use representation. It also reads 
 * the interrupt register to ensure a new interrupt is triggered for the next event.
 * 
 * @param energy Pointer to an `AS3935_Energy_t` structure to store the read values.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_ReadLightningEnergyAndDistance(AS3935_Energy_t *energy) {
    // Read the interrupt register (0x03) to clear any existing interrupts
    if (AS3935_ReadRegister(INT_MASK_ANT, &AS3935_Sensor.INT_FREQ.Val.Value, 1) != HAL_OK) {
        return HAL_ERROR; // Communication error
    }

    // Check if we received a lightning interrupt
    if (AS3935_Sensor.INT_FREQ.Val.BitField.INT == INT_L) { // INT_L corresponds to lightning detection
        // Read registers 0x04 to 0x07 into the byte array of the structure
        if (AS3935_ReadRegister(ENERGY_LIGHT_LSB, energy->ByteArray, 4) != HAL_OK) {
            return HAL_ERROR; // Communication error
        }

        // Extract the lightning energy (20 bits) and distance estimation (6 bits)
        energy->LightningEnergy = (energy->LSB) | ((uint32_t)(energy->MSB) << 8) | ((uint32_t)(energy->MMSB) << 16);
        energy->DistanceEstimation = energy->Distance;
    }

    return HAL_OK;
}

/**
 * @brief Configures the tuning capacitors for the AS3935 sensor.
 * 
 * @param tuningCap The desired tuning capacitor value.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_setTuningCapacitor(AS3935_TUNE_CAP_t tuningCap) {
    AS3935_Sensor.IRQ.Val.BitField.TUN_CAP = tuningCap;
    return AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
}

/**
 * @brief Configures the oscillator display settings.
 * 
 * @param oscDisplay The desired oscillator display setting.
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_SetOscillatorDisplay(AS3935_OSCDisplay_t oscDisplay) {
    AS3935_Sensor.IRQ.Val.BitField.OSC_DISP = oscDisplay;
    return AS3935_WriteRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1);
}

/**
 * @brief Reads the TRCO calibration status.
 * 
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_ReadTRCOCalibrationStatus(void) {
    return AS3935_ReadRegister(CALIB_TRCO, &AS3935_Sensor.TRCO.Val.Value, 1);
}

/**
 * @brief Reads the SRCO calibration status.
 * 
 * @retval HAL_OK     Operation successful.
 * @retval HAL_ERROR  Communication failure.
 */
HAL_StatusTypeDef AS3935_ReadSRCOCalibrationStatus(void) {
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
    // Set the IRQ pin to display TRCO
    if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_TRCO_ONLY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Send the RCO calibration command
    if (AS3935_SendDirectCommand(CALIB_RCO) != HAL_OK) {
        return HAL_ERROR;
    }

    // Wait for calibration to complete
    HAL_Delay(AS3935_CMD_DELAY); // Increase delay for stabilization

    // Validate TRCO calibration
    if (AS3935_ReadRegister(CALIB_TRCO, &AS3935_Sensor.TRCO.Val.Value, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (!AS3935_Sensor.TRCO.Val.BitField.TRCO_CALIB_DONE) {
        printf("TRCO calibration failed.\n");
        return HAL_ERROR;
    }

    // Repeat for SRCO
    if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_SRCO_ONLY) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(AS3935_CMD_DELAY);
    if (AS3935_ReadRegister(CALIB_SRCO, &AS3935_Sensor.SRCO.Val.Value, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (!AS3935_Sensor.SRCO.Val.BitField.SRCO_CALIB_DONE) {
        printf("SRCO calibration failed.\n");
        return HAL_ERROR;
    }

    // Reset IRQ pin to normal interrupt mode
    if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_DISABLED) != HAL_OK) {
        return HAL_ERROR;
    }

    printf("Oscillator calibration successful.\n");
    return HAL_OK;
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
    // Enable TRCO display on IRQ pin for validation
    if (AS3935_ReadRegister(FREQ_DISP_IRQ, &AS3935_Sensor.IRQ.Val.Value, 1) != HAL_OK) {
        return HAL_ERROR; // Failed to read IRQ register
    }
    // Set the oscillator display configuration to display TRCO only
    if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_TRCO_ONLY) != HAL_OK) {
        return HAL_ERROR; // Failed to configure TRCO display
    }
    // Allow time for TRCO display to stabilize
    HAL_Delay(2);
    // Disable TRCO display on IRQ pin
    if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_DISABLED) != HAL_OK) {
        return HAL_ERROR; // Failed to disable TRCO display
    }
    return HAL_OK; // Recalibration successful
}

static uint32_t MeasureFrequencyWithTimer(void) {
    uint32_t captureValue[101] = {0}; // Array to store 101 capture values
    const uint32_t timerClock = 72000000; // TIM3 clock frequency (72 MHz)
    uint32_t totalPeriod = 0;

    // Start TIM3 in input capture mode
    HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_3);

    // Capture 101 consecutive rising edges
    for (uint8_t i = 0; i < 101; i++) {
        while (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC3) == RESET) {
            // Wait indefinitely for the capture event
        }
        captureValue[i] = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC3);
    }

    // Stop TIM3
    HAL_TIM_IC_Stop(&htim3, TIM_CHANNEL_3);

    // Calculate total period over 101 captures
    for (uint8_t i = 0; i < 100; i++) { // 100 intervals between 101 captures
        if (captureValue[i + 1] >= captureValue[i]) {
            totalPeriod += (captureValue[i + 1] - captureValue[i]);
        } else {
            totalPeriod += ((htim3.Instance->ARR + 1) - captureValue[i] + captureValue[i + 1]);
        }
    }

    // Calculate average period
    uint32_t averagePeriod = totalPeriod / 100; // 100 intervals for 101 captures

    // Calculate frequency
    return (averagePeriod > 0) ? (timerClock / averagePeriod) : 0; // Avoid division by zero
}

///*
//1. Enable TIM3, set it to 4MHz in Input Capture
//2. Measure the time between the 2 rising edges, or between 2 falling edges
//*/

uint8_t AS3935_TuneAntenna(void) {
    uint8_t optimalTuningCap = 0xFF; // Default to failure
    uint32_t measuredFreq = 0;
    int32_t minErrorSRCO = INT32_MAX;
    int32_t minErrorTRCO = INT32_MAX;
    int32_t minErrorLCO = INT32_MAX;
    const uint16_t stabilizationTime = 200; // Allow frequency to stabilize
    const uint32_t targetFreqLCO = 500000;   // Target fLCO in Hz
    const uint16_t toleranceLCO = 17500;     // fLCO tolerance in Hz
    const uint16_t targetFreqTRCO = 32250;   // Target fTRCO in Hz
    const uint16_t toleranceTRCO = 1750;     // fTRCO tolerance in Hz
    const uint32_t targetFreqSRCO = 1125000; // Target fSRCO in Hz
    const uint16_t toleranceSRCO = 65000;    // fSRCO tolerance in Hz

    // Set up TIM3
    MX_TIM3_Init();

    for (AS3935_TUNE_CAP_t tuningCap = TUN_CAP_0PF; tuningCap <= TUN_CAP_120PF; tuningCap++) {
        int32_t errorSRCO, errorTRCO, errorLCO;

        // Tuning for SRCO
        if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_SRCO_ONLY) != HAL_OK) {
            printf("Failed to set TUN_CAP for SRCO: %d pF\n", tuningCap * 8);
            continue;
        }

        HAL_Delay(stabilizationTime); // Allow stabilization

        measuredFreq = MeasureFrequencyWithTimer();
        errorSRCO = abs((int32_t)measuredFreq - (int32_t)targetFreqSRCO);

        // Verify SRCO calibration
        if (AS3935_ReadRegister(CALIB_SRCO, &AS3935_Sensor.SRCO.Val.Value, 1) != HAL_OK || 
            !AS3935_Sensor.SRCO.Val.BitField.SRCO_CALIB_DONE) {
            printf("SRCO calibration failed for TUN_CAP: %d pF\n", tuningCap * 8);
            //continue;
        }

        if (errorSRCO < minErrorSRCO && errorSRCO <= (int32_t)toleranceSRCO) {
            minErrorSRCO = errorSRCO;
            optimalTuningCap = tuningCap;
        }

        printf("TUN_CAP: %d pF, SRCO Measured: %d Hz, Error: %d Hz\n", tuningCap * 8, measuredFreq, errorSRCO);

        // Disable oscillator display
        if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_DISABLED) != HAL_OK) {
            printf("Failed to disable SRCO display.\n");
            continue;
        }

        // Tuning for TRCO
        if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_TRCO_ONLY) != HAL_OK) {
            printf("Failed to set TUN_CAP for TRCO: %d pF\n", tuningCap * 8);
            continue;
        }

        HAL_Delay(stabilizationTime); // Allow stabilization

        measuredFreq = MeasureFrequencyWithTimer();
        errorTRCO = abs((int32_t)measuredFreq - (int32_t)targetFreqTRCO);

        // Verify TRCO calibration
        if (AS3935_ReadRegister(CALIB_TRCO, &AS3935_Sensor.TRCO.Val.Value, 1) != HAL_OK || 
            !AS3935_Sensor.TRCO.Val.BitField.TRCO_CALIB_DONE) {
            printf("TRCO calibration failed for TUN_CAP: %d pF\n", tuningCap * 8);
            //continue;
        }

        if (errorTRCO < minErrorTRCO && errorTRCO <= (int32_t)toleranceTRCO) {
            minErrorTRCO = errorTRCO;
            optimalTuningCap = tuningCap;
        }

        printf("TUN_CAP: %d pF, TRCO Measured: %d Hz, Error: %d Hz\n", tuningCap * 8, measuredFreq, errorTRCO);

        // Disable oscillator display
        if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_DISABLED) != HAL_OK) {
            printf("Failed to disable TRCO display.\n");
            continue;
        }

        // Tuning for LCO
        if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_LCO_ONLY) != HAL_OK) {
            printf("Failed to set TUN_CAP for LCO: %d pF\n", tuningCap * 8);
            continue;
        }

        HAL_Delay(stabilizationTime); // Allow stabilization

        measuredFreq = MeasureFrequencyWithTimer() * 16; // Multiply by 16 for LCO
        errorLCO = abs((int32_t)measuredFreq - (int32_t)targetFreqLCO);

        if (errorLCO < minErrorLCO && errorLCO <= (int32_t)toleranceLCO) {
            minErrorLCO = errorLCO;
            optimalTuningCap = tuningCap;
        }

        printf("TUN_CAP: %d pF, LCO Measured: %d Hz, Error: %d Hz\n", tuningCap * 8, measuredFreq, errorLCO);

        // Disable oscillator display
        if (AS3935_SetOscillatorDisplay(OSC_DISPLAY_DISABLED) != HAL_OK) {
            printf("Failed to disable LCO display.\n");
            continue;
        }
    }

    // Log the final tuning result
    if (optimalTuningCap != 0xFF) {
        printf("Optimal TUN_CAP: %d pF\n", optimalTuningCap * 8);
    } else {
        printf("Failed to tune the antenna.\n");
    }

    // Restore IRQ pin to interrupt mode
    Init_IntAS3935();

    return optimalTuningCap;
}





#include "as3935.h"
#include "i2c.h"

// Initialize the AS3935 lightning sensor
HAL_StatusTypeDef AS3935_Init(void) {
    HAL_StatusTypeDef status;

    // Wake up the sensor
    if (!AS3935_WakeUp()) {
        return HAL_ERROR; // Wakeup failed
    }

    // Set default Noise Floor Level (0-7, higher is less sensitive to noise)
    AS3935_SetNoiseFloorLevel(4); // Example: Set to level 2

    // Set AFE gain (e.g., indoor vs outdoor mode)
    AS3935_SetAFEGain(1); // Example: Gain = 3 (for indoor conditions)

    // Set default lightning event threshold
    AS3935_SetLightningEventThreshold(1); // Example: Minimum lightning threshold

    // Recalibrate oscillators
    if (!AS3935_CalibrateOscillators()) {
        return HAL_ERROR; // Calibration failed
    }

    // Enable antenna frequency display (optional, for debugging)
    AS3935_EnableAntennaFrequencyDisplay(false);

    return HAL_OK; // Initialization successful
}

// Power management
void AS3935_PowerDown(void) {
    uint8_t pwrReg;
    ReadRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &pwrReg, 1, &hi2c2);
    pwrReg |= 0x01; // Set bit 0 for power down
    WriteRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &pwrReg, 1, &hi2c2);
}

bool AS3935_WakeUp(void) {
    uint8_t pwrReg;
    ReadRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &pwrReg, 1, &hi2c2);
    pwrReg &= ~0x01; // Clear bit 0 to wake up
    WriteRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &pwrReg, 1, &hi2c2);

    HAL_Delay(2); // Wait 2ms for wake-up

    // Calibrate TRCO
    AS3935_SendDirectCommand(CALIB_RCO);
    HAL_Delay(2);

    uint8_t trcoStatus;
    ReadRegister(AS3935_I2C_ADDRESS, CALIB_TRCO, &trcoStatus, 1, &hi2c2);
    return (trcoStatus & 0x80) != 0;
}

void AS3935_RecalibrateAfterPowerDown(void) {
    AS3935_SendDirectCommand(CALIB_RCO);
    HAL_Delay(2);
    AS3935_DisplayOscillator(true, 5); // Enable TRCO display
    HAL_Delay(2);
    AS3935_DisplayOscillator(false, 5); // Disable TRCO display
}

// Oscillator management
bool AS3935_CalibrateOscillators(void) {
    AS3935_SendDirectCommand(CALIB_RCO);
    HAL_Delay(2);

    uint8_t trcoStatus, srcoStatus;
    ReadRegister(AS3935_I2C_ADDRESS, CALIB_TRCO, &trcoStatus, 1, &hi2c2);
    ReadRegister(AS3935_I2C_ADDRESS, CALIB_SRCO, &srcoStatus, 1, &hi2c2);

    return (trcoStatus & 0x80) && (srcoStatus & 0x80);
}

void AS3935_DisplayOscillator(bool enable, uint8_t oscillator) {
    uint8_t irqReg;
    ReadRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &irqReg, 1, &hi2c2);
    if (enable) {
        irqReg |= (1 << (oscillator + 5));
    } else {
        irqReg &= ~(1 << (oscillator + 5));
    }
    WriteRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &irqReg, 1, &hi2c2);
}

// Antenna tuning
void AS3935_SetTuningCapacitance(uint8_t capacitance) {
    uint8_t irqReg;
    ReadRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &irqReg, 1, &hi2c2);
    irqReg = (irqReg & ~0x0F) | (capacitance & 0x0F);
    WriteRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &irqReg, 1, &hi2c2);
}

void AS3935_SetFrequencyDivision(uint8_t divisionRatio) {
    uint8_t intFreqReg;
    ReadRegister(AS3935_I2C_ADDRESS, INT_MASK_ANT, &intFreqReg, 1, &hi2c2);
    intFreqReg = (intFreqReg & ~0xC0) | ((divisionRatio & 0x03) << 6);
    WriteRegister(AS3935_I2C_ADDRESS, INT_MASK_ANT, &intFreqReg, 1, &hi2c2);
}

void AS3935_EnableAntennaFrequencyDisplay(bool enable) {
    uint8_t irqReg;
    ReadRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &irqReg, 1, &hi2c2);
    if (enable) {
        irqReg |= 0x80;
    } else {
        irqReg &= ~0x80;
    }
    WriteRegister(AS3935_I2C_ADDRESS, FREQ_DISP_IRQ, &irqReg, 1, &hi2c2);
}

// Distance estimation
AS3935_Distance_t AS3935_GetDistanceToStorm(void) {
    uint8_t distance;
    ReadRegister(AS3935_I2C_ADDRESS, DISTANCE, &distance, 1, &hi2c2);
    distance &= AS3935_DISTANCE_MASK;
		return distance;
}

uint32_t AS3935_GetLightningEnergy(void) {
    uint8_t energyLSB, energyMSB, energyMMSB;
    ReadRegister(AS3935_I2C_ADDRESS, ENERGY_LIGHT_LSB, &energyLSB, 1, &hi2c2);
    ReadRegister(AS3935_I2C_ADDRESS, ENERGY_LIGHT_MSB, &energyMSB, 1, &hi2c2);
    ReadRegister(AS3935_I2C_ADDRESS, ENERGY_LIGHT_MMSB, &energyMMSB, 1, &hi2c2);

    // Combine the 3 bytes into a 20-bit value
    uint32_t energy = (energyMMSB & 0x1F) << 16 | (energyMSB << 8) | energyLSB;
    return energy;
}

// Noise management
void AS3935_SetNoiseFloorLevel(uint8_t level) {
    uint8_t noiseReg;
    ReadRegister(AS3935_I2C_ADDRESS, THRESHOLD, &noiseReg, 1, &hi2c2);
    noiseReg = (noiseReg & ~0x70) | ((level & 0x07) << 4);
    WriteRegister(AS3935_I2C_ADDRESS, THRESHOLD, &noiseReg, 1, &hi2c2);
}

uint8_t AS3935_ReadNoiseFloorLevel(void) {
    uint8_t noiseReg;
    ReadRegister(AS3935_I2C_ADDRESS, THRESHOLD, &noiseReg, 1, &hi2c2);
    return (noiseReg >> 4) & 0x07;
}

// Interrupt management
void AS3935_MaskDisturber(bool mask) {
    uint8_t intReg;
    ReadRegister(AS3935_I2C_ADDRESS, INT_MASK_ANT, &intReg, 1, &hi2c2);
    if (mask) {
        intReg |= 0x20;
    } else {
        intReg &= ~0x20;
    }
    WriteRegister(AS3935_I2C_ADDRESS, INT_MASK_ANT, &intReg, 1, &hi2c2);
}

uint8_t AS3935_ReadInterruptStatus(void) {
    uint8_t intReg;
    ReadRegister(AS3935_I2C_ADDRESS, INT_MASK_ANT, &intReg, 1, &hi2c2);
    return intReg & 0x0F;
}

// Lightning detection and statistics
void AS3935_SetLightningEventThreshold(uint8_t threshold) {
    uint8_t lightningReg;
    ReadRegister(AS3935_I2C_ADDRESS, LIGHTNING_REG, &lightningReg, 1, &hi2c2);
    lightningReg = (lightningReg & ~0x30) | ((threshold & 0x03) << 4);
    WriteRegister(AS3935_I2C_ADDRESS, LIGHTNING_REG, &lightningReg, 1, &hi2c2);
}

void AS3935_ClearStatistics(void) {
    uint8_t lightningReg;
    ReadRegister(AS3935_I2C_ADDRESS, LIGHTNING_REG, &lightningReg, 1, &hi2c2);
    lightningReg |= 0x40; // Set CL_STAT bit
    WriteRegister(AS3935_I2C_ADDRESS, LIGHTNING_REG, &lightningReg, 1, &hi2c2);
    HAL_Delay(1);
    lightningReg &= ~0x40; // Clear CL_STAT bit
    WriteRegister(AS3935_I2C_ADDRESS, LIGHTNING_REG, &lightningReg, 1, &hi2c2);
}

// Direct commands
void AS3935_SendDirectCommand(uint8_t command) {
    WriteRegister(AS3935_I2C_ADDRESS, command, NULL, 0, &hi2c2);
}

// AFE Gain configuration
void AS3935_SetAFEGain(uint8_t gain) {
    uint8_t pwrReg;
    ReadRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &pwrReg, 1, &hi2c2);
    pwrReg = (pwrReg & ~0x3E) | ((gain & 0x1F) << 1);
    WriteRegister(AS3935_I2C_ADDRESS, AFE_GAIN, &pwrReg, 1, &hi2c2);
}

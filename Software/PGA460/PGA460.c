#include <string.h>
#include "usart.h"
#include "PGA460_REG.h"
#include "Transducers.h"
#include "PGA460.h"

// Initialize the array of ultrasonic sensors
PGA460_Sensor_t myUltraSonicArray[ULTRASONIC_SENSOR_COUNT];

// Helper function to calculate checksum for a data frame
static uint8_t PGA460_CalculateChecksum(const uint8_t *data, uint16_t length) {
    uint16_t sum = 0;
    for (uint16_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return ~((uint8_t)(sum & 0xFF));
}

// Function to send a command to the PGA460
static HAL_StatusTypeDef PGA460_SendData(uint8_t sensorID, PGA460_Command_t command, const uint8_t *data, uint8_t dataSize) {
    if (sensorID >= ULTRASONIC_SENSOR_COUNT) {
        return HAL_ERROR; // Invalid sensor ID
    }

    uint8_t frame[64];
    frame[0] = PGA460_SYNC;      // Sync byte
    frame[1] = (uint8_t)command; // Command byte

    if (data && dataSize > 0) {
        memcpy(&frame[2], data, dataSize);
    }

    frame[dataSize + 2] = PGA460_CalculateChecksum(&frame[1], dataSize + 1); // Append checksum
    return HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, frame, dataSize + 3, HAL_MAX_DELAY);
}

// Function to receive a response from the PGA460
static HAL_StatusTypeDef PGA460_ReceiveData(uint8_t sensorID, uint8_t *response, uint8_t responseSize) {
    if (sensorID >= ULTRASONIC_SENSOR_COUNT) {
        return HAL_ERROR; // Invalid sensor ID
    }
    return HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, response, responseSize, HAL_MAX_DELAY);
}

// Function to initialize 3x PGA460 sensors
void PGA460_Init() {
		
    myUltraSonicArray[0].PGA460_Data = transducer;
		myUltraSonicArray[0].uartPort = &huart1;
	  myUltraSonicArray[1].PGA460_Data = transducer;
		myUltraSonicArray[1].uartPort = &huart4;
	  myUltraSonicArray[2].PGA460_Data = transducer;
		myUltraSonicArray[2].uartPort = &huart5;
}

// Register read function
HAL_StatusTypeDef PGA460_RegisterRead(uint8_t sensorID, uint8_t regAddr, uint8_t *regValue) {
    uint8_t data[] = { regAddr };
    HAL_StatusTypeDef status = PGA460_SendData(sensorID, PGA460_CMD_REGISTER_READ, data, 1);
    if (status != HAL_OK) return status;

    return PGA460_ReceiveData(sensorID, regValue, 1);
}

// Register write function
HAL_StatusTypeDef PGA460_RegisterWrite(uint8_t sensorID, uint8_t regAddr, uint8_t regValue) {
    uint8_t data[] = { regAddr, regValue };
    return PGA460_SendData(sensorID, PGA460_CMD_REGISTER_WRITE, data, 2);
}

// EEPROM bulk read
HAL_StatusTypeDef PGA460_EEPROMBulkRead(uint8_t sensorID, uint8_t *dataBuffer) {
    HAL_StatusTypeDef status = PGA460_SendData(sensorID, PGA460_CMD_EEPROM_BULK_READ, NULL, 0);
    if (status != HAL_OK) return status;

    return PGA460_ReceiveData(sensorID, dataBuffer, 43);
}

// EEPROM bulk write
HAL_StatusTypeDef PGA460_EEPROMBulkWrite(uint8_t sensorID, const uint8_t *dataBuffer) {
    return PGA460_SendData(sensorID, PGA460_CMD_EEPROM_BULK_WRITE, dataBuffer, 43);
}

// Ultrasonic measurement result
HAL_StatusTypeDef PGA460_GetUltrasonicMeasurement(uint8_t sensorID, uint8_t *dataBuffer, uint16_t objectCount) {
    HAL_StatusTypeDef status = PGA460_SendData(sensorID, PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT, NULL, 0);
    if (status != HAL_OK) return status;

    return PGA460_ReceiveData(sensorID, dataBuffer, 4 * objectCount);
}

// Temperature and noise measurement
HAL_StatusTypeDef PGA460_TemperatureAndNoiseMeasurement(uint8_t sensorID, uint8_t *temperature, uint8_t *noise) {
    uint8_t response[2];
    HAL_StatusTypeDef status = PGA460_SendData(sensorID, PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT, NULL, 0);
    if (status != HAL_OK) return status;

    status = PGA460_ReceiveData(sensorID, response, sizeof(response));
    if (status == HAL_OK) {
        *temperature = response[0];
        *noise = response[1];
    }
    return status;
}

// System diagnostics
HAL_StatusTypeDef PGA460_SystemDiagnostics(uint8_t sensorID, uint8_t *diagnostics) {
    HAL_StatusTypeDef status = PGA460_SendData(sensorID, PGA460_CMD_SYSTEM_DIAGNOSTICS, NULL, 0);
    if (status != HAL_OK) return status;

    return PGA460_ReceiveData(sensorID, diagnostics, 2);
}

// Time-varying gain bulk read
HAL_StatusTypeDef PGA460_TimeVaryingGainBulkRead(uint8_t sensorID, uint8_t *gainData) {
    HAL_StatusTypeDef status = PGA460_SendData(sensorID, PGA460_CMD_TVG_BULK_READ, NULL, 0);
    if (status != HAL_OK) return status;

    return PGA460_ReceiveData(sensorID, gainData, 7);
}

// Time-varying gain bulk write
HAL_StatusTypeDef PGA460_TimeVaryingGainBulkWrite(uint8_t sensorID, const uint8_t *gainData) {
    return PGA460_SendData(sensorID, PGA460_CMD_TVG_BULK_WRITE, gainData, 7);
}

// Threshold bulk read
HAL_StatusTypeDef PGA460_ThresholdBulkRead(uint8_t sensorID, uint8_t *thresholdData) {
    HAL_StatusTypeDef status = PGA460_SendData(sensorID, PGA460_CMD_THRESHOLD_BULK_READ, NULL, 0);
    if (status != HAL_OK) return status;

    return PGA460_ReceiveData(sensorID, thresholdData, 32);
}

// Threshold bulk write
HAL_StatusTypeDef PGA460_ThresholdBulkWrite(uint8_t sensorID, const uint8_t *thresholdData) {
    return PGA460_SendData(sensorID, PGA460_CMD_THRESHOLD_BULK_WRITE, thresholdData, 32);
}

// Transducer echo data dump
HAL_StatusTypeDef PGA460_TransducerEchoDataDump(uint8_t sensorID, uint8_t *dataBuffer) {
    HAL_StatusTypeDef status = PGA460_SendData(sensorID, PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP, NULL, 0);
    if (status != HAL_OK) return status;

    return PGA460_ReceiveData(sensorID, dataBuffer, 128); // Assuming 128 bytes for echo data
}

// Digital bandpass filter bandwidth:
// BandWidth = 2 × (BPF_BW + 1) [kHz]
void setBandpassFilterBandwidth(PGA460_Sensor_t *sensor, uint8_t bandWidth) {
    sensor->PGA460_Data.INIT_GAIN.Val.BitField.BPF_BW = (bandWidth - 2) / 2;
}

uint8_t getBandpassFilterBandwidth(PGA460_Sensor_t *sensor) {
    return 2 * (sensor->PGA460_Data.INIT_GAIN.Val.BitField.BPF_BW + 1);
}

// Init_Gain = 0.5 × (GAIN_INIT+1) + value(AFE_GAIN_RNG) [dB]
// Where value(AFE_GAIN_RNG) is the corresponding value in dB for
// bits set for AFE_GAIN_RNG in DECPL_TEMP register
void setInitialGain(PGA460_Sensor_t *sensor, uint8_t gain) {
    int gainValue = 2 * (gain - sensor->PGA460_Data.DECPL_TEMP.Val.BitField.AFE_GAIN_RNG);
    sensor->PGA460_Data.INIT_GAIN.Val.BitField.GAIN_INIT = gainValue > 0 ? gainValue - 1 : 0;
}

uint8_t getInitialGain(PGA460_Sensor_t *sensor) {
    return 0.5 * (sensor->PGA460_Data.INIT_GAIN.Val.BitField.GAIN_INIT + 1) +
           sensor->PGA460_Data.DECPL_TEMP.Val.BitField.AFE_GAIN_RNG;
}

// Burst frequency equation parameter:
// Frequency = 0.2 × FREQ + 30 [kHz]
// The valid FREQ parameter value range is from 0 to 250 (00h to FAh)
void setFrequency(PGA460_Sensor_t *sensor, uint8_t frequency) {
    sensor->PGA460_Data.FREQ = (uint8_t)((frequency - 30) / 0.2);
    if (sensor->PGA460_Data.FREQ > 250) sensor->PGA460_Data.FREQ = 250;
}

uint8_t getFrequency(PGA460_Sensor_t *sensor) {
    return (uint8_t)(0.2 * sensor->PGA460_Data.FREQ + 30);
}

// Threshold level comparator deglitch period:
// deglitch period = (THR_CMP_DEGLITCH × 8) [μs]
void setDeglitchPeriod(PGA460_Sensor_t *sensor, uint8_t deglitchPeriod) {
    sensor->PGA460_Data.DEADTIME.Val.BitField.THR_CMP_DEGLTCH = deglitchPeriod / 8;
}

uint8_t getDeglitchPeriod(PGA460_Sensor_t *sensor) {
    return 8 * sensor->PGA460_Data.DEADTIME.Val.BitField.THR_CMP_DEGLTCH;
}

// Burst Pulse Dead-Time:
// DeadTime = 0.0625 × PULSE_DT[μs]
void setBurstPulseDeadTime(PGA460_Sensor_t *sensor, uint8_t burstPulseDeadTime) {
    sensor->PGA460_Data.DEADTIME.Val.BitField.PULSE_DT = burstPulseDeadTime / 0.0625;
}

uint8_t getBurstPulseDeadTime(PGA460_Sensor_t *sensor) {
    return sensor->PGA460_Data.DEADTIME.Val.BitField.PULSE_DT * 0.0625;
}

// Number of burst pulses for Preset1 PULSE_P1 0 to 15
// Note: 0h means one pulse is generated on OUTA only 
void setBurstPulseP1(PGA460_Sensor_t *sensor, uint8_t burstPulseP1) {
    if (burstPulseP1 > 31) burstPulseP1 = 31;
    sensor->PGA460_Data.PULSE_P1.Val.BitField.P1_PULSE = burstPulseP1;
}

// Number of burst pulses for Preset2 PULSE_P2 0 to 15
// Note: 0h means one pulse is generated on OUTA only 
void setBurstPulseP2(PGA460_Sensor_t *sensor, uint8_t burstPulseP2) {
    if (burstPulseP2 > 31) burstPulseP2 = 31;
    sensor->PGA460_Data.PULSE_P2.Val.BitField.P2_PULSE = burstPulseP2;
}

// UART interface address 0 to 7
void setAddress(PGA460_Sensor_t *sensor, uint8_t address) {
    if (address > 7) address = 7;
    sensor->PGA460_Data.PULSE_P2.Val.BitField.UART_ADDR = address;
}

// Disable Current Limit for Preset1 and Preset2
// 0b = current limit enabled
// 1b = current limit disabled
void setCurrentLimitStatus(PGA460_Sensor_t *sensor, uint8_t currentLimitStatus) {
    sensor->PGA460_Data.CURR_LIM_P1.Val.BitField.DIS_CL = currentLimitStatus ? 1 : 0;
}

// Driver Current Limit for Preset1
// Current_Limit = 7 × CURR_LIM1 + 50 [mA]
void setCurrentLimitValP1(PGA460_Sensor_t *sensor, uint16_t currentLimitValP1) {
    if (currentLimitValP1 > 491) currentLimitValP1 = 491;
    sensor->PGA460_Data.CURR_LIM_P1.Val.BitField.CURR_LIM1 = (currentLimitValP1 - 50) / 7;
}

uint16_t getCurrentLimitValP1(PGA460_Sensor_t *sensor) {
    return 7 * sensor->PGA460_Data.CURR_LIM_P1.Val.BitField.CURR_LIM1 + 50;
}

// Lowpass filter cutoff frequency:
// Cut off frequency = LPF_CO + 1 [kHz]
void setLowpassFilterCutoffFrequency(PGA460_Sensor_t *sensor, uint8_t lowpassFilterCutoffFrequency) {
    if (lowpassFilterCutoffFrequency > 4) lowpassFilterCutoffFrequency = 4;
    sensor->PGA460_Data.CURR_LIM_P2.Val.BitField.LPF_CO = lowpassFilterCutoffFrequency - 1;
}

uint8_t getLowpassFilterCutoffFrequency(PGA460_Sensor_t *sensor) {
    return sensor->PGA460_Data.CURR_LIM_P2.Val.BitField.LPF_CO + 1;
}

// Driver Current Limit for Preset2
// Current_Limit = 7 × CURR_LIM2 + 50 [mA]
void setCurrentLimitValP2(PGA460_Sensor_t *sensor, uint16_t currentLimitValP2) {
    if (currentLimitValP2 > 491) currentLimitValP2 = 491;
    sensor->PGA460_Data.CURR_LIM_P2.Val.BitField.CURR_LIM2 = (currentLimitValP2 - 50) / 7;
}

uint16_t getCurrentLimitValP2(PGA460_Sensor_t *sensor) {
    return 7 * sensor->PGA460_Data.CURR_LIM_P2.Val.BitField.CURR_LIM2 + 50;
}

// Preset1 record time length:
// Record time = 4.096 × (P1_REC + 1) [ms]
void setRecordTimeP1(PGA460_Sensor_t *sensor, uint16_t recordTimeP1) {
    if (recordTimeP1 < 4096) recordTimeP1 = 4096;
    sensor->PGA460_Data.REC_LENGTH.Val.BitField.P1_REC = recordTimeP1 / 4096 - 1;
}

uint16_t getRecordTimeP1(PGA460_Sensor_t *sensor) {
    return 4096 * (sensor->PGA460_Data.REC_LENGTH.Val.BitField.P1_REC + 1);
}

// Preset2 record time length:
// Record time = 4.096 × (P2_REC + 1) [ms]
void setRecordTimeP2(PGA460_Sensor_t *sensor, uint16_t recordTimeP2) {
    if (recordTimeP2 < 4096) recordTimeP2 = 4096;
    sensor->PGA460_Data.REC_LENGTH.Val.BitField.P2_REC = recordTimeP2 / 4096 - 1;
}

uint16_t getRecordTimeP2(PGA460_Sensor_t *sensor) {
    return 4096 * (sensor->PGA460_Data.REC_LENGTH.Val.BitField.P2_REC + 1);
}

// Frequency diagnostic window length:
// For value 0h, the diagnostic is disabled.
// For values 0 to Fh, the window length is given by
// 3 × FDIAG_LEN [Signal Periods]
void setFrequencyDiagnosticWindowLength(PGA460_Sensor_t *sensor, uint8_t frequencyDiagnosticWindowLength) {
    if (frequencyDiagnosticWindowLength > 45) frequencyDiagnosticWindowLength = 45;
    sensor->PGA460_Data.FREQ_DIAG.Val.BitField.FDIAG_LEN = frequencyDiagnosticWindowLength / 3;
}

uint8_t getFrequencyDiagnosticWindowLength(PGA460_Sensor_t *sensor) {
    return 3 * sensor->PGA460_Data.FREQ_DIAG.Val.BitField.FDIAG_LEN;
}

// Frequency diagnostic start time:
// Start time = 100 × FDIAG_START [μs]
// Note: this time is relative to the end-of-burst time
void setFrequencyDiagnosticStartTime(PGA460_Sensor_t *sensor, uint16_t frequencyDiagnosticStartTime) {
    if (frequencyDiagnosticStartTime > 1500) frequencyDiagnosticStartTime = 1500;
    sensor->PGA460_Data.FREQ_DIAG.Val.BitField.FDIAG_START = frequencyDiagnosticStartTime / 100;
}

uint16_t getFrequencyDiagnosticStartTime(PGA460_Sensor_t *sensor) {
    return 100 * sensor->PGA460_Data.FREQ_DIAG.Val.BitField.FDIAG_START;
}

// Saturation diagnostic threshold level:
void setSaturationDiagThreshold(PGA460_Sensor_t *sensor, uint8_t satThreshold) {
    if (satThreshold > 15) satThreshold = 15;
    sensor->PGA460_Data.SAT_FDIAG_TH.Val.BitField.SAT_TH = satThreshold;
}

// Frequency diagnostic absolute error time threshold:
// Error threshold = (FDIAG_ERR_TH + 1) [μs]
void setFrequencyDiagErrorThreshold(PGA460_Sensor_t *sensor, uint8_t freqDiagErrorThreshold) {
    if (freqDiagErrorThreshold > 7) freqDiagErrorThreshold = 7;
    sensor->PGA460_Data.SAT_FDIAG_TH.Val.BitField.FDIAG_ERR_TH = freqDiagErrorThreshold - 1;
}

uint8_t getFrequencyDiagErrorThreshold(PGA460_Sensor_t *sensor) {
    return sensor->PGA460_Data.SAT_FDIAG_TH.Val.BitField.FDIAG_ERR_TH + 1;
}

// Voltage diagnostic threshold:
// Value range from 1 to 8 (for FVOLT_ERR_TH)
void setVoltageDiagThreshold(PGA460_Sensor_t *sensor, uint8_t voltageDiagThreshold) {
    if (voltageDiagThreshold > 7) voltageDiagThreshold = 7;
    sensor->PGA460_Data.FVOLT_DEC.Val.BitField.FVOLT_ERR_TH = voltageDiagThreshold;
}

uint8_t getVoltageDiagThreshold(PGA460_Sensor_t *sensor) {
    return sensor->PGA460_Data.FVOLT_DEC.Val.BitField.FVOLT_ERR_TH + 1;
}

// Sleep mode timer:
// Timer range: 250 ms to 4 s
void setSleepModeTimer(PGA460_Sensor_t *sensor, uint8_t sleepModeTimer) {
    if (sleepModeTimer > 3) sleepModeTimer = 3;
    sensor->PGA460_Data.FVOLT_DEC.Val.BitField.LPM_TMR = sleepModeTimer;
}

// Decouple time or temperature value:
// If DECPL_TEMP_SEL = 0 (Time Decouple) Time = 4096 × (DECPL_T + 1) [μs]
// If DECPL_TEMP_SEL = 1 (Temperature Decouple) Temperature = 10 × DECPL_T - 40 [degC]
void setDecoupleTimeOrTemperature(PGA460_Sensor_t *sensor, uint8_t decoupleValue) {
    if (decoupleValue > 15) decoupleValue = 15;
    sensor->PGA460_Data.DECPL_TEMP.Val.BitField.DECPL_T = decoupleValue;
}

// Non-linear scaling noise level:
// Value ranges from 0 to 31 with 1 LSB steps
void setNoiseLevel(PGA460_Sensor_t *sensor, uint8_t noiseLevel) {
    if (noiseLevel > 31) noiseLevel = 31;
    sensor->PGA460_Data.DSP_SCALE.Val.BitField.NOISE_LVL = noiseLevel;
}

// Temperature-scale offset (signed value range -8 to 7):
void setTemperatureScaleOffset(PGA460_Sensor_t *sensor, int8_t tempOffset) {
    if (tempOffset < -8) tempOffset = -8;
    else if (tempOffset > 7) tempOffset = 7;
    sensor->PGA460_Data.TEMP_TRIM.Val.BitField.TEMP_OFF = tempOffset & 0x0F;
}

int8_t getTemperatureScaleOffset(PGA460_Sensor_t *sensor) {
    int8_t offset = sensor->PGA460_Data.TEMP_TRIM.Val.BitField.TEMP_OFF;
    return (offset & 0x08) ? (offset | 0xF0) : offset;
}

// Temperature-scale gain (signed value range -8 to 7):
void setTemperatureScaleGain(PGA460_Sensor_t *sensor, int8_t tempGain) {
    if (tempGain < -8) tempGain = -8;
    else if (tempGain > 7) tempGain = 7;
    sensor->PGA460_Data.TEMP_TRIM.Val.BitField.TEMP_GAIN = tempGain & 0x0F;
}

int8_t getTemperatureScaleGain(PGA460_Sensor_t *sensor) {
    int8_t gain = sensor->PGA460_Data.TEMP_TRIM.Val.BitField.TEMP_GAIN;
    return (gain & 0x08) ? (gain | 0xF0) : gain;
}
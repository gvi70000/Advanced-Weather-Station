#ifndef PGA460_H
#define PGA460_H

#include "stm32f3xx_hal.h"
#include "PGA460_REG.h"

// Synchronization byte for PGA460 communication
#define PGA460_SYNC 0x55

// Number of ultrasonic sensors in the array
#define ULTRASONIC_SENSOR_COUNT 3

// Enum for PGA460 commands
typedef enum {
    // UART Command Codes
    PGA460_CMD_BURST_AND_LISTEN_PRESET1       = 0x00,
    PGA460_CMD_BURST_AND_LISTEN_PRESET2       = 0x01,
    PGA460_CMD_LISTEN_ONLY_PRESET1            = 0x02,
    PGA460_CMD_LISTEN_ONLY_PRESET2            = 0x03,
    PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT     = 0x04,
    PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT  = 0x05,
    PGA460_CMD_TEMP_AND_NOISE_RESULT          = 0x06,
    PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP      = 0x07,
    PGA460_CMD_SYSTEM_DIAGNOSTICS             = 0x08,
    PGA460_CMD_REGISTER_READ                  = 0x09,
    PGA460_CMD_REGISTER_WRITE                 = 0x0A,
    PGA460_CMD_EEPROM_BULK_READ               = 0x0B,
    PGA460_CMD_EEPROM_BULK_WRITE              = 0x0C,
    PGA460_CMD_TVG_BULK_READ                  = 0x0D,
    PGA460_CMD_TVG_BULK_WRITE                 = 0x0E,
    PGA460_CMD_THRESHOLD_BULK_READ            = 0x0F,
    PGA460_CMD_THRESHOLD_BULK_WRITE           = 0x10,

    // Broadcast Commands
    PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1  = 0x11,
    PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2  = 0x12,
    PGA460_CMD_BROADCAST_LISTEN_ONLY_P1       = 0x13,
    PGA460_CMD_BROADCAST_LISTEN_ONLY_P2       = 0x14,
    PGA460_CMD_BROADCAST_TEMP_NOISE_MEASURE   = 0x15,
    PGA460_CMD_BROADCAST_REGISTER_WRITE       = 0x16,
    PGA460_CMD_BROADCAST_EEPROM_BULK_WRITE    = 0x17,
    PGA460_CMD_BROADCAST_TVG_BULK_WRITE       = 0x18,
    PGA460_CMD_BROADCAST_THRESHOLD_BULK_WRITE = 0x19
} PGA460_Command_t;

// Public Function Prototypes
void PGA460_ProcessReceivedData(uint8_t uartID, uint8_t *data, uint16_t length);
void PGA460_Init();
HAL_StatusTypeDef PGA460_RegisterRead(uint8_t sensorID, uint8_t regAddr, uint8_t *regValue);
HAL_StatusTypeDef PGA460_RegisterWrite(uint8_t sensorID, uint8_t regAddr, uint8_t regValue);
HAL_StatusTypeDef PGA460_EEPROMBulkRead(uint8_t sensorID, uint8_t *dataBuffer);
HAL_StatusTypeDef PGA460_EEPROMBulkWrite(uint8_t sensorID);
HAL_StatusTypeDef PGA460_GetUltrasonicMeasurement(uint8_t sensorID, uint8_t *dataBuffer, uint16_t objectCount);
HAL_StatusTypeDef PGA460_TemperatureAndNoiseMeasurement(uint8_t sensorID, uint8_t *temperature, uint8_t *noise);
HAL_StatusTypeDef PGA460_TemperatureAndNoiseResult(uint8_t sensorID, uint8_t *temperature, uint8_t *noise);
HAL_StatusTypeDef PGA460_TransducerEchoDataDump(uint8_t sensorID, uint8_t *dataBuffer);
HAL_StatusTypeDef PGA460_SystemDiagnostics(uint8_t sensorID, uint8_t *diagnostics);
HAL_StatusTypeDef PGA460_TimeVaryingGainBulkRead(uint8_t sensorID, uint8_t *gainData);
HAL_StatusTypeDef PGA460_TimeVaryingGainBulkWrite(uint8_t sensorID, const uint8_t *gainData);
HAL_StatusTypeDef PGA460_ThresholdBulkRead(uint8_t sensorID, uint8_t *thresholdData);
HAL_StatusTypeDef PGA460_ThresholdBulkWrite(uint8_t sensorID, const uint8_t *thresholdData);

// Bandpass filter bandwidth
void setBandpassFilterBandwidth(PGA460_Sensor_t *sensor, uint8_t bandWidth);
uint8_t getBandpassFilterBandwidth(PGA460_Sensor_t *sensor);

// Initial gain
void setInitialGain(PGA460_Sensor_t *sensor, uint8_t gain);
uint8_t getInitialGain(PGA460_Sensor_t *sensor);

// Burst frequency
void setFrequency(PGA460_Sensor_t *sensor, uint8_t frequency);
uint8_t getFrequency(PGA460_Sensor_t *sensor);

// Deglitch period
void setDeglitchPeriod(PGA460_Sensor_t *sensor, uint8_t deglitchPeriod);
uint8_t getDeglitchPeriod(PGA460_Sensor_t *sensor);

// Burst pulse dead time
void setBurstPulseDeadTime(PGA460_Sensor_t *sensor, uint8_t burstPulseDeadTime);
uint8_t getBurstPulseDeadTime(PGA460_Sensor_t *sensor);

// Burst pulses for Preset1
void setBurstPulseP1(PGA460_Sensor_t *sensor, uint8_t burstPulseP1);

// Burst pulses for Preset2
void setBurstPulseP2(PGA460_Sensor_t *sensor, uint8_t burstPulseP2);

// UART address
void setAddress(PGA460_Sensor_t *sensor, uint8_t address);

// Current limit status
void setCurrentLimitStatus(PGA460_Sensor_t *sensor, uint8_t currentLimitStatus);

// Current limit value for Preset1
void setCurrentLimitValP1(PGA460_Sensor_t *sensor, uint16_t currentLimitValP1);
uint16_t getCurrentLimitValP1(PGA460_Sensor_t *sensor);

// Current limit value for Preset2
void setCurrentLimitValP2(PGA460_Sensor_t *sensor, uint16_t currentLimitValP2);
uint16_t getCurrentLimitValP2(PGA460_Sensor_t *sensor);

// Lowpass filter cutoff frequency
void setLowpassFilterCutoffFrequency(PGA460_Sensor_t *sensor, uint8_t lowpassFilterCutoffFrequency);
uint8_t getLowpassFilterCutoffFrequency(PGA460_Sensor_t *sensor);

// Record time for Preset1
void setRecordTimeP1(PGA460_Sensor_t *sensor, uint16_t recordTimeP1);
uint16_t getRecordTimeP1(PGA460_Sensor_t *sensor);

// Record time for Preset2
void setRecordTimeP2(PGA460_Sensor_t *sensor, uint16_t recordTimeP2);
uint16_t getRecordTimeP2(PGA460_Sensor_t *sensor);

// Frequency diagnostic window length
void setFrequencyDiagnosticWindowLength(PGA460_Sensor_t *sensor, uint8_t frequencyDiagnosticWindowLength);
uint8_t getFrequencyDiagnosticWindowLength(PGA460_Sensor_t *sensor);

// Frequency diagnostic start time
void setFrequencyDiagnosticStartTime(PGA460_Sensor_t *sensor, uint16_t frequencyDiagnosticStartTime);
uint16_t getFrequencyDiagnosticStartTime(PGA460_Sensor_t *sensor);

// Saturation diagnostic threshold
void setSaturationDiagThreshold(PGA460_Sensor_t *sensor, uint8_t satThreshold);

// Voltage diagnostic threshold
void setVoltageDiagThreshold(PGA460_Sensor_t *sensor, uint8_t voltageDiagThreshold);
uint8_t getVoltageDiagThreshold(PGA460_Sensor_t *sensor);

// Sleep mode timer
void setSleepModeTimer(PGA460_Sensor_t *sensor, uint8_t sleepModeTimer);

// Decouple time or temperature
void setDecoupleTimeOrTemperature(PGA460_Sensor_t *sensor, uint8_t decoupleValue);

// Noise level
void setNoiseLevel(PGA460_Sensor_t *sensor, uint8_t noiseLevel);

// Temperature scale offset
void setTemperatureScaleOffset(PGA460_Sensor_t *sensor, int8_t tempOffset);
int8_t getTemperatureScaleOffset(PGA460_Sensor_t *sensor);

// Temperature scale gain
void setTemperatureScaleGain(PGA460_Sensor_t *sensor, int8_t tempGain);
// int8_t getTemperatureScaleGain(PGA460_Sensor_t *sensor);
#endif // PGA460_H
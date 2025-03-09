#ifndef PGA460_H
#define PGA460_H

#include "stm32f3xx_hal.h"
#include "PGA460_REG.h"

// Synchronization byte for PGA460 communication
#define PGA460_SYNC 0x55

// Number of ultrasonic sensors in the array
#define ULTRASONIC_SENSOR_COUNT 3

typedef enum {
    PGA460_MEAS_DISTANCE	= 0,	// Retrieve distance measurement (in meters)
    PGA460_MEAS_WIDTH			= 1,	// Retrieve width of echo signal (in microseconds)
    PGA460_MEAS_AMPLITUDE	= 2		// Retrieve peak amplitude of echo signal (8-bit raw value)
} PGA460_MeasResult_t;

typedef enum {
 PGA460_CMD_GET_TEMP = 0x00,
 PGA460_CMD_GET_NOISE = 0x01
} PGA460_CmdType_t;

#define PGA460_TEMP_ERR 999.0f
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

/* -------------------------------------------------
 * Initialization Functions
 * ------------------------------------------------- */
HAL_StatusTypeDef PGA460_Init(void);
HAL_StatusTypeDef PGA460_CheckStatus(const uint8_t sensorID);

/* -------------------------------------------------
 * Register Read/Write Functions
 * ------------------------------------------------- */
HAL_StatusTypeDef PGA460_RegisterRead(const uint8_t sensorID, const uint8_t regAddr, uint8_t *regValue);
HAL_StatusTypeDef PGA460_RegisterWrite(const uint8_t sensorID, const uint8_t regAddr, const uint8_t regValue);

/* -------------------------------------------------
 * EEPROM Functions
 * ------------------------------------------------- */
HAL_StatusTypeDef PGA460_EEPROMBulkRead(const uint8_t sensorID, PGA460_Sensor_t *dataBuffer);
HAL_StatusTypeDef PGA460_EEPROMBulkWrite(uint8_t sensorID);
HAL_StatusTypeDef PGA460_BurnEEPROM(uint8_t sensorID);

/* -------------------------------------------------
 * Configuration Functions
 * ------------------------------------------------- */
HAL_StatusTypeDef PGA460_InitTimeVaryingGain(const uint8_t sensorID, const PGA460_GainRange_t gain_range, const PGA460_TVG_Level_t timeVaryingGain);

/* -------------------------------------------------
 * Measurement Functions
 * ------------------------------------------------- */
HAL_StatusTypeDef PGA460_UltrasonicCmd(const uint8_t sensorID, const PGA460_Command_t cmd, const uint8_t numObjUpdate);
HAL_StatusTypeDef PGA460_PullUltrasonicMeasResult(const uint8_t sensorID, uint8_t *resultBuffer);
float PGA460_ProcessUltrasonicMeasResult(const uint8_t sensorID, const uint8_t objIndex, const PGA460_MeasResult_t type);
HAL_StatusTypeDef PGA460_GetUltrasonicMeasurement(const uint8_t sensorID, uint8_t *dataBuffer, const uint16_t objectCount);
float PGA460_ReadTemperatureOrNoise(const uint8_t sensorID, const PGA460_CmdType_t mode);
HAL_StatusTypeDef PGA460_SetTemperatureOffset(uint8_t sensorID, float externalTempC);
HAL_StatusTypeDef PGA460_GetSystemDiagnostics(const uint8_t sensorID, const uint8_t run, const uint8_t diag, float *diagResult);
HAL_StatusTypeDef PGA460_GetTimeVaryingGain(const uint8_t sensorID, uint8_t *gainData);
HAL_StatusTypeDef PGA460_GetThresholds(const uint8_t sensorID, uint8_t *thresholdData);
HAL_StatusTypeDef PGA460_GetEchoDataDump(const uint8_t sensorID, uint8_t *dataBuffer);

/* -------------------------------------------------
 * Bandpass & Gain Configuration Functions
 * ------------------------------------------------- */
void setBandpassFilterBandwidth(const uint8_t sensorID, uint8_t bandWidth);
uint8_t getBandpassFilterBandwidth(const uint8_t sensorID);
void setInitialGain(const uint8_t sensorID, uint8_t gain);
uint8_t getInitialGain(const uint8_t sensorID);

/* -------------------------------------------------
 * Frequency & Timing Configuration
 * ------------------------------------------------- */
void setFrequency(const uint8_t sensorID, uint8_t frequency);
uint8_t getFrequency(const uint8_t sensorID);
void setDeglitchPeriod(const uint8_t sensorID, uint8_t deglitchPeriod);
uint8_t getDeglitchPeriod(const uint8_t sensorID);
void setBurstPulseDeadTime(const uint8_t sensorID, uint8_t burstPulseDeadTime);
uint8_t getBurstPulseDeadTime(const uint8_t sensorID);
void setBurstPulseP1(const uint8_t sensorID, uint8_t burstPulseP1);
void setBurstPulseP2(const uint8_t sensorID, uint8_t burstPulseP2);

/* -------------------------------------------------
 * Device Configuration Functions
 * ------------------------------------------------- */
void setAddress(const uint8_t sensorID, uint8_t address);
void setCurrentLimitStatus(const uint8_t sensorID, uint8_t currentLimitStatus);
void setSleepModeTimer(const uint8_t sensorID, uint8_t sleepModeTimer);
void setDecoupleTimeOrTemperature(const uint8_t sensorID, uint8_t decoupleValue);
void setNoiseLevel(const uint8_t sensorID, uint8_t noiseLevel);

/* -------------------------------------------------
 * Temperature & Scale Configuration
 * ------------------------------------------------- */
void setTemperatureScaleOffset(const uint8_t sensorID, int8_t tempOffset);
int8_t getTemperatureScaleOffset(const uint8_t sensorID);
void setTemperatureScaleGain(const uint8_t sensorID, int8_t tempGain);
int8_t getTemperatureScaleGain(const uint8_t sensorID);

#endif // PGA460_H
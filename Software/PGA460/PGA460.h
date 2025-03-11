#ifndef PGA460_H
#define PGA460_H

#include "stm32f3xx_hal.h"
#include "PGA460_REG.h"

// Public Function Prototypes

void PGA460_ReadAllSensors(void);
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
HAL_StatusTypeDef PGA460_GetUltrasonicMeasurement(const uint8_t sensorID);
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
#ifndef PGA460_H
#define PGA460_H

#include "stm32f3xx_hal.h"
#include "PGA460_REG.h"

typedef enum {
    // --- PGA460 UART Diagnostic Flags ---
		PGA460_ERR_NONE = 0,
		PGA460_ERR_GEN ,	
		PGA460_ERR_UART_TX ,
		PGA460_ERR_UART_RX ,
		PGA460_ERR_UART_UN ,
    PGA460_ERR_DEVICE_BUSY ,	// Bit 0
    PGA460_ERR_SYNC_RATE ,		// Bit 1
    PGA460_ERR_SYNC_WIDTH ,		// Bit 2
    PGA460_ERR_CHECKSUM ,			// Bit 3
    PGA460_ERR_INVALID_CMD ,	// Bit 4
    PGA460_ERR_UART_FRAME ,		// Bit 5
    // --- Application-Level Error Codes ---
    PGA460_ERR_EEPROM_BURN ,
    PGA460_ERR_CRC_FAIL ,
    PGA460_ERR_SENSOR_INIT_FAIL ,
		PGA460_ERR_GET_THR_FAIL ,
		PGA460_ERR_TIMEOUT
} PGA460_Error_t;

// Public Function Prototypes
extern PGA460_Sensor_t myUltraSonicArray[ULTRASONIC_SENSOR_COUNT];
void calculateSpeedOfSound(void);
void calculateWind(uint32_t ToF_up[3], uint32_t ToF_down[3], float speed_of_sound, float *wind_speed, float *wind_direction);


/* -------------------------------------------------
 * Initialization Functions
 * ------------------------------------------------- */
PGA460_Error_t PGA460_Init(void);
PGA460_Error_t PGA460_CheckStatus(const uint8_t sensorID);

/* -------------------------------------------------
 * Register Read/Write Functions
 * ------------------------------------------------- */
PGA460_Error_t PGA460_RegisterRead(const uint8_t sensorID, const uint8_t regAddr, uint8_t *regValue);
PGA460_Error_t PGA460_RegisterWrite(const uint8_t sensorID, const uint8_t regAddr, const uint8_t regValue);

/* -------------------------------------------------
 * EEPROM Functions
 * ------------------------------------------------- */
PGA460_Error_t PGA460_EEPROMBulkRead(const uint8_t sensorID);
PGA460_Error_t PGA460_EEPROMBulkWrite(uint8_t sensorID);
PGA460_Error_t PGA460_BurnEEPROM(uint8_t sensorID);

/* -------------------------------------------------
 * Configuration Functions
 * ------------------------------------------------- */
PGA460_Error_t PGA460_GetTVG(uint8_t sensorID);
PGA460_Error_t PGA460_SetTVG(const uint8_t sensorID, const PGA460_GainRange_t gain_range, const PGA460_TVG_Level_t timeVaryingGain);
PGA460_Error_t PGA460_GetThresholds(const uint8_t sensorID);
PGA460_Error_t PGA460_SetThresholds(const uint8_t sensorID, PGA460_TRH_Level_t thresholdLevel);
PGA460_Error_t PGA460_AutoThreshold(uint8_t sensorID, uint8_t noiseMargin, uint8_t windowIndex, uint8_t autoMax, uint8_t loops);
PGA460_Error_t PGA460_AutoThreshold_Internal(uint8_t sensorID, PGA460_Command_t cmd, uint8_t noiseMargin, uint8_t windowIndex, uint8_t autoMax, uint8_t loops, PGA460_THR_t *thr);
/* -------------------------------------------------
 * Measurement Functions
 * ------------------------------------------------- */
PGA460_Error_t PGA460_UltrasonicCmd(const uint8_t sensorID, const PGA460_Command_t cmd);
PGA460_Error_t PGA460_GetUltrasonicMeasurement(const uint8_t sensorID);
PGA460_Error_t PGA460_GetEchoDataDump(uint8_t sensorID, uint8_t preset, uint8_t *echoOut);
float PGA460_ReadTemperatureOrNoise(const uint8_t sensorID, const PGA460_CmdType_t mode);
PGA460_Error_t PGA460_SetTemperatureOffset(uint8_t sensorID, float externalTempC);
PGA460_Error_t PGA460_GetSystemDiagnostics(const uint8_t sensorID, const uint8_t run, const uint8_t diag, float *diagResult);

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
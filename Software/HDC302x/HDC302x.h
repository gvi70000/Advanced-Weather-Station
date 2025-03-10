
/**
 * @file HDC302x.h
 * @brief Header file for the HDC302x temperature and humidity sensor library.
 *
 * This library contains all the definitions, macros, and function prototypes
 * for initializing, configuring, and reading data from the HDC302x sensor.
 */

#ifndef HDC302X_H
#define HDC302X_H

#include "stm32f3xx_hal.h"
#include "i2c.h"
#include <stdint.h>

#define HDC_READ_TIME 1000 //read HDC sensors every second

/** @defgroup HDC302x_I2C_Addresses HDC302x I2C Addresses
 * @{
 */
#define HDC302X_SENSOR_1_ADDR (0x44 << 1)  ///< I2C address for Sensor 1 (0x44)
#define HDC302X_SENSOR_2_ADDR (0x45 << 1)  ///< I2C address for Sensor 2 (0x45)

/** @defgroup HDC302x_Constants Conversion Constants
 * @{
 */
#define HDC302X_RH_COEFF							(100.0f/65535.0f)		///< Humidity conversion coefficient
#define HDC302X_RH_COEFF_INV					(65535.0f / 100.0f)	///< Inverse of humidity conversion coefficient
#define HDC302X_TEMP_COEFF1						(175.0f/65535.0f)		///< Temperature conversion coefficient 1
#define HDC302X_TEMP_COEFF2						45.0f								///< Temperature conversion coefficient 2
#define HDC302X_TEMP_COEFF1_INV				(65535.0f / 175.0f)	///< Inverse of temperature conversion coefficient
#define HDC302X_TEMP_COEFF3						(45.0f * HDC302X_TEMP_COEFF1_INV)

#define RH_LSB												0.1953125f		// LSB for RH offset
#define T_LSB													0.1708984375f	// LSB for Temperature offset
#define HDC302X_CRC_INIT							0xFF  ///< Initial CRC value for CRC-8/NRSC-5 checksum
#define HDC302X_CRC_POLY							0x31  ///< Polynomial used for CRC-8/NRSC-5 checksum
#define HDC302X_SIGN_MASK							0x80
#define HDC302X_HUMIDITY_MSB_MASK			0x7F  ///< Mask to extract the 7 MSBs for humidity
#define HDC302X_TEMPERATURE_MSB_MASK	0x1FF ///< Mask to extract the 9 MSBs for temperature

#define DEW_POINT_CONST_A							17.27f ///< Magnus-Tetens constant A
#define DEW_POINT_CONST_B							237.7f ///< Magnus-Tetens constant B

/**
 * @brief Sensor commands. They have the bytes swaped for STM32 little-endian
 */
/** Trigger-On Demand Mode Commands */
#define HDC302X_CMD_MEASURE_DEMAND_LPM0	0x0024 ///< On-demand measurement (low noise)
#define HDC302X_CMD_MEASURE_DEMAND_LPM1	0x0B24 ///< On-demand measurement (low power 1)
#define HDC302X_CMD_MEASURE_DEMAND_LPM2	0x1624 ///< On-demand measurement (low power 2)
#define HDC302X_CMD_MEASURE_DEMAND_LPM3	0xFF24 ///< On-demand measurement (low power 3)

/** Auto Measurement Mode 0.5Hz */
#define HDC302X_CMD_MEASURE_2S_LPM0		0x3220 ///< Auto measurement: 1 every 2 seconds (LPM0)
#define HDC302X_CMD_MEASURE_2S_LPM1		0x2420 ///< Auto measurement: 1 every 2 seconds (LPM1)
#define HDC302X_CMD_MEASURE_2S_LPM2		0x2F20 ///< Auto measurement: 1 every 2 seconds (LPM2)
#define HDC302X_CMD_MEASURE_2S_LPM3		0xFF20 ///< Auto measurement: 1 every 2 seconds (LPM3)

/** Auto Measurement Mode 1Hz */
#define HDC302X_CMD_MEASURE_01_LPM0		0x3021 ///< Auto measurement: 1 per second (LPM0)
#define HDC302X_CMD_MEASURE_01_LPM1		0x2621 ///< Auto measurement: 1 per second (LPM1)
#define HDC302X_CMD_MEASURE_01_LPM2		0x2D21 ///< Auto measurement: 1 per second (LPM2)
#define HDC302X_CMD_MEASURE_01_LPM3		0xFF21 ///< Auto measurement: 1 per second (LPM3)

/** Auto Measurement Mode 2Hz */
#define HDC302X_CMD_MEASURE_02_LPM0		0x3622 ///< Auto measurement: 2 per second (LPM0)
#define HDC302X_CMD_MEASURE_02_LPM1		0x2022 ///< Auto measurement: 2 per second (LPM1)
#define HDC302X_CMD_MEASURE_02_LPM2		0x2B22 ///< Auto measurement: 2 per second (LPM2)
#define HDC302X_CMD_MEASURE_02_LPM3		0xFF22 ///< Auto measurement: 2 per second (LPM3)

/** Auto Measurement Mode 4Hz */
#define HDC302X_CMD_MEASURE_04_LPM0		0x3423 ///< Auto measurement: 4 per second (LPM0)
#define HDC302X_CMD_MEASURE_04_LPM1		0x2223 ///< Auto measurement: 4 per second (LPM1)
#define HDC302X_CMD_MEASURE_04_LPM2		0x2923 ///< Auto measurement: 4 per second (LPM2)
#define HDC302X_CMD_MEASURE_04_LPM3		0xFF23 ///< Auto measurement: 4 per second (LPM3)

/** Auto Measurement Mode 10Hz */
#define HDC302X_CMD_MEASURE_10_LPM0		0x3727 ///< Auto measurement: 10 per second (LPM0)
#define HDC302X_CMD_MEASURE_10_LPM1		0x2127 ///< Auto measurement: 10 per second (LPM1)
#define HDC302X_CMD_MEASURE_10_LPM2		0x2A27 ///< Auto measurement: 10 per second (LPM2)
#define HDC302X_CMD_MEASURE_10_LPM3		0xFF27 ///< Auto measurement: 10 per second (LPM3)

#define HDC302X_CMD_RETURN_TO_TRIGGER	0x9330 ///< Exit, then return to Trigger-on Demand Mode

/** Measurement Commands */
#define HDC302X_CMD_MEASURE_READ                       0x00E0 ///< Measurement Readout of T and RH
#define HDC302X_CMD_READ_RH_ONLY                       0x01E0 ///< Measurement Readout of RH only
/** Measurement History and Statistics Commands */
#define HDC302X_CMD_READ_HISTORY_MIN_T		0x02E0 ///< Measurement History Readout of Minimum T
#define HDC302X_CMD_READ_HISTORY_MAX_T		0x03E0 ///< Measurement History Readout of Maximum T
#define HDC302X_CMD_READ_HISTORY_MIN_RH		0x04E0 ///< Measurement History Readout of Minimum RH
#define HDC302X_CMD_READ_HISTORY_MAX_RH		0x05E0 ///< Measurement History Readout of Maximum RH

/** Configure ALERT Thresholds of T and RH */
#define HDC302X_CMD_SET_ALERT_LOW			0x0061 ///< Configures Thresholds for "Set Low Alert"
#define HDC302X_CMD_SET_ALERT_HIGH		0x1D61 ///< Configures Thresholds for "Set High Alert"
#define HDC302X_CMD_CLR_ALERT_LOW			0x0B61 ///< Configures Thresholds for "Clear Low Alert"
#define HDC302X_CMD_CLR_ALERT_HIGH		0x1661 ///< Configures Thresholds for "Clear High Alert"

/** Read ALERT Thresholds of T and RH */
#define HDC302X_CMD_GET_ALERT_SET_LOW			0x02E1 ///< Read Thresholds for "Set Low Alert"
#define HDC302X_CMD_GET_ALERT_SET_HIGH		0x1FE1 ///< Read Thresholds for "Set High Alert"
#define HDC302X_CMD_GET_ALERT_CLR_LOW			0x09E1 ///< Read Thresholds for "Clear Low Alert"
#define HDC302X_CMD_GET_ALERT_CLR_HIGH		0x14E1 ///< Read Thresholds for "Clear High Alert"

/** Integrated Heater Commands */
#define HDC302X_CMD_HEATER_ENABLE                      0x6D30 ///< Enable heater
#define HDC302X_CMD_HEATER_DISABLE                     0x6630 ///< Disable heater
#define HDC302X_CMD_HEATER_CONFIG                      0x6E30 ///< Configure & Read Back Heater Settings

/** Status register commands */
#define HDC302X_READ_STATUS                            0x2DF3 ///< Status Register Read Content
#define HDC302X_CLEAR_STATUS                           0x4130 ///< Status Register Clear Content

/** Reset Commands */
#define HDC302X_CMD_SOFT_RESET                         0xA230 ///< Soft reset command

/** NIST and Manufacturer Identification Commands */
#define HDC302X_CMD_READ_NIST_ID_BYTE_5_4              0x8336 ///< Read NIST ID bytes 5 and 4
#define HDC302X_CMD_READ_NIST_ID_BYTE_3_2              0x8436 ///< Read NIST ID bytes 3 and 2
#define HDC302X_CMD_READ_NIST_ID_BYTE_1_0              0x8536 ///< Read NIST ID bytes 1 and 0
#define HDC302X_CMD_READ_MANUFACTURER_ID               0x8137 ///< Read manufacturer ID

/** NVM (EEPROM) Programming Commands */
#define HDC302X_CMD_PROGRAM_ALERT_THRESHOLDS_TO_NVM    0x5561 ///< Program ALERT Thresholds of T and RH to Non-Volatile Memory (NVM)
#define HDC302X_CMD_PROGRAM_READ_OFFSET_VALUES         0x04A0 ///< Program/Read Offset Values of Relative Humidity and Temperature Results
#define HDC302X_CMD_PROGRAM_READ_DEFAULT_STATE         0xBB61 ///< Program/Read Default Device Power-On/Reset Measurement State

/**
 * @brief Structure to represent HDC302x default power-on/reset configurations.
 */
typedef struct __attribute__((packed)) {
    uint16_t CFG_VAL; ///< 16-bit configuration value MSB + LSB flipped for STM32
    uint8_t  CFG_CRC; ///< 8-bit configuration value CRC
} HDC302x_Config_t;

/** Lowest Noise Configurations */
#define HDC302X_CONFIG_0_5HZ_LOWEST_NOISE  ((HDC302x_Config_t){ .CFG_VAL = 0x0300, .CFG_CRC = 0xD2 }) ///< 0.5 Hz, Lowest Noise
#define HDC302X_CONFIG_1HZ_LOWEST_NOISE    ((HDC302x_Config_t){ .CFG_VAL = 0x0500, .CFG_CRC = 0x74 }) ///< 1 Hz, Lowest Noise
#define HDC302X_CONFIG_2HZ_LOWEST_NOISE    ((HDC302x_Config_t){ .CFG_VAL = 0x0700, .CFG_CRC = 0x16 }) ///< 2 Hz, Lowest Noise
#define HDC302X_CONFIG_4HZ_LOWEST_NOISE    ((HDC302x_Config_t){ .CFG_VAL = 0x0900, .CFG_CRC = 0x09 }) ///< 4 Hz, Lowest Noise
#define HDC302X_CONFIG_10HZ_LOWEST_NOISE   ((HDC302x_Config_t){ .CFG_VAL = 0x0B00, .CFG_CRC = 0x6B }) ///< 10 Hz, Lowest Noise

/** Low Power Mode 1 Configurations */
#define HDC302X_CONFIG_0_5HZ_LPM1          ((HDC302x_Config_t){ .CFG_VAL = 0x1300, .CFG_CRC = 0x91 }) ///< 0.5 Hz, Low Power Mode 1
#define HDC302X_CONFIG_1HZ_LPM1            ((HDC302x_Config_t){ .CFG_VAL = 0x1500, .CFG_CRC = 0x37 }) ///< 1 Hz, Low Power Mode 1
#define HDC302X_CONFIG_2HZ_LPM1            ((HDC302x_Config_t){ .CFG_VAL = 0x1700, .CFG_CRC = 0x55 }) ///< 2 Hz, Low Power Mode 1
#define HDC302X_CONFIG_4HZ_LPM1            ((HDC302x_Config_t){ .CFG_VAL = 0x1900, .CFG_CRC = 0x4A }) ///< 4 Hz, Low Power Mode 1
#define HDC302X_CONFIG_10HZ_LPM1           ((HDC302x_Config_t){ .CFG_VAL = 0x1B00, .CFG_CRC = 0x28 }) ///< 10 Hz, Low Power Mode 1

/** Low Power Mode 2 Configurations */
#define HDC302X_CONFIG_0_5HZ_LPM2          ((HDC302x_Config_t){ .CFG_VAL = 0x2300, .CFG_CRC = 0x54 }) ///< 0.5 Hz, Low Power Mode 2
#define HDC302X_CONFIG_1HZ_LPM2            ((HDC302x_Config_t){ .CFG_VAL = 0x2500, .CFG_CRC = 0xF2 }) ///< 1 Hz, Low Power Mode 2
#define HDC302X_CONFIG_2HZ_LPM2            ((HDC302x_Config_t){ .CFG_VAL = 0x2700, .CFG_CRC = 0x90 }) ///< 2 Hz, Low Power Mode 2
#define HDC302X_CONFIG_4HZ_LPM2            ((HDC302x_Config_t){ .CFG_VAL = 0x2900, .CFG_CRC = 0x8F }) ///< 4 Hz, Low Power Mode 2
#define HDC302X_CONFIG_10HZ_LPM2           ((HDC302x_Config_t){ .CFG_VAL = 0x2B00, .CFG_CRC = 0xED }) ///< 10 Hz, Low Power Mode 2

/** Low Power Mode 3 Configurations */
#define HDC302X_CONFIG_0_5HZ_LPM3          ((HDC302x_Config_t){ .CFG_VAL = 0x3300, .CFG_CRC = 0x17 }) ///< 0.5 Hz, Low Power Mode 3
#define HDC302X_CONFIG_1HZ_LPM3            ((HDC302x_Config_t){ .CFG_VAL = 0x3500, .CFG_CRC = 0xB1 }) ///< 1 Hz, Low Power Mode 3
#define HDC302X_CONFIG_2HZ_LPM3            ((HDC302x_Config_t){ .CFG_VAL = 0x3700, .CFG_CRC = 0xD3 }) ///< 2 Hz, Low Power Mode 3
#define HDC302X_CONFIG_4HZ_LPM3            ((HDC302x_Config_t){ .CFG_VAL = 0x3900, .CFG_CRC = 0xCC }) ///< 4 Hz, Low Power Mode 3
#define HDC302X_CONFIG_10HZ_LPM3           ((HDC302x_Config_t){ .CFG_VAL = 0x3B00, .CFG_CRC = 0xAE }) ///< 10 Hz, Low Power Mode 3

/** Factory Default Configuration */
#define HDC302X_CONFIG_FACTORY_DEFAULT     ((HDC302x_Config_t){ .CFG_VAL = 0x0000, .CFG_CRC = 0x81 }) ///< Restore Factory Default (Sleep Mode)

/**
 * @brief Structure to represent HDC302x heater configurations.
 */
typedef struct __attribute__((packed)) {
    uint16_t HEATER_VAL; ///< 16-bit heater power MSB + LSB flipped for STM32
    uint8_t  HCRC;       ///< 8-bit CRC for the heater configuration
} HDC302x_HeaterConfig_t;

/** Predefined configurations for HDC302x heater settings */
#define HDC302X_HEATER_OFF	((HDC302x_HeaterConfig_t){ .HEATER_VAL = 0x0000, .HCRC = 0x31 }) ///< Heater off (0%)
#define HDC302X_HEATER_25		((HDC302x_HeaterConfig_t){ .HEATER_VAL = 0x9F00, .HCRC = 0x96 }) ///< Heater at 25% (quarter power)
#define HDC302X_HEATER_50		((HDC302x_HeaterConfig_t){ .HEATER_VAL = 0xFF03, .HCRC = 0x00 }) ///< Heater at 50% (half power)
#define HDC302X_HEATER_75		((HDC302x_HeaterConfig_t){ .HEATER_VAL = 0xFF1F, .HCRC = 0x12 }) ///< Heater at 75% (three-quarter power)
#define HDC302X_HEATER_100	((HDC302x_HeaterConfig_t){ .HEATER_VAL = 0xFF3F, .HCRC = 0x06 }) ///< Heater at 100% (full power)

/** 
 * @brief Sensor operating modes.
 */
typedef enum {
		HDC302X_MODE_NORMAL,		///< Normal operation mode
    HDC302X_MODE_LOW_POWER	///< Low power operation mode
} HDC302x_Mode_t;

/**
 * @brief Structure to hold temperature and humidity data.
 */
typedef struct __attribute__((packed)) {
		float Temperature;		///< Temperature in degrees Celsius
		float Humidity;				///< Relative Humidity in percentage
} HDC302x_Data_t;

/**
 * @brief Structure to hold history data (min/max temperature and humidity).
 */
typedef struct __attribute__((packed)) {
		HDC302x_Data_t MAX; ///< Maximum recorded temperature in degrees Celsius and RH in %
		HDC302x_Data_t MIN; ///< MIN recorded temperature in degrees Celsius and RH in %
} HDC302x_History_t;

/**
 * @brief Status register structure.
 */
typedef struct __attribute__((packed)) {
    union {
        uint16_t Value; /**< Complete register value. */
        struct {
            uint16_t overall_alert  : 1;   ///< Bit 15: Overall Alert Status (0 = No active alerts, 1 = At least one alert)
            uint16_t reserved4      : 1;   ///< Bit 14: Reserved
            uint16_t heater_status  : 1;   ///< Bit 13: Heater Status (0 = Disabled, 1 = Enabled)
            uint16_t reserved3      : 1;   ///< Bit 12: Reserved
            uint16_t rh_alert       : 1;   ///< Bit 11: RH Tracking Alert (0 = No RH alert, 1 = RH alert)
            uint16_t t_alert        : 1;   ///< Bit 10: T Tracking Alert (0 = No T alert, 1 = T alert)
            uint16_t rh_high_alert  : 1;   ///< Bit 9: RH High Tracking Alert (0 = No RH High alert, 1 = RH High alert)
            uint16_t rh_low_alert   : 1;   ///< Bit 8: RH Low Tracking Alert (0 = No RH Low alert, 1 = RH Low alert)
            uint16_t t_high_alert   : 1;   ///< Bit 7: T High Tracking Alert (0 = No T High alert, 1 = T High alert)
            uint16_t t_low_alert    : 1;   ///< Bit 6: T Low Tracking Alert (0 = No T Low alert, 1 = T Low alert)
            uint16_t reserved2      : 1;   ///< Bit 5: Reserved
            uint16_t device_reset   : 1;   ///< Bit 4: Device Reset Detected (0 = No reset, 1 = Reset detected)
            uint16_t reserved1      : 3;   ///< Bits 1-3: Reserved
            uint16_t checksum_error : 1;   ///< Bit 0: Checksum verification of last data write (0 = Pass, 1 = Fail)
        } BitField;
    } Val;
} HDC302x_Status_t;

/**
 * @brief Main sensor structure representing an HDC302x device.
 */
typedef struct {
    uint8_t Address;								///< I2C address of the sensor
    HDC302x_Config_t Config;				///< Current configuration settings
    HDC302x_HeaterConfig_t Heater;	///< Heater configuration settings
    HDC302x_Status_t Status;				///< Status register data
    HDC302x_Data_t Data;						///< Latest temperature and humidity readings
} HDC302x_t;

HAL_StatusTypeDef HDC302x_Init(uint8_t senID);
HAL_StatusTypeDef HDC3020_SoftReset(uint8_t senID);
HAL_StatusTypeDef HDC302x_SetConfiguration(uint8_t senID, const HDC302x_Config_t *config);
HAL_StatusTypeDef HDC3020_ReadStatusRegister(uint8_t senID);
HAL_StatusTypeDef HDC3020_ClearStatusRegister(uint8_t senID);
HAL_StatusTypeDef HDC302x_ReadTemperatureAndHumidity(uint8_t senID, HDC302x_Data_t *data);
HAL_StatusTypeDef HDC3020_SelectMeasurementMode(uint8_t senID, uint16_t command);
HAL_StatusTypeDef HDC3020_GetMeasurementHistory(uint8_t senID, HDC302x_History_t *history);
HAL_StatusTypeDef HDC302x_TransferAlertLimitsToNVM(uint8_t senID);
HAL_StatusTypeDef HDC302x_SetAlertLimits(uint8_t senID, HDC302x_Data_t highAlertValue, HDC302x_Data_t lowAlertValue, HDC302x_Data_t highAlertClear, HDC302x_Data_t lowAlertClear);
HAL_StatusTypeDef HDC302x_GetAlertLimits(uint8_t senID, HDC302x_Data_t *highAlertValue, HDC302x_Data_t *lowAlertValue, HDC302x_Data_t *highAlertClear, HDC302x_Data_t *lowAlertClear);
HAL_StatusTypeDef HDC302x_TransferOffsetsToNVM(uint8_t senID);
HAL_StatusTypeDef HDC3020_SetOffset(uint8_t senID, float RH_Offset, float T_Offset);
HAL_StatusTypeDef HDC3020_GetOffset(uint8_t senID, float *rhOffset, float *tOffset);
uint8_t HDC3020_IsHeaterOn(uint8_t senID);
HAL_StatusTypeDef HDC3020_ControlHeater(uint8_t senID, const HDC302x_HeaterConfig_t *config);
HAL_StatusTypeDef HDC302x_ProgramAlertThresholdsToNVM(uint8_t senID);
HAL_StatusTypeDef HDC302x_ProgramReadDefaultState(uint8_t senID, uint8_t programDefault, uint16_t *state);
HAL_StatusTypeDef HDC302x_ReadManufacturerID(uint8_t senID, uint16_t *manufacturerID);
float CalculateDewPoint(float temperature, float humidity);
#endif // HDC302X_H

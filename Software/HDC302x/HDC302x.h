#ifndef HDC302X_H
#define HDC302X_H

#include "stm32f3xx_hal.h"
#include "i2c.h"
#include <stdint.h>

// I2C addresses for HDC302x sensors
#define HDC302X_SENSOR_1_ADDR (0x44 << 1)  // Sensor 1 (0x44)
#define HDC302X_SENSOR_2_ADDR (0x47 << 1)  // Sensor 2 (0x46)

// HDC302x Registers
#define HDC302X_REG_TEMP        0x00  // Temperature Measurement Register
#define HDC302X_REG_HUMIDITY    0x01  // Humidity Measurement Register
#define HDC302X_REG_CONFIG      0x0F  // Configuration Register
#define HDC302X_REG_RESET       0x1E  // Reset Command
#define HDC302X_REG_MANUF_ID    0xFE  // Manufacturer ID Register
#define HDC302X_REG_DEVICE_ID   0xFF  // Device ID Register

#define HDC302X_RH_COEFF				0.00152587890625f
#define HDC302X_TEMP_COEFF1			0.0025177001953125f
#define HDC302X_TEMP_COEFF2			40.0f

// Dew point constants for calculation (Magnus-Tetens approximation)
#define DEW_POINT_CONST_A       17.27f
#define DEW_POINT_CONST_B       237.7f

// Heater power values
typedef enum {
    HDC302X_HEATER_OFF		= 0x00,
    HDC302X_HEATER_LOW		= 0x01,
    HDC302X_HEATER_MEDIUM	= 0x02,
    HDC302X_HEATER_HIGH		= 0x03
} HDC302x_HeaterPower_t;

// Sensor modes
typedef enum {
    HDC302X_MODE_NORMAL,
    HDC302X_MODE_LOW_POWER
} HDC302x_Mode_t;

// HDC302x commands based on the Excel data
typedef enum {
    HDC302X_CMD_TRIGGER_ON_DEMAND_LOW_NOISE				= 0x2400,
    HDC302X_CMD_TRIGGER_ON_DEMAND_LOW_POWER_1			= 0x240B,
    HDC302X_CMD_TRIGGER_ON_DEMAND_LOW_POWER_2			= 0x2416,
    HDC302X_CMD_TRIGGER_ON_DEMAND_LOW_POWER_3			= 0x24FF,
    HDC302X_CMD_AUTO_MEASUREMENT_2S_LOW_NOISE			= 0x2032,
    HDC302X_CMD_AUTO_MEASUREMENT_2S_LOW_POWER_1		= 0x2024,
    HDC302X_CMD_AUTO_MEASUREMENT_2S_LOW_POWER_2		= 0x202F,
    HDC302X_CMD_AUTO_MEASUREMENT_2S_LOW_POWER_3		= 0x20FF,
    HDC302X_CMD_AUTO_MEASUREMENT_1S_LOW_NOISE			= 0x2130,
    HDC302X_CMD_AUTO_MEASUREMENT_1S_LOW_POWER_1		= 0x2126,
    HDC302X_CMD_AUTO_MEASUREMENT_1S_LOW_POWER_2		= 0x212D,
    HDC302X_CMD_AUTO_MEASUREMENT_1S_LOW_POWER_3		= 0x21FF,
    HDC302X_CMD_AUTO_MEASUREMENT_2PS_LOW_NOISE		= 0x2236,
    HDC302X_CMD_AUTO_MEASUREMENT_2PS_LOW_POWER_1	= 0x2220,
    HDC302X_CMD_AUTO_MEASUREMENT_2PS_LOW_POWER_2	= 0x222B,
    HDC302X_CMD_AUTO_MEASUREMENT_2PS_LOW_POWER_3	= 0x22FF,
    HDC302X_CMD_AUTO_MEASUREMENT_4PS_LOW_NOISE		= 0x2334,
    HDC302X_CMD_AUTO_MEASUREMENT_4PS_LOW_POWER_1	= 0x2322,
    HDC302X_CMD_AUTO_MEASUREMENT_4PS_LOW_POWER_2	= 0x2329,
    HDC302X_CMD_AUTO_MEASUREMENT_4PS_LOW_POWER_3	= 0x23FF,
    HDC302X_CMD_AUTO_MEASUREMENT_10PS_LOW_NOISE		= 0x2737,
    HDC302X_CMD_AUTO_MEASUREMENT_10PS_LOW_POWER_1 = 0x2721,
    HDC302X_CMD_AUTO_MEASUREMENT_10PS_LOW_POWER_2 = 0x272A,
    HDC302X_CMD_AUTO_MEASUREMENT_10PS_LOW_POWER_3 = 0x27FF,
    HDC302X_CMD_SOFT_RESET												= 0x30A2,
    HDC302X_CMD_HEATER_ENABLE											= 0x306D,
    HDC302X_CMD_HEATER_DISABLE										= 0x3066,
    HDC302X_CMD_READ_MANUFACTURER_ID							= 0x3781
} HDC302x_Command_t;

// HDC302x sensor structure
typedef struct {
    uint8_t address;               // I2C address of the sensor
    uint8_t heater_on;             // Heater status
    HDC302x_Mode_t mode;           // Sensor mode
} HDC302x_t;

// Structure to hold temperature and humidity data
typedef struct __attribute__((packed)) {
    float Temperature; // Temperature in degrees Celsius
    float Humidity;    // Relative Humidity in percentage
} HDC302x_Data_t;

// Function prototypes for HDC302x sensor communication
HAL_StatusTypeDef HDC302x_Init(HDC302x_t* sensor, uint8_t address);
HAL_StatusTypeDef HDC302x_ReadData(HDC302x_t* sensor, HDC302x_Data_t* data);
HAL_StatusTypeDef HDC302x_ReadTemperature(HDC302x_t* sensor, float* temperature);
HAL_StatusTypeDef HDC302x_ReadHumidity(HDC302x_t* sensor, float* humidity);
HAL_StatusTypeDef HDC302x_ResetSensor(HDC302x_t* sensor);
HAL_StatusTypeDef HDC302x_EnableHeater(HDC302x_t* sensor, HDC302x_HeaterPower_t power);
HAL_StatusTypeDef HDC302x_DisableHeater(HDC302x_t* sensor);
HAL_StatusTypeDef HDC302x_SetHeaterPower(HDC302x_t* sensor, HDC302x_HeaterPower_t power);
float HDC302x_CalculateDewPoint(float temperature, float humidity);
HAL_StatusTypeDef HDC302x_ControlHeaterBasedOnDewPoint(HDC302x_t* sensor, float temperature, float humidity);

#endif // HDC302X_H

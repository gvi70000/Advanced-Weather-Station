#include "HDC302x.h"
#include "i2c.h"
#include <math.h>

// Initialize the HDC302x sensor in Auto Measurement Mode at 10Hz with Low Noise
HAL_StatusTypeDef HDC302x_Init(HDC302x_t* sensor, uint8_t address) {
    sensor->address = address;
    sensor->heater_on = HDC302X_HEATER_OFF;
    sensor->mode = HDC302X_MODE_LOW_POWER;

    // Step 1: Reset the sensor using the reset command
    HAL_StatusTypeDef status = HDC302x_ResetSensor(sensor);
    if (status != HAL_OK) return status;

    // Step 2: Set the sensor to Auto Measurement Mode with 1Hz Low Noise configuration
    uint16_t command = HDC302X_CMD_AUTO_MEASUREMENT_1S_LOW_NOISE;
    uint8_t config_value[2] = { (uint8_t)(command >> 8), (uint8_t)(command & 0xFF) };
    status = WriteRegister(sensor->address, HDC302X_REG_CONFIG, config_value, 2, &hi2c2);
    if (status != HAL_OK) return status;

    // Step 3: Set temperature threshold to 20°C
    uint16_t tempThreshold = (uint16_t)(((20.0f + 40.0f) / 165.0f) * 65536.0f); // Convert to register value
    uint8_t temp_threshold[2] = { (uint8_t)(tempThreshold & 0xFF), (uint8_t)(tempThreshold >> 8) };
    status = WriteRegister(sensor->address, HDC302X_REG_TEMP, temp_threshold, 2, &hi2c2);
    if (status != HAL_OK) return status;

    // Step 4: Set humidity threshold to 50%
    uint16_t humThreshold = (uint16_t)((50.0f / 100.0f) * 65536.0f); // Convert to register value
    uint8_t hum_threshold[2] = { (uint8_t)(humThreshold & 0xFF), (uint8_t)(humThreshold >> 8) };
    status = WriteRegister(sensor->address, HDC302X_REG_HUMIDITY, hum_threshold, 2, &hi2c2);
    if (status != HAL_OK) return status;

    return HAL_OK; // Initialization successful
}

// Function to read both temperature and humidity from the HDC302x sensor in one I2C read operation
HAL_StatusTypeDef HDC302x_ReadData(HDC302x_t* sensor, HDC302x_Data_t* data) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[4]; // Buffer to hold temperature and humidity data
    uint16_t temp_raw, humid_raw;

    // Read 4 bytes starting from the temperature register
    status = ReadRegister(sensor->address, HDC302X_REG_TEMP, raw_data, 4, &hi2c2);
    if (status != HAL_OK) {
        return status;
    }

    // Extract temperature and humidity from the raw data
    temp_raw = (raw_data[0] << 8) | raw_data[1];
    humid_raw = (raw_data[2] << 8) | raw_data[3];

    // Convert raw values to physical values
    data->Temperature = temp_raw * HDC302X_TEMP_COEFF1 - HDC302X_TEMP_COEFF2;
    data->Humidity = humid_raw * HDC302X_RH_COEFF;

    return HAL_OK;
}

// Read temperature from the HDC302x sensor
HAL_StatusTypeDef HDC302x_ReadTemperature(HDC302x_t* sensor, float* temperature) {
    uint8_t data[2];
    HAL_StatusTypeDef status = ReadRegister(sensor->address, HDC302X_REG_TEMP, data, 2, &hi2c2);
    if (status != HAL_OK) return status;

    uint16_t temp_raw = (data[0] << 8) | data[1];
    *temperature = temp_raw * HDC302X_TEMP_COEFF1 - HDC302X_TEMP_COEFF2;
    return HAL_OK;
}

// Read humidity from the HDC302x sensor
HAL_StatusTypeDef HDC302x_ReadHumidity(HDC302x_t* sensor, float* humidity) {
    uint8_t data[2];
    HAL_StatusTypeDef status = ReadRegister(sensor->address, HDC302X_REG_HUMIDITY, data, 2, &hi2c2);
    if (status != HAL_OK) return status;

    uint16_t humid_raw = (data[0] << 8) | data[1];
    *humidity = humid_raw * HDC302X_RH_COEFF;
    return HAL_OK;
}

// Perform a software reset on the HDC302x sensor
HAL_StatusTypeDef HDC302x_ResetSensor(HDC302x_t* sensor) {
    uint16_t command = HDC302X_CMD_SOFT_RESET;
    uint8_t reset_command[2] = { (uint8_t)(command >> 8), (uint8_t)(command & 0xFF) };
    return WriteRegister(sensor->address, HDC302X_REG_RESET, reset_command, 2, &hi2c2);
}

// Enable heater with specified power level
HAL_StatusTypeDef HDC302x_EnableHeater(HDC302x_t* sensor, HDC302x_HeaterPower_t power) {
    sensor->heater_on = 1;
    return HDC302x_SetHeaterPower(sensor, power);
}

// Disable the heater
HAL_StatusTypeDef HDC302x_DisableHeater(HDC302x_t* sensor) {
    sensor->heater_on = 0;
    return HDC302x_SetHeaterPower(sensor, HDC302X_HEATER_OFF);
}

// Set the heater power on the sensor
HAL_StatusTypeDef HDC302x_SetHeaterPower(HDC302x_t* sensor, HDC302x_HeaterPower_t power) {
    uint16_t command = (uint16_t)power;
    uint8_t config_value[2] = { (uint8_t)(command >> 8), (uint8_t)(command & 0xFF) };
    return WriteRegister(sensor->address, HDC302X_REG_CONFIG, config_value, 2, &hi2c2);
}

// Calculate dew point using the Magnus-Tetens formula
float HDC302x_CalculateDewPoint(float temperature, float humidity) {
    float alpha = ((DEW_POINT_CONST_A * temperature) / (DEW_POINT_CONST_B + temperature)) + logf(humidity / 100.0f);
    float dew_point = (DEW_POINT_CONST_B * alpha) / (DEW_POINT_CONST_A - alpha);
    return dew_point;
}

// Automatically control the heater based on the dew point
HAL_StatusTypeDef HDC302x_ControlHeaterBasedOnDewPoint(HDC302x_t* sensor, float temperature, float humidity) {
    float dew_point = HDC302x_CalculateDewPoint(temperature, humidity);
    // If the dew point is within 2 degrees of the current temperature, turn on the heater
    if (temperature - dew_point < 2.0f) {
        return HDC302x_EnableHeater(sensor, HDC302X_HEATER_MEDIUM);  // Set heater to medium power if dew point is close to ambient temperature
    } else {
        return HDC302x_DisableHeater(sensor);  // Disable heater if conditions do not require it
    }
}

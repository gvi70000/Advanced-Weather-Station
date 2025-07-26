#include <string.h>
#include <stdio.h>
#include <math.h>
#include "usart.h"
#include "PGA460_REG.h"
#include "Transducers.h"
#include "PGA460.h"
#include "debug.h"

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif

// Initialize the array of ultrasonic sensors
PGA460_Sensor_t myUltraSonicArray[ULTRASONIC_SENSOR_COUNT];

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;

extern const PGA460_TGV_t TGV_25_STRUCT;
extern const PGA460_TGV_t TGV_50_STRUCT;
extern const PGA460_TGV_t TGV_75_STRUCT;

const CommandArray commands[PGA_CMD_COUNT] = {
    {{PGA460_SYNC, PGA460_CMD_BURST_AND_LISTEN_PRESET1, 255 - PGA460_CMD_BURST_AND_LISTEN_PRESET1}},			// PGA460_CMD_BURST_AND_LISTEN_PRESET1
    {{PGA460_SYNC, PGA460_CMD_BURST_AND_LISTEN_PRESET2, 255 - PGA460_CMD_BURST_AND_LISTEN_PRESET2}},			// PGA460_CMD_BURST_AND_LISTEN_PRESET2
    {{PGA460_SYNC, PGA460_CMD_LISTEN_ONLY_PRESET1, 255 - PGA460_CMD_LISTEN_ONLY_PRESET1}},						// PGA460_CMD_LISTEN_ONLY_PRESET1
    {{PGA460_SYNC, PGA460_CMD_LISTEN_ONLY_PRESET2, 255 - PGA460_CMD_LISTEN_ONLY_PRESET2}},						// PGA460_CMD_LISTEN_ONLY_PRESET2
    {{PGA460_SYNC, PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT, 255 - PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT}},		// PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT
    {{PGA460_SYNC, PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT, 255 - PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT}},	// PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT
    {{PGA460_SYNC, PGA460_CMD_TEMP_AND_NOISE_RESULT, 255 - PGA460_CMD_TEMP_AND_NOISE_RESULT}},					// PGA460_CMD_TEMP_AND_NOISE_RESULT
    {{PGA460_SYNC, PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP, 255 - PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP}},			// PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP
    {{PGA460_SYNC, PGA460_CMD_SYSTEM_DIAGNOSTICS, 255 - PGA460_CMD_SYSTEM_DIAGNOSTICS}},						// PGA460_CMD_SYSTEM_DIAGNOSTICS
    {{PGA460_SYNC, PGA460_CMD_REGISTER_READ, 255 - PGA460_CMD_REGISTER_READ}},									// PGA460_CMD_REGISTER_READ
    {{PGA460_SYNC, PGA460_CMD_REGISTER_WRITE, 255 - PGA460_CMD_REGISTER_WRITE}},								// PGA460_CMD_REGISTER_WRITE
    {{PGA460_SYNC, PGA460_CMD_EEPROM_BULK_READ, 255 - PGA460_CMD_EEPROM_BULK_READ}},							// PGA460_CMD_EEPROM_BULK_READ
    {{PGA460_SYNC, PGA460_CMD_EEPROM_BULK_WRITE, 255 - PGA460_CMD_EEPROM_BULK_WRITE}},							// PGA460_CMD_EEPROM_BULK_WRITE
    {{PGA460_SYNC, PGA460_CMD_TVG_BULK_READ, 255 - PGA460_CMD_TVG_BULK_READ}},									// PGA460_CMD_TVG_BULK_READ
    {{PGA460_SYNC, PGA460_CMD_TVG_BULK_WRITE, 255 - PGA460_CMD_TVG_BULK_WRITE}},								// PGA460_CMD_TVG_BULK_WRITE
    {{PGA460_SYNC, PGA460_CMD_THRESHOLD_BULK_READ, 255 - PGA460_CMD_THRESHOLD_BULK_READ}},						// PGA460_CMD_THRESHOLD_BULK_READ
    {{PGA460_SYNC, PGA460_CMD_THRESHOLD_BULK_WRITE, 255 - PGA460_CMD_THRESHOLD_BULK_WRITE}},					// PGA460_CMD_THRESHOLD_BULK_WRITE
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1, 255 - PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1}},	// PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2, 255 - PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2}},	// PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_LISTEN_ONLY_P1, 255 - PGA460_CMD_BROADCAST_LISTEN_ONLY_P1}},			// PGA460_CMD_BROADCAST_LISTEN_ONLY_P1
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_LISTEN_ONLY_P2, 255 - PGA460_CMD_BROADCAST_LISTEN_ONLY_P2}},			// PGA460_CMD_BROADCAST_LISTEN_ONLY_P2
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_TEMP_NOISE_MEASURE, 255 - PGA460_CMD_BROADCAST_TEMP_NOISE_MEASURE}},	// PGA460_CMD_BROADCAST_TEMP_NOISE_MEASURE
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_REGISTER_WRITE, 255 - PGA460_CMD_BROADCAST_REGISTER_WRITE}},			// PGA460_CMD_BROADCAST_REGISTER_WRITE
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_EEPROM_BULK_WRITE, 255 - PGA460_CMD_BROADCAST_EEPROM_BULK_WRITE}},		// PGA460_CMD_BROADCAST_EEPROM_BULK_WRITE
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_TVG_BULK_WRITE, 255 - PGA460_CMD_BROADCAST_TVG_BULK_WRITE}},			// PGA460_CMD_BROADCAST_TVG_BULK_WRITE
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_THRESHOLD_BULK_WRITE, 255 - PGA460_CMD_BROADCAST_THRESHOLD_BULK_WRITE}}	// PGA460_CMD_BROADCAST_THRESHOLD_BULK_WRITE
};

// Reflection Path Length
#define DISTANCE 0.34308            // Total reflection path length in meters
#define US_TO_SEC 1e-6              // Conversion factor from microseconds to seconds

#define R_D 287.05                  // Specific gas constant for dry air (J/kg·K)
#define R_V 461.495                 // Specific gas constant for water vapor (J/kg·K)
#define GAMMA 1.4                   // Adiabatic index for air
#define L 0.0065                    // Temperature lapse rate (K/m)
#define T0 288.15                   // Standard temperature at sea level (K)
#define P0 101325.0                 // Standard pressure at sea level (Pa)
#define G 9.80665                   // Gravitational acceleration (m/s²)
#define M 0.0289644                 // Molar mass of air (kg/mol)
#define R 8.3144598                 // Universal gas constant (J/(mol·K))
#define KELVIN_OFFSET 273.15        // Conversion from Celsius to Kelvin
#define RH_DIVISOR 100.0            // Converts RH percentage to a fraction
#define WATER_VAPOR_EFFECT 0.6077   // Effect of water vapor on speed of sound
#define SATURATION_CONSTANT 6.1078  // Constant for saturation vapor pressure calculation

	typedef struct __attribute__((packed)) {
		float Height;
		float Temperature;
		float RH;
		float Pressure;
		float SoundSpeed;
	} PGA460_EnvData_t;

PGA460_EnvData_t externalData;

float soundSpeed = 343.0f;
	

PGA460_Diag_t LastDiag;
static uint8_t EEPROM_Stat = 0; // EEPROM 0 is Locked, 1 is Unlocked

static void PGA460_PrintDiagnostic(PGA460_Diag_t diag) {
    if ((diag.Value & 0xC0) != 0x40) {
        DEBUG("Invalid Diagnostic Marker (expected 0x40-0x7F, got 0x%02X)\n", diag.Value);
        return;
    }
    if (diag.BitField.DeviceBusy)        DEBUG("PGA460: Device was busy\n");
    if (diag.BitField.SyncRateError)     DEBUG("PGA460: Sync rate too high/low\n");
    if (diag.BitField.SyncWidthMismatch) DEBUG("PGA460: Sync field width mismatch\n");
    if (diag.BitField.ChecksumMismatch)  DEBUG("PGA460: Checksum mismatch (controller vs device)\n");
    if (diag.BitField.InvalidCommand)    DEBUG("PGA460: Invalid command sent\n");
    if (diag.BitField.UARTFrameError)    DEBUG("PGA460: UART frame/stop bit/contention error\n");
}

static uint8_t PGA460_CalculateChecksum(const uint8_t *data, uint8_t len) {
    uint16_t carry = 0;
    for (uint8_t i = 0; i < len; i++) {
        carry += data[i];
        if (carry > 0xFF) {
            carry -= 255;
        }
    }
    return (uint8_t)(~carry & 0xFF);
}

static uint8_t PGA460_CalculateEEPROM_CRC(const uint8_t *data, uint8_t len) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(~sum + 1);  // 2's complement
}

static PGA460_Error_t PGA460_SetEEPROMAccess(const uint8_t sensorID, const uint8_t unlock) {
    uint8_t accessByte = unlock ? PGA460_UNLOCK_EEPROM : PGA460_LOCK_EEPROM;
    uint8_t cmd[5] = {PGA460_SYNC, PGA460_CMD_REGISTER_WRITE, REG_EE_CTRL, accessByte, 0x00};
    cmd[4] = PGA460_CalculateChecksum(&cmd[1], 3); // Checksum over CMD, ADDR, DATA
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, cmd, sizeof(cmd), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM %s failed\n", sensorID, unlock ? "unlock" : "lock");
        return PGA460_ERR_UART_TX;
    }
    // Update status flag on success
    EEPROM_Stat = unlock ? 1 : 0;
    return PGA460_ERR_NONE;
}

static void PrintThresholdStruct(const PGA460_THR_t *thr) {
    printf("P1: ");
    for (int i = 0; i < 16; ++i) {
        uint8_t val = *((uint8_t*)thr + i);
        printf("0x%02X, ", val);
    }
    printf("\nP2: ");
    for (int i = 16; i < 32; ++i) {
        uint8_t val = *((uint8_t*)thr + i);
        printf("0x%02X, ", val);
    }
    printf("\nCRC: 0x%02X\n", thr->THR_CRC);
}
// Function to calculate the speed of sound
void calculateSpeedOfSound(void) {
		float T_k = externalData.Temperature + KELVIN_OFFSET;

    float P_total = externalData.Pressure;
    if (externalData.Height != 0) {
        if (externalData.Height > 0) {
            P_total = P0 * powf((1 - (L * externalData.Height) / T0), (G * M) / (R * L));
        } else {
            P_total = P0 * powf((1 + (L * fabsf(externalData.Height)) / T0), (G * M) / (R * L));
        }
    }

    float P_sat = SATURATION_CONSTANT * powf(10, (7.5f * externalData.Temperature) / (externalData.Temperature + 237.3f)) * 100;
    float P_v = P_sat * (externalData.RH / RH_DIVISOR);  // Partial pressure of water vapor
    float P_d = P_total - P_v;                      // Partial pressure of dry air
    float H = P_v / P_d;

    externalData.SoundSpeed = sqrtf(GAMMA * R_D * T_k * (1 + WATER_VAPOR_EFFECT * H));
}

// Function to calculate wind speed and direction
void calculateWind(uint32_t ToF_up[3], uint32_t ToF_down[3], float speed_of_sound, float *wind_speed, float *wind_direction) {
    float delta_t[3] = {0.0f}, avg_t[3] = {0.0f};
    float v_x = 0.0f, v_y = 0.0f;

    for (int i = 0; i < 3; i++) {
        float ToF_up_sec = ToF_up[i] * US_TO_SEC;
        float ToF_down_sec = ToF_down[i] * US_TO_SEC;

        delta_t[i] = ToF_down_sec - ToF_up_sec;
        avg_t[i] = (ToF_up_sec + ToF_down_sec) / 2.0f;

        if (fabsf(avg_t[i]) < 1e-9f) {
            DEBUG("Error: avg_t[%d] is too small!\n", i);
            *wind_speed = 0.0f;
            *wind_direction = 0.0f;
            return;
        }
    }

    v_x = (DISTANCE / 2.0f) * (delta_t[0] / avg_t[0]);
    v_y = (DISTANCE / 2.0f) * (delta_t[1] / avg_t[1]);

    *wind_speed = sqrtf(v_x * v_x + v_y * v_y);
    *wind_direction = atan2f(v_y, v_x) * (180.0f / M_PI);
    if (*wind_direction < 0) {
        *wind_direction += 360.0f;
    }
}

// Function to initialize 3x PGA460 sensors
PGA460_Error_t PGA460_Init(void) {
    static const UART_HandleTypeDef *const uartPorts[ULTRASONIC_SENSOR_COUNT] = {&huart1, &huart4, &huart5};
    for (uint8_t i = 0; i < ULTRASONIC_SENSOR_COUNT; i++) {
        // Step 1: Assign UART and transducer config
        myUltraSonicArray[i].uartPort = (UART_HandleTypeDef *)uartPorts[i];
        myUltraSonicArray[i].PGA460_Data = transducer;
        HAL_UART_Init(myUltraSonicArray[i].uartPort);
        HAL_Delay(100);
        // Step 2: Bulk Threshold Write (clear THR_CRC_ERR)
        if (PGA460_SetThresholds(i, PGA460_TRH_CC) != HAL_OK) {
            DEBUG("Sensor %d: Threshold Write Failed!\n", i);
            return PGA460_ERR_GEN;
        }
//				if (PGA460_AutoThreshold(i, 15, 0, 255, 3) != HAL_OK) {
//            DEBUG("Sensor %d: Auto THR Failed!\n", i);
//            return PGA460_ERR_GEN;
//				}
        // Step 4: TVG Write
        if (PGA460_SetTVG(i, PGA460_GAIN_58_90dB, PGA460_TVG_CUSTOM) != HAL_OK) {
            DEBUG("Sensor %d: TVG Bulk Write Failed!\n", i);
            return PGA460_ERR_GEN;
        }
        // Step 3: EEPROM Bulk Write
        if (PGA460_EEPROMBulkWrite(i) != HAL_OK) {
            DEBUG("Sensor %d: EEPROM Write Failed!\n", i);
            return PGA460_ERR_GEN;
        }
        // Step 5: Diagnostics
        if (PGA460_CheckStatus(i) != HAL_OK) {
            DEBUG("Sensor %d: Status Check Failed!\n", i);
            return PGA460_ERR_GEN;
        }
        float diagValue = 0.0;
        if (PGA460_GetSystemDiagnostics(i, 1, 0, &diagValue) == HAL_OK)
            DEBUG("Sensor %d: Frequency Diagnostic = %.2f kHz\n", i, diagValue);
        if (PGA460_GetSystemDiagnostics(i, 0, 1, &diagValue) == HAL_OK)
            DEBUG("Sensor %d: Decay Period Diagnostic = %.2f us\n", i, diagValue);
        if (PGA460_GetSystemDiagnostics(i, 0, 2, &diagValue) == HAL_OK)
            DEBUG("Sensor %d: Die Temperature = %.2f C\n", i, diagValue);
        if (PGA460_GetSystemDiagnostics(i, 0, 3, &diagValue) == HAL_OK)
            DEBUG("Sensor %d: Noise Level = %.0f\n", i, diagValue);
        // Step 6: Optional EEPROM Burn
        // if (PGA460_BurnEEPROM(i) != HAL_OK) {
        //     DEBUG("Sensor %d: EEPROM Burn Failed!\n", i);
        //     return PGA460_ERR_GEN;
        // }

        // Step 7: Optional Echo Data Dump (commented out for normal operation)
				uint8_t echoBuf[128];
				if (PGA460_GetEchoDataDump(i, PGA460_CMD_BURST_AND_LISTEN_PRESET1, echoBuf) != PGA460_ERR_NONE)
						return PGA460_ERR_GEN;
				DEBUG("Sensor %d: Echo Data Dump:\n", i);
				for (uint8_t j = 0; j < 128; j++) {
						DEBUG("%d%s", echoBuf[j], (j < 127) ? "," : "\n");
						//if (echoBuf[j] < 200) continue;
						//		printf("Sensor %d: Echo at index %d = %d\n", i, j, echoBuf[j]);
				}
				DEBUG("-----\n");
				PrintThresholdStruct(&myUltraSonicArray[i].PGA460_Data.THR);
				DEBUG("-----\n");
				HAL_Delay(1000);
    }
    return PGA460_ERR_NONE;
}

PGA460_Error_t PGA460_CheckStatus(const uint8_t sensorID) {
    // Read both DEV_STAT registers
    if (PGA460_RegisterRead(sensorID, REG_DEV_STAT0, &myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT0.Val.Value) != HAL_OK ||
        PGA460_RegisterRead(sensorID, REG_DEV_STAT1, &myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT1.Val.Value) != HAL_OK) {
        DEBUG("Sensor %d: Communication error! Could not read status registers.\n", (int)sensorID);
        return PGA460_ERR_GEN;
    }
    // --- DEV_STAT0 Flags ---
    if (myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT0.Val.BitField.CMW_WU_ERR)  DEBUG("Sensor %d: [CMW_WU_ERR] Wake-up error!\n", sensorID);
    if (myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT0.Val.BitField.THR_CRC_ERR) DEBUG("Sensor %d: [THR_CRC_ERR] Threshold CRC error!\n", sensorID);
    if (myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT0.Val.BitField.EE_CRC_ERR)  DEBUG("Sensor %d: [EE_CRC_ERR] EEPROM CRC error!\n", sensorID);
    if (myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT0.Val.BitField.TRIM_CRC_ERR)DEBUG("Sensor %d: [TRIM_CRC_ERR] Trim CRC error!\n", sensorID);
    //if ((myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT0.Val.Value & 0x0F) == 0)   DEBUG("Sensor %d: DEV_STAT0 CRC flags all OK.\n", sensorID);
    // --- DEV_STAT1 Flags ---
    if (myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT1.Val.BitField.TSD_PROT)    DEBUG("Sensor %d: [TSD_PROT] Thermal shutdown occurred!\n", sensorID);
    if (myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT1.Val.BitField.IOREG_OV)    DEBUG("Sensor %d: [IOREG_OV] IOREG Overvoltage!\n", sensorID);
    if (myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT1.Val.BitField.IOREG_UV)    DEBUG("Sensor %d: [IOREG_UV] IOREG Undervoltage!\n", sensorID);
    if (myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT1.Val.BitField.AVDD_OV)     DEBUG("Sensor %d: [AVDD_OV] AVDD Overvoltage!\n", sensorID);
    if (myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT1.Val.BitField.AVDD_UV)     DEBUG("Sensor %d: [AVDD_UV] AVDD Undervoltage!\n", sensorID);
    if (myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT1.Val.BitField.VPWR_OV)     DEBUG("Sensor %d: [VPWR_OV] VPWR Overvoltage!\n", sensorID);
    if (myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT1.Val.BitField.VPWR_UV)     DEBUG("Sensor %d: [VPWR_UV] VPWR Undervoltage!\n", sensorID);
    //if ((myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT1.Val.Value & 0x7F) == 0)  DEBUG("Sensor %d: DEV_STAT1 power and thermal status all OK.\n", sensorID);
    return PGA460_ERR_NONE;
}

// Function to read a register from the PGA460
PGA460_Error_t PGA460_RegisterRead(const uint8_t sensorID, const uint8_t regAddr, uint8_t *regValue) {
    // Prepare register read command: SYNC, CMD, ADDR, CHK
    uint8_t cmd[PGA_READ_SIZE] = {PGA460_SYNC, PGA460_CMD_REGISTER_READ, regAddr, 0x00};// checksum placeholder
    cmd[3] = PGA460_CalculateChecksum(&cmd[1], 2);  // Calculate checksum over CMD and ADDR
    // Optional: flush UART RX buffer to clear previous data
    __HAL_UART_FLUSH_DRREGISTER(myUltraSonicArray[sensorID].uartPort);
    // Transmit command frame
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, cmd, PGA_READ_SIZE, UART_TIMEOUT) != HAL_OK) {
        return PGA460_ERR_UART_TX;
    }
		//memset(cmd, 0, PGA_READ_SIZE);
    // Receive 3-byte response: [CMD echo, DATA, CHK]
		
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, cmd, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        return PGA460_ERR_UART_RX;
    }
    *regValue = cmd[1];
    return PGA460_ERR_NONE;
}

// Function to write a register on the PGA460
PGA460_Error_t PGA460_RegisterWrite(const uint8_t sensorID, const uint8_t regAddr, const uint8_t regValue) {
    // Prepare frame: SYNC, CMD, ADDR, DATA, CHK
    uint8_t txFrame[PGA_WRITE_SIZE] = {PGA460_SYNC, PGA460_CMD_REGISTER_WRITE, regAddr, regValue, 0x00};
    txFrame[4] = PGA460_CalculateChecksum(&txFrame[1], 3);
    // Transmit frame
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, txFrame, PGA_WRITE_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Register Write TX Failed! Addr 0x%02X\n", sensorID, regAddr);
        return PGA460_ERR_UART_TX;
    }
		HAL_Delay(10);
    // EEPROM auto-locks after write, no manual lock required
    return PGA460_ERR_NONE;
}

PGA460_Error_t PGA460_EEPROMBulkRead(const uint8_t sensorID) {
    uint8_t dataBuffer[45] = {0};
    // Step 1: Send EEPROM Bulk Read Command
		if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_EEPROM_BULK_READ].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Bulk Read Command Failed!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
		HAL_Delay(10);
    // Step 2: Receive EEPROM content (0x00 to 0x2C = 44 bytes + 1 diag/status = 45)
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, dataBuffer, sizeof(dataBuffer), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Bulk Read Failed!\n", sensorID);
        return PGA460_ERR_UART_RX;
    }
		PGA460_PrintDiagnostic((PGA460_Diag_t){ .Value = dataBuffer[0] });
    // Optional: Check for bus idle (0xFF flood)
    uint8_t idle = 1;
    for (int i = 0; i < sizeof(dataBuffer); i++) {
        if (dataBuffer[i] != 0xFF) {
            idle = 0;
            break;
        }
    }
    if (idle) {
        DEBUG("Sensor %d: EEPROM Read response was idle (0xFF)\n", sensorID);
        return PGA460_ERR_TIMEOUT;
    }
    // Step 3: Copy EEPROM content [1..44] to struct (skip byte 0 = status/diag)
    memcpy(&myUltraSonicArray[sensorID].PGA460_Data.EEPROM, &dataBuffer[1], sizeof(PGA460_EEPROMConfig_t));
    // Optional: Validate EE_CRC (last byte is .EE_CRC at offset 0x2B)
    const uint8_t *eepromBytes = (const uint8_t *)&myUltraSonicArray[sensorID].PGA460_Data.EEPROM;
    uint8_t expectedCRC = PGA460_CalculateEEPROM_CRC(eepromBytes, 43);
    if (eepromBytes[43] != expectedCRC) {
        DEBUG("Sensor %d: EEPROM CRC mismatch! Expected 0x%02X, Got 0x%02X\n", sensorID, expectedCRC, eepromBytes[43]);
        return PGA460_ERR_CHECKSUM;
    }
    DEBUG("Sensor %d: EEPROM Bulk Read Successful.\n", sensorID);
    return PGA460_ERR_NONE;
}

// EEPROM bulk write
PGA460_Error_t PGA460_EEPROMBulkWrite(uint8_t sensorID) {
    uint8_t frame[46];
    uint8_t *eepromBytes = (uint8_t*)&myUltraSonicArray[sensorID].PGA460_Data.EEPROM;
    // Step 1: Unlock EEPROM if needed
    if (EEPROM_Stat == 0) {
        if (PGA460_SetEEPROMAccess(sensorID, 1) != HAL_OK) {
            DEBUG("Sensor %d: EEPROM unlock failed before bulk write\n", sensorID);
            return PGA460_ERR_GEN;
        }
    }
    // Step 2: Update EEPROM CRC (covers bytes 0–42)
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.EE_CRC = PGA460_CalculateEEPROM_CRC(eepromBytes, 43);
    // Step 3: Construct UART frame
    frame[0] = PGA460_SYNC;
    frame[1] = PGA460_CMD_EEPROM_BULK_WRITE;
    memcpy(&frame[2], eepromBytes, 44);  // 44 bytes
    frame[46 - 1] = PGA460_CalculateChecksum(&frame[1], 44);  // Checksum over CMD + EEPROM bytes
    // Step 4: Transmit
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, frame, sizeof(frame), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Bulk Write Failed!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
    HAL_Delay(70);  // Wait for EEPROM write
		PGA460_CheckStatus(sensorID);
    return PGA460_ERR_NONE;
}

PGA460_Error_t PGA460_BurnEEPROM(uint8_t sensorID) {
    // Step 1: Send 0x68 to REG_EE_CTRL to unlock EEPROM with EE_PRGM=0
    uint8_t unlockFrame[5] = {PGA460_SYNC, PGA460_CMD_REGISTER_WRITE, REG_EE_CTRL, PGA460_UNLOCK_EEPROM, 0x00};// EE_UNLCK=0xD, EE_PRGM=0
    unlockFrame[4] = PGA460_CalculateChecksum(&unlockFrame[1], 3);
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, unlockFrame, sizeof(unlockFrame), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Burn Step 1 (0x68) Failed!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
    HAL_Delay(1);  // Short delay per spec
    // Step 2: Send 0x69 to REG_EE_CTRL to trigger EEPROM burn (EE_PRGM = 1)
    uint8_t burnFrame[5] = {PGA460_SYNC, PGA460_CMD_REGISTER_WRITE, REG_EE_CTRL, PGA460_LOCK_EEPROM, 0x00};// EE_UNLCK=0xD, EE_PRGM=1
    burnFrame[4] = PGA460_CalculateChecksum(&burnFrame[1], 3);
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, burnFrame, sizeof(burnFrame), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Burn Step 2 (0x69) Failed!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
    HAL_Delay(1000);  // Burn delay per datasheet (typ. 600–1000 ms)
    // Step 3: Read back REG_EE_CTRL to verify burn success
    uint8_t ctrlVal = 0;
    if (PGA460_RegisterRead(sensorID, REG_EE_CTRL, &ctrlVal) != HAL_OK) {
        DEBUG("Sensor %d: Failed to read EE_CTRL after burn!\n", sensorID);
        return PGA460_ERR_UART_RX;
    }
    myUltraSonicArray[sensorID].PGA460_Data.EE_CNTRL.Val.Value = ctrlVal;
    // Step 4: Check EE_PRGM_OK bit (bit 2)
    if (myUltraSonicArray[sensorID].PGA460_Data.EE_CNTRL.Val.BitField.EE_PRGM_OK) {
        DEBUG("Sensor %d: EEPROM Burn Successful (EE_CTRL = 0x%02X)\n", sensorID, ctrlVal);
        return PGA460_ERR_NONE;
    } else {
        DEBUG("Sensor %d: EEPROM Burn Failed (EE_CTRL = 0x%02X)\n", sensorID, ctrlVal);
        return PGA460_ERR_EEPROM_BURN;
    }
}

PGA460_Error_t PGA460_GetTVG(uint8_t sensorID) {
    // Step 1: Send the TVG Bulk Read command (0x0D)
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_TVG_BULK_READ].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK)
    {
        DEBUG("Sensor %d: TVG Bulk Read command failed to send!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
    // Step 2: Receive 7 bytes directly into EEPROM.TGV
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, (uint8_t *)&myUltraSonicArray[sensorID].PGA460_Data.EEPROM.TGV, sizeof(PGA460_TGV_t), UART_TIMEOUT) != HAL_OK)
    {
        DEBUG("Sensor %d: Failed to receive TVG Bulk Read response!\n", sensorID);
        return PGA460_ERR_UART_RX;
    }
    return PGA460_ERR_NONE;
}

PGA460_Error_t PGA460_SetTVG(const uint8_t sensorID, const PGA460_GainRange_t gain_range, const PGA460_TVG_Level_t timeVaryingGain) {
    // Step 1: Set gain range in EEPROM shadow + write to PGA460
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.DECPL_TEMP.Val.BitField.AFE_GAIN_RNG = (AFE_GAIN_RNG_t)gain_range;

    if (PGA460_RegisterWrite(sensorID, REG_DECPL_TEMP, gain_range) != HAL_OK) {
        DEBUG("Sensor %d: InitTVG AFE_GAIN_RNG Failed!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
    // Step 2: Prepare UART TVG frame
    uint8_t frame[10] = {
        PGA460_SYNC,
        PGA460_CMD_TVG_BULK_WRITE
    };

    // Step 3: Copy selected TVG config directly
    switch (timeVaryingGain) {
        case PGA460_TVG_25_PERCENT:
            memcpy(&myUltraSonicArray[sensorID].PGA460_Data.EEPROM.TGV, &TGV_25_STRUCT, sizeof(PGA460_TGV_t));
            break;

        case PGA460_TVG_50_PERCENT:
            memcpy(&myUltraSonicArray[sensorID].PGA460_Data.EEPROM.TGV, &TGV_50_STRUCT, sizeof(PGA460_TGV_t));
             break;

        case PGA460_TVG_75_PERCENT:
            memcpy(&myUltraSonicArray[sensorID].PGA460_Data.EEPROM.TGV, &TGV_75_STRUCT, sizeof(PGA460_TGV_t));
            break;

        case PGA460_TVG_CUSTOM:
            memcpy(&myUltraSonicArray[sensorID].PGA460_Data.EEPROM.TGV, &TGV_CUSTOM_STRUCT, sizeof(PGA460_TGV_t));
            break;
    }
		memcpy(&frame[2], &myUltraSonicArray[sensorID].PGA460_Data.EEPROM.TGV, sizeof(PGA460_TGV_t));
    // Step 4: Apply checksum over CMD + 7 bytes of TVG
    frame[9] = PGA460_CalculateChecksum(&frame[1], 8);
    // Step 5: Transmit TVG configuration
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, frame, sizeof(frame), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: TVG Bulk Write Transmission Failed!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
    HAL_Delay(70);  // Let PGA460 apply new TVG settings
		PGA460_CheckStatus(sensorID);
    return PGA460_ERR_NONE;
}

PGA460_Error_t PGA460_GetThresholds(const uint8_t sensorID) {
/*     // Step 1: Send threshold bulk read command
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_THRESHOLD_BULK_READ].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Threshold Bulk Read Request Failed!\n", sensorID);
        return HAL_ERROR;
    }
    HAL_Delay(10);  // Wait for sensor to respond
    // Step 2: Receive 32 bytes directly into EEPROM shadow struct
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, (uint8_t *)&myUltraSonicArray[sensorID].PGA460_Data.THR, sizeof(PGA460_THR_t), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Threshold Bulk Read Failed!\n", sensorID);
        return HAL_ERROR;
    }
    //DEBUG("Sensor %d: Threshold Bulk Read Successful. CRC = 0x%02X\n", sensorID, myUltraSonicArray[sensorID].PGA460_Data.THR.THR_CRC);
    return PGA460_ERR_NONE; */
    PGA460_THR_t *thrMap = &myUltraSonicArray[sensorID].PGA460_Data.THR;
    uint8_t *dest = (uint8_t *)thrMap;
    const uint8_t startAddr = REG_P1_THR_0; // 0x5F

    for (uint8_t i = 0; i < sizeof(PGA460_THR_t); i++) {
        uint8_t value = 0;
        if (PGA460_RegisterRead(sensorID, startAddr + i, &value) != PGA460_ERR_NONE) {
            return PGA460_ERR_GET_THR_FAIL;
        }
        dest[i] = value;
    }
	return PGA460_ERR_NONE;
}

PGA460_Error_t PGA460_SetThresholds(const uint8_t sensorID, PGA460_TRH_Level_t thresholdLevel) {
    uint8_t frame[35] = {0};
    frame[0] = PGA460_SYNC;
    frame[1] = PGA460_CMD_THRESHOLD_BULK_WRITE;
    // Step 1: Select and copy threshold data (only 32 bytes) into UART frame directly
    switch (thresholdLevel) {
        case PGA460_TRH_25:
            memcpy(&frame[2], &THRESHOLD_25_STRUCT, 33);
            memcpy(&myUltraSonicArray[sensorID].PGA460_Data.THR, &THRESHOLD_25_STRUCT, sizeof(PGA460_THR_t));
            break;
        case PGA460_TRH_50:
            memcpy(&frame[2], &THRESHOLD_50_STRUCT, 33);
            memcpy(&myUltraSonicArray[sensorID].PGA460_Data.THR, &THRESHOLD_50_STRUCT, sizeof(PGA460_THR_t));
            break;
        case PGA460_TRH_75:
            memcpy(&frame[2], &THRESHOLD_75_STRUCT, 33);
            memcpy(&myUltraSonicArray[sensorID].PGA460_Data.THR, &THRESHOLD_75_STRUCT, sizeof(PGA460_THR_t));
            break;
        case PGA460_TRH_CC:
            memcpy(&frame[2], &THRESHOLD_CC_STRUCT, 33);
            memcpy(&myUltraSonicArray[sensorID].PGA460_Data.THR, &THRESHOLD_CC_STRUCT, sizeof(PGA460_THR_t));
            break;
        default:
            DEBUG("Sensor %d: Invalid threshold level!\n", sensorID);
            return PGA460_ERR_INVALID_CMD;
    }
    // Step 2: Append UART checksum
    frame[34] = PGA460_CalculateChecksum(&frame[1], 33);  // CMD + 32 bytes
    // Step 3: Transmit frame
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, frame, sizeof(frame), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Threshold Bulk Write Failed!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
    HAL_Delay(70);  // Let device apply thresholds
    // Step 4: Verify THR_CRC_ERR is cleared
    PGA460_CheckStatus(sensorID);
    return PGA460_ERR_NONE;
}

PGA460_Error_t PGA460_UltrasonicCmd(const uint8_t sensorID, const PGA460_Command_t cmd) {
    switch (cmd) {
        case PGA460_CMD_BURST_AND_LISTEN_PRESET1:
        case PGA460_CMD_BURST_AND_LISTEN_PRESET2:
        case PGA460_CMD_LISTEN_ONLY_PRESET1:
        case PGA460_CMD_LISTEN_ONLY_PRESET2:
        case PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1:
        case PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2:
        case PGA460_CMD_BROADCAST_LISTEN_ONLY_P1:
        case PGA460_CMD_BROADCAST_LISTEN_ONLY_P2:
        {
            // 4-byte command: SYNC + CMD + OBJ + CHK
            uint8_t frame[4] = {commands[cmd].cmdData[0], commands[cmd].cmdData[1], PGA_OBJECTS_TRACKED, 0x00};
            frame[3] = PGA460_CalculateChecksum(&frame[1], 2);  // CMD + OBJ
            if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, frame, sizeof(frame), UART_TIMEOUT) != HAL_OK) {
                DEBUG("Sensor %d: Ultrasonic Command (4-byte) Failed! CMD = 0x%02X\n", sensorID, frame[1]);
                return PGA460_ERR_UART_TX;
            }
            break;
        }
        default:
        {
            // 3-byte command: SYNC + CMD + CHK (precomputed)
            if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[cmd].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
                DEBUG("Sensor %d: Ultrasonic Command (3-byte) Failed! CMD = 0x%02X\n", sensorID, commands[cmd].cmdData[1]);
                return PGA460_ERR_UART_TX;
            }
            break;
        }
    }
    HAL_Delay(70); // Wait for up to 65ms capture time
    return PGA460_ERR_NONE;
}

PGA460_Error_t PGA460_AutoThreshold(uint8_t sensorID, uint8_t noiseMargin, uint8_t windowIndex, uint8_t autoMax, uint8_t loops) {
    PGA460_THR_t *thr = &myUltraSonicArray[sensorID].PGA460_Data.THR;
    PGA460_Error_t status = PGA460_ERR_NONE;
    // Run autotuning for P1
    status = PGA460_AutoThreshold_Internal(sensorID, PGA460_CMD_BURST_AND_LISTEN_PRESET1, noiseMargin, windowIndex, autoMax, loops, thr);
    if (status != PGA460_ERR_NONE) return status;
    // Run autotuning for P2
    status = PGA460_AutoThreshold_Internal(sensorID, PGA460_CMD_BURST_AND_LISTEN_PRESET2, noiseMargin, windowIndex, autoMax, loops, thr);
    if (status != PGA460_ERR_NONE) return status;
    // Calculate CRC for full threshold struct (addresses 0x5F to 0x7E)
    thr->THR_CRC = PGA460_CalculateEEPROM_CRC((const uint8_t *)thr, sizeof(PGA460_THR_t) - 1);
    // Write updated thresholds to device
    memcpy(&THRESHOLD_CC_STRUCT, thr, sizeof(PGA460_THR_t));
    if (PGA460_SetThresholds(sensorID, PGA460_TRH_CC) != HAL_OK) {
        DEBUG("Sensor %d: Failed to apply new thresholds!\n", sensorID);
        return PGA460_ERR_GEN;
    }
    return PGA460_ERR_NONE;
}

PGA460_Error_t PGA460_AutoThreshold_Internal(uint8_t sensorID, PGA460_Command_t cmd, uint8_t noiseMargin, uint8_t windowIndex, uint8_t autoMax, uint8_t loops, PGA460_THR_t *thr) {
    uint8_t echoSum[128] = {0};
    uint8_t echoBuf[128];
    uint8_t maxVals[12] = {0};
    uint8_t eddMarkers[13] = {0};
    uint16_t recTime_us = 0;
    // Step 1: Read REC_LENGTH and convert to microseconds
    uint8_t rec = 0;
    if (PGA460_RegisterRead(sensorID, REG_REC_LENGTH, &rec) != HAL_OK)
        return PGA460_ERR_UART_RX;
    rec = ((cmd == PGA460_CMD_BURST_AND_LISTEN_PRESET1 || cmd == PGA460_CMD_LISTEN_ONLY_PRESET1) ? (rec >> 4) : (rec & 0x0F)) & 0x0F;
    recTime_us = (rec + 1) * 4096;
    // Step 2: Calculate threshold window markers
    for (uint8_t i = 0; i <= 12; i++) {
        eddMarkers[i] = (i * 128 * 100) / (recTime_us / 100);
        if (eddMarkers[i] > 127) eddMarkers[i] = 127;
    }
    // Step 3: Capture and print BEFORE echo
    if (PGA460_GetEchoDataDump(sensorID, cmd, echoBuf) != PGA460_ERR_NONE)
        return PGA460_ERR_GEN;
    DEBUG("Sensor %d: Echo Data Dump BEFORE:\n", sensorID);
    for (uint8_t i = 0; i < 128; i++) {
        DEBUG("%d%s", echoBuf[i], (i < 127) ? "," : "\n");
    }
    // Step 4: Average echo over multiple loops
    memset(echoSum, 0, sizeof(echoSum));
    for (uint8_t l = 0; l < loops; l++) {
        if (PGA460_GetEchoDataDump(sensorID, cmd, echoBuf) != PGA460_ERR_NONE)
            return PGA460_ERR_GEN;
        for (uint8_t i = 0; i < 128; i++) {
            echoSum[i] += echoBuf[i];
        }
    }
    for (uint8_t i = 0; i < 128; i++) {
        echoSum[i] /= loops;
    }
    // Step 5: Partition max values
    for (uint8_t p = 0; p < autoMax; p++) {
        for (uint8_t i = eddMarkers[p]; i < eddMarkers[p + 1]; i++) {
            if (echoSum[i] > maxVals[p])
                maxVals[p] = echoSum[i];
        }
    }
    // Step 6: Add noise margin and quantize
    for (uint8_t i = 0; i < autoMax; i++) {
        uint16_t val = maxVals[i] + noiseMargin;
        if (val > 248) val = 248;
        maxVals[i] = (val + 7) / 8;
    }
    // Step 7: Select and write 16-byte threshold block
    uint8_t updateP1 = (cmd == PGA460_CMD_BURST_AND_LISTEN_PRESET1 || cmd == PGA460_CMD_LISTEN_ONLY_PRESET1);
    uint8_t *thrStart;
		if (updateP1)
				thrStart = (uint8_t *)&thr->P1_THR_0;
		else
				thrStart = (uint8_t *)&thr->P2_THR_0;

    for (uint8_t i = 0; i < 6; i++) {
        thrStart[i] = (windowIndex << 4) | windowIndex;
    }
    thrStart[6]  = (maxVals[0] << 3) | (maxVals[1] >> 2);
    thrStart[7]  = (maxVals[1] << 6) | (maxVals[2] << 1) | (maxVals[3] >> 4);
    thrStart[8]  = (maxVals[3] << 4) | (maxVals[4] >> 1);
    thrStart[9]  = (maxVals[4] << 7) | (maxVals[5] << 2) | (maxVals[6] >> 3);
    thrStart[10] = (maxVals[6] << 5) | maxVals[7];
    if (autoMax > 8)  thrStart[11] = maxVals[8];
    if (autoMax > 9)  thrStart[12] = maxVals[9];
    if (autoMax > 10) thrStart[13] = maxVals[10];
    if (autoMax > 11) thrStart[14] = maxVals[11];
    // Step 8: Compute offset (byte 15)
    int8_t offset = 0;
    for (uint8_t i = 0; i < autoMax; i++) {
        int8_t delta = ((int8_t)maxVals[i] * 8 + noiseMargin) - ((int8_t)maxVals[i] * 8);
        if (delta > offset) offset = delta;
    }
    if (offset > 7) offset = 7;
    thrStart[15] = offset & 0x0F;
    // Step 9: Capture and print AFTER echo
    if (PGA460_GetEchoDataDump(sensorID, cmd, echoBuf) != PGA460_ERR_NONE)
        return PGA460_ERR_GEN;
    DEBUG("Sensor %d: Echo Data Dump AFTER:\n", sensorID);
    for (uint8_t i = 0; i < 128; i++) {
        DEBUG("%d%s", echoBuf[i], (i < 127) ? "," : "\n");
    }
    // Step 10: Print threshold structure
    DEBUG("Sensor %d: Final Threshold Structure (%s):\n", sensorID, updateP1 ? "P1" : "P2");
    for (int i = 0; i < 16; i++) {
        DEBUG("%02X%s", thrStart[i], ((i + 1) % 16 == 0) ? "\n" : " ");
    }
    DEBUG("\nSensor %d: AutoThreshold completed on %s\n", sensorID, updateP1 ? "P1" : "P2");
    return PGA460_ERR_NONE;
}

PGA460_Error_t PGA460_GetUltrasonicMeasurement(const uint8_t sensorID) {
    uint8_t localBuffer[PGA_OBJ_DATA_SIZE] = {0};
    // Step 1: Send UMR command (0x07)
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Measurement Result Request Failed!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
    // Step 2: Receive result into buffer (2-byte header + N objects * 4 bytes)
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, localBuffer, sizeof(localBuffer), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Measurement Result Retrieval Failed!\n", sensorID);
        return PGA460_ERR_UART_RX;
    }
    // Step 3: Parse each object result
    PGA460_Error_t status = PGA460_ERR_GEN;
    for (uint8_t i = 0; i < PGA_OBJECTS_TRACKED; i++) {
        uint8_t base = 2 + (i * 4);  // Skip 2-byte diagnostic header
        uint16_t rawTOF = ((uint16_t)localBuffer[base] << 8) | localBuffer[base + 1];
        uint8_t width = localBuffer[base + 2];
        uint8_t amplitude = localBuffer[base + 3];
        DEBUG("Sensor %d: Raw TOF = %04X\n", sensorID, rawTOF);  // Add this line
        PGA460_MeasData_t *obj = &myUltraSonicArray[sensorID].objects[i];
        if (rawTOF > 0 && rawTOF != 0xFFFF) {
            obj->distance = ((float)rawTOF / 2.0f) * 1e-6f * soundSpeed;  // in meters
            obj->width = width * 16;   // width in microseconds
            obj->amplitude = amplitude;
            DEBUG("Sensor %d: Obj %d -> %.3f m, Width = %u us, Amplitude = %u\n", sensorID, i + 1, obj->distance, obj->width, obj->amplitude);
            status = PGA460_ERR_NONE;  // At least one valid result
        } else {
            obj->distance = 0.0f;
            obj->width = 0;
            obj->amplitude = 0;
            DEBUG("Sensor %d: Obj %d -> Invalid (TOF = 0x%04X)\n", sensorID, i + 1, rawTOF);
        }
    }
    return status;
}

PGA460_Error_t PGA460_GetEchoDataDump(uint8_t sensorID, uint8_t preset, uint8_t *echoOut) {
    uint8_t response[130] = {0};

    // Step 1: Enable Echo Data Dump Mode (EE_CTRL = 0x80)
    if (PGA460_RegisterWrite(sensorID, REG_EE_CTRL, ECHO_DATA_DUMP_ENABLE) != PGA460_ERR_NONE) {
        DEBUG("Sensor %d: Failed to enable Echo Data Dump mode!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
    HAL_Delay(10);  // Allow mode to activate
    // Step 2: Trigger Preset Burst+Listen or Listen-Only command
    if (PGA460_UltrasonicCmd(sensorID, (PGA460_Command_t)preset) != PGA460_ERR_NONE) {
        DEBUG("Sensor %d: Echo Trigger Preset Failed!\n", sensorID);
        return PGA460_ERR_GEN;
    }
    HAL_Delay(70);  // Allow for full echo capture (max 65ms + margin)
    // Step 3: Request Bulk Echo Data Dump (CMD 0x06)
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Echo Data Dump Request Failed!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
    // Step 4: Receive Echo Data Dump (2-byte header + 128 bins)
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, response, sizeof(response), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Echo Data Dump Receive Failed!\n", sensorID);
        return PGA460_ERR_GEN;
    }
    // Step 5: Disable Echo Data Dump Mode
    if (PGA460_RegisterWrite(sensorID, REG_EE_CTRL, ECHO_DATA_DUMP_DISABLE) != PGA460_ERR_NONE) {
        DEBUG("Sensor %d: Failed to disable Echo Data Dump mode!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
    // Step 6: Copy only the useful 128 echo bins (ignoring first 2 header bytes)
    memcpy(echoOut, &response[2], 128);
    return PGA460_ERR_NONE;
}

float PGA460_ReadTemperatureOrNoise(const uint8_t sensorID, const PGA460_CmdType_t mode) {
    const uint8_t isNoise = (mode == PGA460_CMD_GET_NOISE);
    float result = PGA460_TEMP_ERR;  // Default invalid return
    uint8_t buffer[4] = {0};
    // Step 1: Request Temperature or Noise Measurement
    buffer[0] = PGA460_SYNC;
    buffer[1] = PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT;
    buffer[2] = isNoise; // 0 = temperature, 1 = noise
    buffer[3] = PGA460_CalculateChecksum(&buffer[1], 2);
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, buffer, 4, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Temp/Noise Measurement Command Failed!\n", sensorID);
        return result;
    }
    HAL_Delay(10);  // Delay for measurement to complete
    // Step 2: Send command to read the result (fixed 3-byte command)
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_TEMP_AND_NOISE_RESULT].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Result Request Command Failed!\n", sensorID);
        return result;
    }
    // Step 3: Read 4-byte result frame
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, buffer, 4, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Failed to Receive Temp/Noise Data!\n", sensorID);
        return result;
    }
    // Step 4: Parse result
    if (!isNoise) {
        result = ((float)(buffer[1]) - 64.0f) / 1.5f;  // Temperature in °C
    } else {
        result = (float)(buffer[2]);  // Noise level (raw 8-bit)
    }
    DEBUG("Sensor %d: %s = %.2f\n", sensorID, isNoise ? "Noise Level" : "Temperature (C)", result);
    return result;
}

PGA460_Error_t PGA460_SetTemperatureOffset(uint8_t sensorID, float externalTempC) {
    float internalTempC = PGA460_ReadTemperatureOrNoise(sensorID, PGA460_CMD_GET_TEMP);
    int8_t offset = 0;
    uint8_t tempTrim = 0;
    // Step 1: Validate internal temperature reading
    if (internalTempC == PGA460_TEMP_ERR) {
        DEBUG("Sensor %d: Failed to read internal temperature!\n", sensorID);
        return PGA460_ERR_GEN;
    }
    DEBUG("Sensor %d: Internal Temperature = %.2f C\n", sensorID, internalTempC);
    // Step 2: Calculate signed offset between external and internal
    offset = (int8_t)roundf(externalTempC - internalTempC);
    if (offset > 63) offset = 63;
    if (offset < -64) offset = -64;
    // Step 3: Read current TEMP_TRIM register
    if (PGA460_RegisterRead(sensorID, REG_TEMP_TRIM, &tempTrim) != HAL_OK) {
        DEBUG("Sensor %d: Failed to read TEMP_TRIM register!\n", sensorID);
        return PGA460_ERR_UART_RX;
    }
    DEBUG("Sensor %d: Original TEMP_TRIM = 0x%02X\n", sensorID, tempTrim);
    // Step 4: Merge with sign-magnitude offset (bit 7 = sign, bits 6:0 = magnitude)
    uint8_t signedOffset = (offset < 0) ? ((~offset + 1) & 0x7F) | 0x80 : offset & 0x7F;
    tempTrim = (tempTrim & 0x80) | (signedOffset & 0x7F);  // Preserve MSB if needed
    // Step 5: Write updated TEMP_TRIM back
    if (PGA460_RegisterWrite(sensorID, REG_TEMP_TRIM, tempTrim) != HAL_OK) {
        DEBUG("Sensor %d: Failed to write TEMP_TRIM register!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
    DEBUG("Sensor %d: New TEMP_TRIM = 0x%02X (Offset: %+dC)\n", sensorID, tempTrim, offset);
    return PGA460_ERR_NONE;
}

PGA460_Error_t PGA460_GetSystemDiagnostics(const uint8_t sensorID, const uint8_t run, const uint8_t diag, float *diagResult) {
    uint8_t response[4] = {0};
    // Optional Step 1: Issue a Burst-and-Listen Command if Requested
    if (run) {
        if (PGA460_UltrasonicCmd(sensorID, PGA460_CMD_BURST_AND_LISTEN_PRESET1) != HAL_OK) {
            DEBUG("Sensor %d: Burst-and-Listen Command Failed!\n", sensorID);
            return PGA460_ERR_GEN;
        }
        HAL_Delay(100); // Wait for measurement to complete
    }
    // Step 2: Send System Diagnostics Command (0x14)
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_SYSTEM_DIAGNOSTICS].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: System Diagnostics Request Failed!\n", sensorID);
        return PGA460_ERR_UART_TX;
    }
    // Step 3: Receive 4-byte diagnostic response
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, response, 4, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: System Diagnostics Retrieval Failed!\n", sensorID);
        return PGA460_ERR_UART_RX;
    }
    // Step 4: Special Cases - Temperature / Noise handled separately
    if (diag == 2) {  // Temperature (°C)
        *diagResult = PGA460_ReadTemperatureOrNoise(sensorID, PGA460_CMD_GET_TEMP);
        if (*diagResult == PGA460_TEMP_ERR) {
            DEBUG("Sensor %d: Failed to Retrieve Temperature!\n", sensorID);
            return PGA460_ERR_GEN;
        }
        return PGA460_ERR_NONE;
    }
    if (diag == 3) {  // Noise level
        *diagResult = PGA460_ReadTemperatureOrNoise(sensorID, PGA460_CMD_GET_NOISE);
        if (*diagResult == PGA460_TEMP_ERR) {
            DEBUG("Sensor %d: Failed to Retrieve Noise!\n", sensorID);
            return PGA460_ERR_GEN;
        }
        return PGA460_ERR_NONE;
    }
    // Step 5: Standard diagnostics - Frequency or Decay
    switch (diag) {
        case 0: // Frequency (kHz)
            if (response[1] == 0) {
                DEBUG("Sensor %d: Invalid Frequency Value!\n", sensorID);
                return PGA460_ERR_GEN;
            }
            *diagResult = (1.0 / (response[1] * 0.5e-6)) / 1000.0f; // Convert to kHz
            break;
        case 1: // Decay period (µs)
            *diagResult = response[2] * 16.0f;
            break;
        default:
            DEBUG("Sensor %d: Invalid Diagnostic Type (%d)!\n", sensorID, diag);
            return PGA460_ERR_GEN;
    }

    DEBUG("Sensor %d: Diagnostic [%d] Result: %.2f\n", sensorID, diag, *diagResult);
    return PGA460_ERR_NONE;
}

uint8_t getBandpassFilterBandwidth(const uint8_t sensorID) {
    // Bandwidth = 2 * (BPF_BW + 1) [kHz]
    return 2 * (myUltraSonicArray[sensorID].PGA460_Data.EEPROM.INIT_GAIN.Val.BitField.BPF_BW + 1);
}

void setInitialGain(const uint8_t sensorID, uint8_t gain_dB) {
    // AFE_GAIN_RNG maps to min gain = 32 + 6*n
    uint8_t afeMinGain_dB = myUltraSonicArray[sensorID].PGA460_Data.EEPROM.DECPL_TEMP.Val.BitField.AFE_GAIN_RNG * 6 + 32;
    // Reverse the formula: GAIN_INIT = 2 * (gain_dB - afeMinGain_dB)
    int gainVal = 2 * (gain_dB - afeMinGain_dB);
    // Clamp to 6-bit field range
    if (gainVal < 0) gainVal = 0;
    if (gainVal > 63) gainVal = 63;
    // Update the structure
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.INIT_GAIN.Val.BitField.GAIN_INIT = gainVal;
    // Write to the actual register (0x1B)
    PGA460_RegisterWrite(sensorID, REG_INIT_GAIN, myUltraSonicArray[sensorID].PGA460_Data.EEPROM.INIT_GAIN.Val.Value);
}

uint8_t getInitialGain(const uint8_t sensorID) {
    float gain_dB = 0.5f * (myUltraSonicArray[sensorID].PGA460_Data.EEPROM.INIT_GAIN.Val.BitField.GAIN_INIT + 1) + (myUltraSonicArray[sensorID].PGA460_Data.EEPROM.DECPL_TEMP.Val.BitField.AFE_GAIN_RNG * 6) + 32;
    return (uint8_t)(gain_dB + 0.5f);  // Rounded to nearest integer
}

void setFrequency(const uint8_t sensorID, uint8_t frequency_kHz) {
    uint8_t regVal = (uint8_t)(((float)frequency_kHz - 30.0f) / 0.2f + 0.5f);  // Round to nearest
    if (regVal > 250) regVal = 250;  // Clamp to max allowed
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.FREQ = regVal;
    // Write immediately to the register if runtime update is needed
    PGA460_RegisterWrite(sensorID, REG_FREQUENCY, regVal);
}

uint8_t getFrequency(const uint8_t sensorID) {
    uint8_t regVal = myUltraSonicArray[sensorID].PGA460_Data.EEPROM.FREQ;
    return (uint8_t)(regVal * 0.2f + 30.0f + 0.5f);  // Round to nearest
}

void setDeglitchPeriod(const uint8_t sensorID, uint8_t deglitch_us) {
    uint8_t value = deglitch_us / 8;
    if (value > 7) value = 7;  // Clamp to 3-bit field range
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.DEADTIME.Val.BitField.THR_CMP_DEGLTCH = value;
    // Write immediately to hardware
    PGA460_RegisterWrite(sensorID, REG_DEADTIME, myUltraSonicArray[sensorID].PGA460_Data.EEPROM.DEADTIME.Val.Value);
}

uint8_t getDeglitchPeriod(const uint8_t sensorID) {
    return myUltraSonicArray[sensorID].PGA460_Data.EEPROM.DEADTIME.Val.BitField.THR_CMP_DEGLTCH * 8;  // Convert from register scale to microseconds
}

void setBurstPulseDeadTime(const uint8_t sensorID, uint8_t burstPulseDeadTime_us) {
    uint8_t value = (uint8_t)(burstPulseDeadTime_us / 0.0625f);
    if (value > 255) value = 255;
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.DEADTIME.Val.BitField.PULSE_DT = value;
    // Write immediately to DEADTIME register
    PGA460_RegisterWrite(sensorID, REG_DEADTIME, myUltraSonicArray[sensorID].PGA460_Data.EEPROM.DEADTIME.Val.Value);
}

uint8_t getBurstPulseDeadTime(const uint8_t sensorID) {
    return (uint8_t)(myUltraSonicArray[sensorID].PGA460_Data.EEPROM.DEADTIME.Val.BitField.PULSE_DT * 0.0625f);
}

void setBurstPulseP1(const uint8_t sensorID, uint8_t burstPulseP1) {
    if (burstPulseP1 > 31) burstPulseP1 = 31;
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.PULSE_P1.Val.BitField.P1_PULSE = burstPulseP1;
    // Write to PULSE_P1 register (0x1E)
    PGA460_RegisterWrite(sensorID, REG_PULSE_P1, myUltraSonicArray[sensorID].PGA460_Data.EEPROM.PULSE_P1.Val.Value);
}

void setBurstPulseP2(const uint8_t sensorID, uint8_t burstPulseP2) {
    if (burstPulseP2 > 31) burstPulseP2 = 31;
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.PULSE_P2.Val.BitField.P2_PULSE = burstPulseP2;
    // Write to PULSE_P2 register (0x1F)
    PGA460_RegisterWrite(sensorID, REG_PULSE_P2, myUltraSonicArray[sensorID].PGA460_Data.EEPROM.PULSE_P2.Val.Value);
}

void setAddress(const uint8_t sensorID, uint8_t address) {
    if (address > 7) address = 7;
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.PULSE_P2.Val.BitField.UART_ADDR = address;
    // Write to PULSE_P2 register (0x1F)
    PGA460_RegisterWrite(sensorID, REG_PULSE_P2, myUltraSonicArray[sensorID].PGA460_Data.EEPROM.PULSE_P2.Val.Value);
}

//ToDo add Lim for P2?
void setCurrentLimitStatus(const uint8_t sensorID, uint8_t currentLimitStatus) {
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.CURR_LIM_P1.Val.BitField.DIS_CL = currentLimitStatus ? 1 : 0;
    // Writes to CURR_LIM_P1 (0x20)
    PGA460_RegisterWrite(sensorID, REG_CURR_LIM_P1, myUltraSonicArray[sensorID].PGA460_Data.EEPROM.CURR_LIM_P1.Val.Value);
}

void setSleepModeTimer(const uint8_t sensorID, uint8_t sleepModeTimer) {
    if (sleepModeTimer > 3) sleepModeTimer = 3;
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.FVOLT_DEC.Val.BitField.LPM_TMR = (LPM_TMR_t)sleepModeTimer;
    // Write updated value to register 0x25
    PGA460_RegisterWrite(sensorID, REG_FVOLT_DEC, myUltraSonicArray[sensorID].PGA460_Data.EEPROM.FVOLT_DEC.Val.Value);
}

void setDecoupleTimeOrTemperature(const uint8_t sensorID, uint8_t decoupleValue) {
    if (decoupleValue > 15) decoupleValue = 15;
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.DECPL_TEMP.Val.BitField.DECPL_T = decoupleValue;
    // Write updated value to register 0x26
    PGA460_RegisterWrite(sensorID, REG_DECPL_TEMP, myUltraSonicArray[sensorID].PGA460_Data.EEPROM.DECPL_TEMP.Val.Value);
}

void setNoiseLevel(const uint8_t sensorID, uint8_t noiseLevel) {
    if (noiseLevel > 31) noiseLevel = 31;
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.DSP_SCALE.Val.BitField.NOISE_LVL = noiseLevel;
    // Write to DSP_SCALE register (0x27)
    PGA460_RegisterWrite(sensorID, REG_DSP_SCALE, myUltraSonicArray[sensorID].PGA460_Data.EEPROM.DSP_SCALE.Val.Value);
}

void setTemperatureScaleOffset(const uint8_t sensorID, int8_t tempOffset) {
    if (tempOffset < -8) tempOffset = -8;
    else if (tempOffset > 7) tempOffset = 7;
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.TEMP_TRIM.Val.BitField.TEMP_OFF = (uint8_t)(tempOffset & 0x0F);
    // Write to TEMP_TRIM register (0x28)
    PGA460_RegisterWrite(sensorID, REG_TEMP_TRIM, myUltraSonicArray[sensorID].PGA460_Data.EEPROM.TEMP_TRIM.Val.Value);
}

int8_t getTemperatureScaleOffset(const uint8_t sensorID) {
    uint8_t raw = myUltraSonicArray[sensorID].PGA460_Data.EEPROM.TEMP_TRIM.Val.BitField.TEMP_OFF;
    return (raw & 0x08) ? (raw | 0xF0) : raw;  // Sign-extend from 4-bit signed value
}

void setTemperatureScaleGain(const uint8_t sensorID, int8_t tempGain) {
    if (tempGain < -8) tempGain = -8;
    else if (tempGain > 7) tempGain = 7;
    myUltraSonicArray[sensorID].PGA460_Data.EEPROM.TEMP_TRIM.Val.BitField.TEMP_GAIN = (uint8_t)(tempGain & 0x0F);
    // Write to TEMP_TRIM register (0x28)
    PGA460_RegisterWrite(sensorID, REG_TEMP_TRIM, myUltraSonicArray[sensorID].PGA460_Data.EEPROM.TEMP_TRIM.Val.Value);
}

// Get Temperature Scale Gain
int8_t getTemperatureScaleGain(const uint8_t sensorID) {
    uint8_t raw = myUltraSonicArray[sensorID].PGA460_Data.EEPROM.TEMP_TRIM.Val.BitField.TEMP_GAIN;
    return (raw & 0x08) ? (raw | 0xF0) : raw;  // Sign-extend 4-bit signed value
}
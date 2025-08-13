#include <string.h>
#include <stdio.h>
#include <math.h>
#include "usart.h"
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

//static void PGA460_PrintDiagnostic(PGA460_Diag_t diag) {
//    if ((diag.Value & 0xC0) != 0x40) {
//        DEBUG("Invalid Diagnostic Marker (expected 0x40-0x7F, got 0x%02X)\n", diag.Value);
//        return;
//    }
//    if (diag.BitField.DeviceBusy)        DEBUG("PGA460: Device was busy\n");
//    if (diag.BitField.SyncRateError)     DEBUG("PGA460: Sync rate too high/low\n");
//    if (diag.BitField.SyncWidthMismatch) DEBUG("PGA460: Sync field width mismatch\n");
//    if (diag.BitField.ChecksumMismatch)  DEBUG("PGA460: Checksum mismatch (controller vs device)\n");
//    if (diag.BitField.InvalidCommand)    DEBUG("PGA460: Invalid command sent\n");
//    if (diag.BitField.UARTFrameError)    DEBUG("PGA460: UART frame/stop bit/contention error\n");
//}

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

// Function to initialize 3x PGA460 sensors
HAL_StatusTypeDef PGA460_Init(void) {
	
		PGA460_Sensor_t sensors[ULTRASONIC_SENSOR_COUNT] = {
				{ .uartPort = &huart1, .Registers = s1 },
				{ .uartPort = &huart4, .Registers = s2 },
				{ .uartPort = &huart5, .Registers = s3 }
		};

    for (uint8_t i = 0; i < ULTRASONIC_SENSOR_COUNT; i++) {
        // Step 1: Assign UART and transducer config
        HAL_UART_Init(myUltraSonicArray[i].uartPort);
        HAL_Delay(100);

    }
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_RegisterRead(uint8_t sensorID, uint8_t regAddr, uint8_t *regValue) {
    // --- TX frame: [SYNC][CMD][REG_ADDR][CHK] ---
    uint8_t tx[4];
    tx[0] = PGA460_SYNC;                           // 0x55
    tx[1] = PGA460_CMD_REGISTER_READ;              // Command code
    tx[2] = regAddr;                               // Register address
    tx[3] = PGA460_CalculateChecksum(&tx[1], 2);   // Checksum over CMD + REG_ADDR
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, tx, sizeof(tx), UART_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    // --- RX frame: [DIAG][DATA][CHK] ---
    uint8_t rx[3] = {0};
    if (HAL_UART_Receive(sensors[sensorID].uartPort, rx, sizeof(rx), UART_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    // Validate checksum over DIAG + DATA
//    if (PGA460_CalculateChecksum(rx, 2) != rx[2]) {
//        return HAL_ERROR;
//    }
    *regValue = rx[1]; // DATA byte
    return HAL_OK;
}

// Function to write a register on the PGA460
HAL_StatusTypeDef PGA460_RegisterWrite(uint8_t sensorID, uint8_t regAddr, uint8_t regValue) {
    // TX frame: [SYNC][CMD][REG_ADDR][REG_VALUE][CHK]
    uint8_t tx[5];
    tx[0] = PGA460_SYNC;                       // 0x55
    tx[1] = PGA460_CMD_REGISTER_WRITE;         // command
    tx[2] = regAddr;                           // register address
    tx[3] = regValue;                          // value to write
    tx[4] = PGA460_CalculateChecksum(&tx[1], 3); // checksum over CMD+ADDR+DATA
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, tx, sizeof(tx), UART_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    // Small settle time (keep if you observed the device needing it)
    HAL_Delay(10);
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_CheckStatus(uint8_t sensorID) {
    // Read both DEV_STAT registers
    if (PGA460_RegisterRead(sensorID, REG_DEV_STAT0, &sensors[sensorID].Registers.Stat0.Val.Value) != HAL_OK ||
        PGA460_RegisterRead(sensorID, REG_DEV_STAT1, &sensors[sensorID].Registers.Stat1.Val.Value) != HAL_OK) {
        DEBUG("Sensor %d: Communication error! Could not read status registers.\n", (int)sensorID);
        return HAL_ERROR;
    }
    // --- DEV_STAT0 Flags ---
    if (sensors[sensorID].Registers.Stat0.Val.BitField.CMW_WU_ERR)
        DEBUG("Sensor %d: [CMW_WU_ERR] Wake-up error!\n", (int)sensorID);
    if (sensors[sensorID].Registers.Stat0.Val.BitField.THR_CRC_ERR)
        DEBUG("Sensor %d: [THR_CRC_ERR] Threshold CRC error!\n", (int)sensorID);
    if (sensors[sensorID].Registers.Stat0.Val.BitField.EE_CRC_ERR)
        DEBUG("Sensor %d: [EE_CRC_ERR] EEPROM CRC error!\n", (int)sensorID);
    if (sensors[sensorID].Registers.Stat0.Val.BitField.TRIM_CRC_ERR)
        DEBUG("Sensor %d: [TRIM_CRC_ERR] Trim CRC error!\n", (int)sensorID);

    // --- DEV_STAT1 Flags ---
    if (sensors[sensorID].Registers.Stat1.Val.BitField.TSD_PROT)
        DEBUG("Sensor %d: [TSD_PROT] Thermal shutdown occurred!\n", (int)sensorID);
    if (sensors[sensorID].Registers.Stat1.Val.BitField.IOREG_OV)
        DEBUG("Sensor %d: [IOREG_OV] IOREG Overvoltage!\n", (int)sensorID);
    if (sensors[sensorID].Registers.Stat1.Val.BitField.IOREG_UV)
        DEBUG("Sensor %d: [IOREG_UV] IOREG Undervoltage!\n", (int)sensorID);
    if (sensors[sensorID].Registers.Stat1.Val.BitField.AVDD_OV)
        DEBUG("Sensor %d: [AVDD_OV] AVDD Overvoltage!\n", (int)sensorID);
    if (sensors[sensorID].Registers.Stat1.Val.BitField.AVDD_UV)
        DEBUG("Sensor %d: [AVDD_UV] AVDD Undervoltage!\n", (int)sensorID);
    if (sensors[sensorID].Registers.Stat1.Val.BitField.VPWR_OV)
        DEBUG("Sensor %d: [VPWR_OV] VPWR Overvoltage!\n", (int)sensorID);
    if (sensors[sensorID].Registers.Stat1.Val.BitField.VPWR_UV)
        DEBUG("Sensor %d: [VPWR_UV] VPWR Undervoltage!\n", (int)sensorID);
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_EEPROMBulkRead(uint8_t sensorID) {
    // --- Step 1: Send EEPROM Bulk Read command ---
    // TX: [SYNC][CMD][CHK]  (checksum over CMD only)
    uint8_t tx[3];
    tx[0] = PGA460_SYNC;                       // 0x55
    tx[1] = PGA460_CMD_EEPROM_BULK_READ;       // 0x0B
    tx[2] = PGA460_CalculateChecksum(&tx[1], 1);
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, tx, sizeof(tx), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Bulk Read Command Failed!\n", (int)sensorID);
        return HAL_ERROR;
    }
    HAL_Delay(10); // device needs a short time to prepare reply
    // --- Step 2: Receive response ---
    // RX: [DIAG][EEPROM bytes 0x00..0x2B] = 1 + 44 = 45 bytes
    uint8_t rx[45] = {0};
    if (HAL_UART_Receive(sensors[sensorID].uartPort, rx, sizeof(rx), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Bulk Read Failed!\n", (int)sensorID);
        return HAL_ERROR;
    }
    // Optional: Detect idle flood (all 0xFF)
    uint8_t idle = 1u;
    for (uint32_t i = 0; i < sizeof(rx); i++) {
        if (rx[i] != 0xFF) { idle = 0u; break; }
    }
    if (idle) {
        DEBUG("Sensor %d: EEPROM Read response was idle (0xFF)\n", (int)sensorID);
        return HAL_ERROR;
    }
    // --- Step 3: Copy EEPROM content (skip rx[0] = DIAG) ---
    // Map 44 bytes [0x00..0x2B] into structured mirror
    memcpy(&sensors[sensorID].Registers.EEData, &rx[1], sizeof(EEImage_t));
    // --- Step 4: Validate EE_CRC (byte index 43 within the 44-byte EEPROM image) ---
    const uint8_t *ee = (const uint8_t *)&sensors[sensorID].Registers.EEData;
    uint8_t expected_crc = PGA460_CalculateEEPROM_CRC(ee, 43); // bytes 0..42
    if (ee[43] != expected_crc) {
        DEBUG("Sensor %d: EEPROM CRC mismatch! Expected 0x%02X, Got 0x%02X\n",
              (int)sensorID, expected_crc, ee[43]);
        return HAL_ERROR;
    }
    DEBUG("Sensor %d: EEPROM Bulk Read Successful.\n", (int)sensorID);
    return HAL_OK;
}

// EEPROM bulk write: send 44-byte EEPROM image (0x00..0x2B) to device shadow
HAL_StatusTypeDef PGA460_EEPROMBulkWrite(uint8_t sensorID) {
    // Pointer to 44-byte EEPROM image in our mirror
    uint8_t *ee = (uint8_t *)&sensors[sensorID].Registers.EEData;
    // Update EE_CRC (covers bytes 0..42; byte 43 is EE_CRC)
    sensors[sensorID].Registers.EEData.Sett.EE_CRC = PGA460_CalculateEEPROM_CRC(ee, 43);
    // Build TX frame: [SYNC][CMD=EEPROM_BULK_WRITE][44 bytes][CHK]
    uint8_t frame[47];
    frame[0] = PGA460_SYNC;                    // 0x55
    frame[1] = PGA460_CMD_EEPROM_BULK_WRITE;   // 0x0C
    memcpy(&frame[2], ee, 44);                 // copy 0x00..0x2B
    frame[46] = PGA460_CalculateChecksum(&frame[1], 45); // checksum over CMD + 44 bytes
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, frame, sizeof(frame), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Bulk Write Failed!\n", (int)sensorID);
        return HAL_ERROR;
    }
    // Give the device a moment to process the shadow write (not the burn)
    HAL_Delay(100);
    // Optional: check status
    (void)PGA460_CheckStatus(sensorID);
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_BurnEEPROM(uint8_t sensorID) {
    // Step 1: Unlock EEPROM (EE_UNLCK=0xD, EE_PRGM=0) => 0x68
    if (PGA460_RegisterWrite(sensorID, REG_EE_CTRL, PGA460_UNLOCK_EEPROM) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Burn Step 1 (0x68) Failed!\n", (int)sensorID);
        return HAL_ERROR;
    }
    HAL_Delay(1); // short delay per datasheet
    // Step 2: Trigger EEPROM burn (EE_UNLCK=0xD, EE_PRGM=1) => 0x69
    if (PGA460_RegisterWrite(sensorID, REG_EE_CTRL, PGA460_LOCK_EEPROM) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Burn Step 2 (0x69) Failed!\n", (int)sensorID);
        return HAL_ERROR;
    }
    // Burn time: ~600–1000 ms per datasheet
    HAL_Delay(1000);
    // Step 3: Read back EE_CTRL to verify
    uint8_t ctrlVal = 0;
    if (PGA460_RegisterRead(sensorID, REG_EE_CTRL, &ctrlVal) != HAL_OK) {
        DEBUG("Sensor %d: Failed to read EE_CTRL after burn!\n", (int)sensorID);
        return HAL_ERROR;
    }
    sensors[sensorID].Registers.eeCtrl.Val.Value = ctrlVal;
    // Step 4: Check EE_PRGM_OK (bit 2)
    if (sensors[sensorID].Registers.eeCtrl.Val.BitField.EE_PRGM_OK) {
        DEBUG("Sensor %d: EEPROM Burn Successful (EE_CTRL = 0x%02X)\n", (int)sensorID, ctrlVal);
        // Optional: re-lock by clearing unlock nibble if you wish:
        // (void)PGA460_RegisterWrite(sensorID, REG_EE_CTRL, 0x00);
        return HAL_OK;
    } else {
        DEBUG("Sensor %d: EEPROM Burn Failed (EE_CTRL = 0x%02X)\n", (int)sensorID, ctrlVal);
        return HAL_ERROR;
    }
}

// TVG bulk read: fetch TVGAIN0..TVGAIN6 into sensors[sid].Registers.EEData.TVG
HAL_StatusTypeDef PGA460_GetTVG(uint8_t sensorID) {
    // --- Step 1: Send TVG Bulk Read command ---
    // TX: [SYNC][CMD=0x0D][CHK]  (checksum over CMD only)
    uint8_t tx[3];
    tx[0] = PGA460_SYNC;                     // 0x55
    tx[1] = PGA460_CMD_TVG_BULK_READ;        // 0x0D
    tx[2] = PGA460_CalculateChecksum(&tx[1], 1);
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, tx, sizeof(tx), UART_TIMEOUT) != HAL_OK) {
        // DEBUG("Sensor %d: TVG Bulk Read command TX failed!\n", (int)sensorID);
        return HAL_ERROR;
    }
    // --- Step 2: Receive DIAG + 7 TVG bytes ---
    // RX: [DIAG][TVGAIN0..TVGAIN6] => 1 + 7 = 8 bytes
    uint8_t rx[8] = {0};
    if (HAL_UART_Receive(sensors[sensorID].uartPort, rx, sizeof(rx), UART_TIMEOUT) != HAL_OK) {
        // DEBUG("Sensor %d: TVG Bulk Read RX failed!\n", (int)sensorID);
        return HAL_ERROR;
    }
    // Optional: detect idle flood (all 0xFF)
    uint8_t idle = 1u;
    for (uint32_t i = 0; i < sizeof(rx); i++) {
        if (rx[i] != 0xFF) { idle = 0u; break; }
    }
    if (idle) return HAL_ERROR;
    // --- Step 3: Copy TVG bytes into mirror (skip rx[0] = DIAG) ---
    memcpy(&sensors[sensorID].Registers.EEData.TVG, &rx[1], sizeof(TVG_t));
    return HAL_OK;
}

// Write TVG (TVGAIN0..TVGAIN6) via bulk write
HAL_StatusTypeDef PGA460_SetTVG(uint8_t sensorID, const TVG_t *tvg) {
    // Build frame: [SYNC][CMD=0x0E][7 TVG bytes][CHK]
    uint8_t frame[10];
    frame[0] = PGA460_SYNC;                   // 0x55
    frame[1] = PGA460_CMD_TVG_BULK_WRITE;     // 0x0E
    memcpy(&frame[2], tvg, sizeof(TVG_t));    // 7 bytes payload
    frame[9] = PGA460_CalculateChecksum(&frame[1], 1 + sizeof(TVG_t)); // over CMD+DATA
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, frame, sizeof(frame), UART_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    // mirror locally on success
    memcpy(&sensors[sensorID].Registers.EEData.TVG, tvg, sizeof(TVG_t));
    HAL_Delay(70); // allow device to apply TVG
    return HAL_OK;
}
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "PGA460_REG.h"
#include "Transducers.h"
#include "PGA460.h"
#include "debug.h"

// Initialize the array of ultrasonic sensors
PGA460_Sensor_t myUltraSonicArray[ULTRASONIC_SENSOR_COUNT];

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;

const CommandArray commands[PGA_CMD_COUNT] = {
    {{PGA460_SYNC, PGA460_CMD_BURST_AND_LISTEN_PRESET1, 255 - PGA460_CMD_BURST_AND_LISTEN_PRESET1}}, // PGA460_CMD_BURST_AND_LISTEN_PRESET1
    {{PGA460_SYNC, PGA460_CMD_BURST_AND_LISTEN_PRESET2, 255 - PGA460_CMD_BURST_AND_LISTEN_PRESET2}}, // PGA460_CMD_BURST_AND_LISTEN_PRESET2
    {{PGA460_SYNC, PGA460_CMD_LISTEN_ONLY_PRESET1, 255 - PGA460_CMD_LISTEN_ONLY_PRESET1}}, // PGA460_CMD_LISTEN_ONLY_PRESET1
    {{PGA460_SYNC, PGA460_CMD_LISTEN_ONLY_PRESET2, 255 - PGA460_CMD_LISTEN_ONLY_PRESET2}}, // PGA460_CMD_LISTEN_ONLY_PRESET2
    {{PGA460_SYNC, PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT, 255 - PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT}}, // PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT
    {{PGA460_SYNC, PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT, 255 - PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT}}, // PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT
    {{PGA460_SYNC, PGA460_CMD_TEMP_AND_NOISE_RESULT, 255 - PGA460_CMD_TEMP_AND_NOISE_RESULT}}, // PGA460_CMD_TEMP_AND_NOISE_RESULT
    {{PGA460_SYNC, PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP, 255 - PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP}}, // PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP
    {{PGA460_SYNC, PGA460_CMD_SYSTEM_DIAGNOSTICS, 255 - PGA460_CMD_SYSTEM_DIAGNOSTICS}}, // PGA460_CMD_SYSTEM_DIAGNOSTICS
    {{PGA460_SYNC, PGA460_CMD_REGISTER_READ, 255 - PGA460_CMD_REGISTER_READ}}, // PGA460_CMD_REGISTER_READ
    {{PGA460_SYNC, PGA460_CMD_REGISTER_WRITE, 255 - PGA460_CMD_REGISTER_WRITE}}, // PGA460_CMD_REGISTER_WRITE
    {{PGA460_SYNC, PGA460_CMD_EEPROM_BULK_READ, 255 - PGA460_CMD_EEPROM_BULK_READ}}, // PGA460_CMD_EEPROM_BULK_READ
    {{PGA460_SYNC, PGA460_CMD_EEPROM_BULK_WRITE, 255 - PGA460_CMD_EEPROM_BULK_WRITE}}, // PGA460_CMD_EEPROM_BULK_WRITE
    {{PGA460_SYNC, PGA460_CMD_TVG_BULK_READ, 255 - PGA460_CMD_TVG_BULK_READ}}, // PGA460_CMD_TVG_BULK_READ
    {{PGA460_SYNC, PGA460_CMD_TVG_BULK_WRITE, 255 - PGA460_CMD_TVG_BULK_WRITE}}, // PGA460_CMD_TVG_BULK_WRITE
    {{PGA460_SYNC, PGA460_CMD_THRESHOLD_BULK_READ, 255 - PGA460_CMD_THRESHOLD_BULK_READ}}, // PGA460_CMD_THRESHOLD_BULK_READ
    {{PGA460_SYNC, PGA460_CMD_THRESHOLD_BULK_WRITE, 255 - PGA460_CMD_THRESHOLD_BULK_WRITE}}, // PGA460_CMD_THRESHOLD_BULK_WRITE
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1, 255 - PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1}}, // PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2, 255 - PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2}}, // PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_LISTEN_ONLY_P1, 255 - PGA460_CMD_BROADCAST_LISTEN_ONLY_P1}}, // PGA460_CMD_BROADCAST_LISTEN_ONLY_P1
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_LISTEN_ONLY_P2, 255 - PGA460_CMD_BROADCAST_LISTEN_ONLY_P2}}, // PGA460_CMD_BROADCAST_LISTEN_ONLY_P2
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_TEMP_NOISE_MEASURE, 255 - PGA460_CMD_BROADCAST_TEMP_NOISE_MEASURE}}, // PGA460_CMD_BROADCAST_TEMP_NOISE_MEASURE
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_REGISTER_WRITE, 255 - PGA460_CMD_BROADCAST_REGISTER_WRITE}}, // PGA460_CMD_BROADCAST_REGISTER_WRITE
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_EEPROM_BULK_WRITE, 255 - PGA460_CMD_BROADCAST_EEPROM_BULK_WRITE}}, // PGA460_CMD_BROADCAST_EEPROM_BULK_WRITE
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_TVG_BULK_WRITE, 255 - PGA460_CMD_BROADCAST_TVG_BULK_WRITE}}, // PGA460_CMD_BROADCAST_TVG_BULK_WRITE
    {{PGA460_SYNC, PGA460_CMD_BROADCAST_THRESHOLD_BULK_WRITE, 255 - PGA460_CMD_BROADCAST_THRESHOLD_BULK_WRITE}}  // PGA460_CMD_BROADCAST_THRESHOLD_BULK_WRITE
};

// Helper function to calculate checksum for a data frame
static uint8_t PGA460_CalculateChecksum(const uint8_t *data, uint8_t len) {
    uint16_t carry = 0;
    for (uint8_t i = 0; i < len; i++) {
        carry += data[i];
        if (carry > 0xFF) {
            carry -= 255;
        }
    }
    carry = ~carry & 0xFF;
    return (uint8_t)carry;
}

void PGA460_ReadAllSensors(void) {
    for (uint8_t sensorID = 0; sensorID < ULTRASONIC_SENSOR_COUNT; sensorID++) {
        if (PGA460_UltrasonicCmd(sensorID, PGA460_CMD_BURST_AND_LISTEN_PRESET1, PGA_OBJECTS_TRACKED) != HAL_OK) {
            DEBUG("Sensor %d: Burst-and-Listen Command Failed!\n", sensorID);
            continue;
        }

        if (PGA460_GetUltrasonicMeasurement(sensorID) != HAL_OK) {
            DEBUG("Sensor %d: Measurement Retrieval Failed!\n", sensorID);
            continue;
        }

        for (uint8_t obj = 0; obj < PGA_OBJECTS_TRACKED; obj++) {
            DEBUG("Sensor %d, Obj %d: Distance = %d m, Width = %u us, Amplitude = %u\n",
                  sensorID, obj + 1,
                  myUltraSonicArray[sensorID].ultrasonicData.objects[obj].distance,
                  myUltraSonicArray[sensorID].ultrasonicData.objects[obj].width,
                  myUltraSonicArray[sensorID].ultrasonicData.objects[obj].amplitude);
        }

        HAL_Delay(100);
    }
}

// Function to initialize 3x PGA460 sensors
HAL_StatusTypeDef PGA460_Init(void) {
    UART_HandleTypeDef *uartPorts[ULTRASONIC_SENSOR_COUNT] = { &huart1, &huart4, &huart5 };

    for (uint8_t i = 0; i < ULTRASONIC_SENSOR_COUNT; i++) {
        myUltraSonicArray[i].PGA460_Data = transducer;
        memcpy(&myUltraSonicArray[i].PGA460_Data.P1_THR_0, &THRESHOLD_50[0], 32); // Copy threshold settings
        myUltraSonicArray[i].uartPort = uartPorts[i];

        // **Step 1: Check Sensor Status**
        if (PGA460_CheckStatus(i) != HAL_OK) {
            DEBUG("Sensor %d: Sensor check failed. Skipping initialization.\n", i);
            return HAL_ERROR;
        }

        // **Step 2: Read EEPROM Control Register**
        PGA460_RegisterRead(i, REG_EE_CTRL, &myUltraSonicArray[i].PGA460_Data.EE_CNTRL.Val.Value);
        DEBUG("Sensor %d: EEPROM Control Register (EE_CTRL) = 0x%02X\n", i, myUltraSonicArray[i].PGA460_Data.EE_CNTRL.Val.Value);

        // **Step 3: Run System Diagnostics**
        float diagValue = 0.0;

        if (PGA460_GetSystemDiagnostics(i, 1, 0, &diagValue) == HAL_OK) {
            DEBUG("Sensor %d: Frequency Diagnostic = %.2f kHz\n", i, diagValue);
        } else {
            DEBUG("Sensor %d: Frequency Diagnostic Failed!\n", i);
        }

        if (PGA460_GetSystemDiagnostics(i, 1, 1, &diagValue) == HAL_OK) {
            DEBUG("Sensor %d: Decay Period Diagnostic = %.2f us\n", i, diagValue);
        } else {
            DEBUG("Sensor %d: Decay Period Diagnostic Failed!\n", i);
        }

        // **Step 3b: Read Temperature & Noise Level**
        float temperature = PGA460_ReadTemperatureOrNoise(i, PGA460_CMD_GET_TEMP);
        if (temperature != PGA460_TEMP_ERR) {
            DEBUG("Sensor %d: Die Temperature = %.2f C\n", i, temperature);
        } else {
            DEBUG("Sensor %d: Temperature Read Failed!\n", i);
        }

        float noiseLevel = PGA460_ReadTemperatureOrNoise(i, PGA460_CMD_GET_NOISE);
        if (noiseLevel != PGA460_TEMP_ERR) {
            DEBUG("Sensor %d: Noise Level = %.0f (8-bit raw)\n", i, noiseLevel);
        } else {
            DEBUG("Sensor %d: Noise Level Read Failed!\n", i);
        }

        // **Step 4: Bulk Write EEPROM (Transducer Settings)**
        if (PGA460_EEPROMBulkWrite(i) != HAL_OK) {
            DEBUG("Sensor %d: EEPROM Bulk Write failed.\n", i);
            return HAL_ERROR;
        }

        // **Step 5: Configure Time-Varying Gain (TVG)**
        if (PGA460_InitTimeVaryingGain(i, PGA460_GAIN_52_84dB, PGA460_TVG_50_PERCENT) != HAL_OK) {
            DEBUG("Sensor %d: TVG Bulk Write failed.\n", i);
            return HAL_ERROR;
        }

        // **Step 6: Optional - Burn EEPROM (If Settings Need to Persist)**
//        if (PGA460_BurnEEPROM(i) != HAL_OK) {
//            DEBUG("Sensor %d: BurnEEPROM failed.\n", i);
//            return HAL_ERROR;
//        }

        // **Step 7: Optional - Retrieve Echo Data Dump for Debugging**
//        uint8_t echoDump[128];
//        if (PGA460_GetEchoDataDump(i, echoDump) != HAL_OK) {
//            DEBUG("Sensor %d: Echo Data Dump failed!\n", i);
//        }
    }

    DEBUG("PGA460 Initialization Complete.\n");
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_CheckStatus(const uint8_t sensorID) {
    // Read both status registers (0x4C and 0x4D) and store directly in myUltraSonicArray
    if (PGA460_RegisterRead(sensorID, REG_DEV_STAT0, (uint8_t *)&myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT0) == HAL_OK && 
        PGA460_RegisterRead(sensorID, REG_DEV_STAT1, (uint8_t *)&myUltraSonicArray[sensorID].PGA460_Data.DEV_STAT1) == HAL_OK) {
        return HAL_OK;
    }
    DEBUG("Sensor %d: Communication error! Could not read status registers.\n", (int)sensorID);
    return HAL_ERROR;
}

// Function to read a register from the PGA460
HAL_StatusTypeDef PGA460_RegisterRead(const uint8_t sensorID, const uint8_t regAddr, uint8_t *regValue) {
    uint8_t frame[4] = {PGA460_SYNC, PGA460_CMD_REGISTER_READ, regAddr, 0x00};  
    frame[3] = PGA460_CalculateChecksum(&frame[1], 3);

    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, frame, 4, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Register Read Transmission Failed! Address 0x%02X\n", sensorID, regAddr);
        return HAL_ERROR;
    }

    uint8_t response[3];
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, response, 3, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Register Read Failed! Address 0x%02X\n", sensorID, regAddr);
        return HAL_ERROR;
    }

    *regValue = response[1];
    return HAL_OK;
}

// Function to write a register on the PGA460
HAL_StatusTypeDef PGA460_RegisterWrite(const uint8_t sensorID, const uint8_t regAddr, const uint8_t regValue) {
    uint8_t frame[5] = {PGA460_SYNC, PGA460_CMD_REGISTER_WRITE, regAddr, regValue, 0x00};  
    frame[4] = PGA460_CalculateChecksum(&frame[1], 4);

    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, frame, 5, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Register Write Transmission Failed! Address 0x%02X\n", sensorID, regAddr);
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef PGA460_EEPROMBulkRead(const uint8_t sensorID, PGA460_Sensor_t *dataBuffer) {
    //uint8_t frame[3] = {PGA460_SYNC, PGA460_CMD_EEPROM_BULK_READ, 0x00}; 
    // Compute checksum
    //frame[2] = PGA460_CalculateChecksum(&frame[1], 1);

    // Step 1: Send EEPROM Bulk Read Command
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_EEPROM_BULK_READ].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Bulk Read Command Failed!\n", sensorID);
        return HAL_ERROR;
    }

    HAL_Delay(10);  // Allow time for EEPROM data retrieval

    // Step 2: Receive 43 bytes into `dataBuffer`
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, (uint8_t *)dataBuffer, 43, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Bulk Read Failed!\n", sensorID);
        return HAL_ERROR;
    }

    DEBUG("Sensor %d: EEPROM Bulk Read Successful!\n", sensorID);
    return HAL_OK;
}

// EEPROM bulk write
HAL_StatusTypeDef PGA460_EEPROMBulkWrite(uint8_t sensorID) {
    uint8_t frame[46];
    // Step 1: Construct command frame
    frame[0] = PGA460_SYNC;
    frame[1] = PGA460_CMD_EEPROM_BULK_WRITE;
    // Copy predefined transducer struct into frame starting at frame[2]
    memcpy(&frame[2], &myUltraSonicArray[sensorID].PGA460_Data, 43);
    // Compute checksum (excluding sync byte)
    frame[45] = PGA460_CalculateChecksum(&frame[1], 44);
    // Step 2: Transmit Bulk Write Command
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, frame, 46, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Bulk Write to Volatile Memory Failed!\n", sensorID);
        return HAL_ERROR;
    }
    HAL_Delay(50);  // Allow the write operation to complete
    DEBUG("Sensor %d: Bulk Write to Volatile Memory Successful!\n", sensorID);
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_BurnEEPROM(uint8_t sensorID) {
    uint8_t burnStatus;

    // Step 1: Unlock EEPROM (Write 0x68 to REG_EE_CTRL)
    if (PGA460_RegisterWrite(sensorID, REG_EE_CTRL, PGA460_UNLOCK_EEPROM) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Unlock Failed at Step 1!\n", sensorID);
        return HAL_ERROR;
    }
    HAL_Delay(1); // Required minimal delay

    // Step 2: Program EEPROM (Write 0x69 to REG_EE_CTRL)
    if (PGA460_RegisterWrite(sensorID, REG_EE_CTRL, PGA460_LOCK_EEPROM) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Burn Command Failed at Step 2!\n", sensorID);
        return HAL_ERROR;
    }
    HAL_Delay(1000); // Wait for EEPROM programming to complete

    // Step 3: Verify EEPROM Burn Success (Read REG_EE_CTRL)
    if (PGA460_RegisterRead(sensorID, REG_EE_CTRL, &burnStatus) != HAL_OK) {
        DEBUG("Sensor %d: Failed to Read EE_CTRL after EEPROM Burn!\n", sensorID);
        return HAL_ERROR;
    }

    // Step 4: Check EE_PGRM_OK (Bit 2)
    if (burnStatus & 0x04) {
        DEBUG("Sensor %d: EEPROM Burn Successful! EE_CTRL = 0x%02X\n", sensorID, burnStatus);
        return HAL_OK;
    } else {
        DEBUG("Sensor %d: EEPROM Burn Failed! EE_CTRL = 0x%02X\n", sensorID, burnStatus);
        return HAL_ERROR;
    }
}

HAL_StatusTypeDef PGA460_InitTimeVaryingGain(const uint8_t sensorID, const PGA460_GainRange_t gain_range, const PGA460_TVG_Level_t timeVaryingGain) {
    // Step 1: Write AFE Gain Range to Register 0x26 (REG_DECPL_TEMP)
    if (PGA460_RegisterWrite(sensorID, REG_DECPL_TEMP, gain_range) != HAL_OK) {
        DEBUG("Sensor %d: AFE Gain Range Write Failed!\n", sensorID);
        return HAL_ERROR;
    }

    // Step 2: Select TVG Level Array
    const uint8_t *tvg_values;
    switch (timeVaryingGain) {
        case PGA460_TVG_25_PERCENT:
            tvg_values = TGV_25;
            break;
        case PGA460_TVG_50_PERCENT:
            tvg_values = TGV_50;
            break;
        case PGA460_TVG_75_PERCENT:
            tvg_values = TGV_75;
            break;
        default:
            DEBUG("Sensor %d: Invalid TVG Level!\n", sensorID);
            return HAL_ERROR;
    }

    // Step 3: Construct TVG Bulk Write Frame
    uint8_t frame[10] = {PGA460_SYNC, PGA460_CMD_TVG_BULK_WRITE};
    memcpy(&frame[2], tvg_values, 7);
    frame[9] = PGA460_CalculateChecksum(&frame[1], 8);

    // Step 4: Transmit TVG Bulk Write Command
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, frame, 9, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: TVG Bulk Write Transmission Failed!\n", sensorID);
        return HAL_ERROR;
    }

    HAL_Delay(50);  // Allow time for TVG update

    DEBUG("Sensor %d: TVG Bulk Write Successful!\n", sensorID);
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_UltrasonicCmd(const uint8_t sensorID, const PGA460_Command_t cmd, const uint8_t numObjUpdate) {
    uint8_t bufCmd[4] = {PGA460_SYNC, cmd, numObjUpdate, 0x00}; // Initialize buffer
	//the CheckSum is 
    // Compute Checksum
    bufCmd[3] = PGA460_CalculateChecksum(&bufCmd[1], 2);
    // Transmit the command 
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, bufCmd, 4, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Ultrasonic Command Transmission Failed!\n", sensorID);
        return HAL_ERROR;
    }
    // Delay for maximum record length (65ms + margin)
    HAL_Delay(70);
    DEBUG("Sensor %d: Ultrasonic Command %d Executed Successfully!\n", sensorID, cmd);
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_GetUltrasonicMeasurement(const uint8_t sensorID) {
    //uint8_t frame[3] = {PGA460_SYNC, PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT, 0x00};
    //frame[2] = PGA460_CalculateChecksum(&frame[1], 1);
		DEBUG("PGA_OBJ_DATA_SIZE %d!\n", PGA_OBJ_DATA_SIZE);
    // Send command via UART
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Measurement Result Request Failed!\n", sensorID);
        return HAL_ERROR;
    }
    //HAL_Delay(10);

    // Receive expected data: 2 bytes (header) + PGA_OBJECTS_TRACKED * 4 (distance, width, amplitude)
    uint8_t localBuffer[PGA_OBJ_DATA_SIZE];
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, localBuffer, PGA_OBJ_DATA_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Measurement Result Retrieval Failed!\n", sensorID);
        return HAL_ERROR;
    }

    // Store Data in myUltraSonicArray[sensorID]
    for (uint8_t i = 0; i < PGA_OBJECTS_TRACKED; i++) {
        // Extract Distance (16-bit value)
        myUltraSonicArray[sensorID].ultrasonicData.objects[i].distance  = (localBuffer[2 + (i * 4)] << 8) | localBuffer[3 + (i * 4)];

        // Extract Width (8-bit)
        myUltraSonicArray[sensorID].ultrasonicData.objects[i].width     = localBuffer[4 + (i * 4)];

        // Extract Amplitude (8-bit)
        myUltraSonicArray[sensorID].ultrasonicData.objects[i].amplitude = localBuffer[5 + (i * 4)];

        // Convert distance to meters using the speed of sound (343 m/s)
        float distance_m = (myUltraSonicArray[sensorID].ultrasonicData.objects[i].distance / 2.0f) * 0.000001f * 343.0f;

        DEBUG("Sensor %d, Obj %d - Distance: %.2f m, Width: %u us, Amplitude: %u\n",
              sensorID, i + 1, distance_m,
              myUltraSonicArray[sensorID].ultrasonicData.objects[i].width * 16, // Convert width to microseconds
              myUltraSonicArray[sensorID].ultrasonicData.objects[i].amplitude);
    }

    return HAL_OK;
}

float PGA460_ReadTemperatureOrNoise(const uint8_t sensorID, const PGA460_CmdType_t mode) {
    uint8_t tempOrNoise = (mode == PGA460_CMD_GET_TEMP) ? 0 : 1;  // 0 for temperature, 1 for noise
    uint8_t buffer[4];  // Reused buffer for sending and receiving
    float result = PGA460_TEMP_ERR; // Default invalid value

    // **Step 1: Send Temperature/Noise Measurement Command**
    buffer[0] = PGA460_SYNC;
    buffer[1] = PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT;
    buffer[2] = tempOrNoise;
    buffer[3] = PGA460_CalculateChecksum(&buffer[1], 2);

    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, buffer, 4, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Temperature/Noise Measurement Command Failed!\n", sensorID);
        return result;
    }
		//Flush Wait Flush
    HAL_Delay(10);  // Wait for the PGA460 to process the measurement

    // **Step 2: Send Read Temperature/Noise Result Command**
    //buffer[1] = PGA460_CMD_TEMP_AND_NOISE_RESULT;
    //buffer[2] = PGA460_CalculateChecksum(&buffer[1], 1);

    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_TEMP_AND_NOISE_RESULT].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Temperature/Noise Result Request Failed!\n", sensorID);
        return result;
    }

    // **Step 3: Read 4-byte Response**
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, buffer, 4, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Failed to Receive Temperature/Noise Data!\n", sensorID);
        return result;
    }

    // **Step 4: Process the Received Data**
    if (mode == PGA460_CMD_GET_TEMP) {
        result = (buffer[1] - 64) / 1.5f;  // Convert to temperature in °C
    } else {
        result = (float)buffer[2];  // Noise level (8-bit raw value)
    }

    DEBUG("Sensor %d: %s Data: %.2f\n", sensorID, (mode == PGA460_CMD_GET_TEMP) ? "Temperature (C)" : "Noise Level", result);

    return result;
}

HAL_StatusTypeDef PGA460_SetTemperatureOffset(uint8_t sensorID, float externalTempC) {
    float pgaTempC = 0.0f;
    int8_t offset = 0;
    uint8_t tempTrim = 0;

    // Step 1: Read PGA460's internal temperature
    pgaTempC = PGA460_ReadTemperatureOrNoise(sensorID, PGA460_CMD_GET_TEMP);
    
    // Check for invalid readings
    if (pgaTempC == PGA460_TEMP_ERR) {
        DEBUG("Sensor %d: Failed to read internal temperature!\n", sensorID);
        return HAL_ERROR;
    }

    DEBUG("Sensor %d: PGA460 Reported Temperature = %.2fC\n", sensorID, pgaTempC);

    // Step 2: Calculate the required temperature offset (round to nearest integer)
    offset = (int8_t)(externalTempC - pgaTempC);

    // Clamp offset to valid range (-64 to +63)
    if (offset > 63) offset = 63;
    if (offset < -64) offset = -64;

    // Step 3: Read the current TEMP_TRIM register value
    if (PGA460_RegisterRead(sensorID, REG_TEMP_TRIM, &tempTrim) != HAL_OK) {
        DEBUG("Sensor %d: Failed to read TEMP_TRIM register!\n", sensorID);
        return HAL_ERROR;
    }

    DEBUG("Sensor %d: Current TEMP_TRIM = 0x%02X\n", sensorID, tempTrim);

    // Step 4: Apply Temperature Offset (Bits 6-0 should be modified)
    tempTrim = (tempTrim & 0x80) | (offset & 0x7F);

    // Step 5: Write the new TEMP_TRIM value back
    if (PGA460_RegisterWrite(sensorID, REG_TEMP_TRIM, tempTrim) != HAL_OK) {
        DEBUG("Sensor %d: Failed to write TEMP_TRIM register!\n", sensorID);
        return HAL_ERROR;
    }

    DEBUG("Sensor %d: New TEMP_TRIM set to 0x%02X (Offset: %dC)\n", sensorID, tempTrim, offset);

    return HAL_OK;
}

HAL_StatusTypeDef PGA460_GetSystemDiagnostics(const uint8_t sensorID, const uint8_t run, const uint8_t diag, float *diagResult) {
    uint8_t response[4] = {0x00, 0x00, 0x00, 0x00};

    // **Step 1: Issue a Burst-and-Listen Command if Requested**
    if (run) {
        if (PGA460_UltrasonicCmd(sensorID, PGA460_CMD_BURST_AND_LISTEN_PRESET1, 1) != HAL_OK) {
            DEBUG("Sensor %d: Burst-and-Listen Command Failed!\n", sensorID);
            return HAL_ERROR;
        }
        HAL_Delay(100); // Allow time for measurement

        // **Step 2: Send System Diagnostics Command**
        //uint8_t frame[3] = {PGA460_SYNC, PGA460_CMD_SYSTEM_DIAGNOSTICS, 0x00};
        //frame[2] = PGA460_CalculateChecksum(&frame[1], 1);

        if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_SYSTEM_DIAGNOSTICS].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
            DEBUG("Sensor %d: System Diagnostics Request Failed!\n", sensorID);
            return HAL_ERROR;
        }
    }

    // **Step 3: Receive Diagnostic Data**
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, response, 4, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: System Diagnostics Retrieval Failed!\n", sensorID);
        return HAL_ERROR;
    }

    // **Step 4: Handle Temperature or Noise Separately**
    if (diag == 2) {  // Temperature
        *diagResult = PGA460_ReadTemperatureOrNoise(sensorID, PGA460_CMD_GET_TEMP);
        if (*diagResult == PGA460_TEMP_ERR) {
            DEBUG("Sensor %d: Failed to Retrieve Temperature Data!\n", sensorID);
            return HAL_ERROR;
        }
        return HAL_OK;
    } 
    else if (diag == 3) {  // Noise Level
        *diagResult = PGA460_ReadTemperatureOrNoise(sensorID, PGA460_CMD_GET_NOISE);
        if (*diagResult == PGA460_TEMP_ERR) {
            DEBUG("Sensor %d: Failed to Retrieve Noise Data!\n", sensorID);
            return HAL_ERROR;
        }
        return HAL_OK;
    }

    // **Step 5: Process and Convert Remaining Diagnostic Results**
    switch (diag) {
        case 0: // Frequency diagnostic in kHz
            *diagResult = (1.0 / (response[1] * 0.0000005)) / 1000.0;
            break;
        case 1: // Decay period in microseconds
            *diagResult = response[2] * 16.0;
            break;
        default:
            DEBUG("Sensor %d: Invalid Diagnostic Request!\n", sensorID);
            return HAL_ERROR;
    }

    DEBUG("Sensor %d: Diagnostic Result for Type %d = %f\n", sensorID, diag, *diagResult);
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_GetTimeVaryingGain(const uint8_t sensorID, uint8_t *gainData) {
    //uint8_t frame[3] = {PGA460_SYNC, PGA460_CMD_TVG_BULK_READ, 0x00};
    //frame[2] = PGA460_CalculateChecksum(&frame[1], 1);
    // Send command
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_TVG_BULK_READ].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: TVG Bulk Read Request Failed!\n", sensorID);
        return HAL_ERROR;
    }
    HAL_Delay(10);
    // Receive 7 bytes of TVG data
    uint8_t localBuffer[7];
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, localBuffer, 7, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: TVG Bulk Read Failed!\n", sensorID);
        return HAL_ERROR;
    }
    memcpy(gainData, localBuffer, 7);
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_GetThresholds(const uint8_t sensorID, uint8_t *thresholdData) {
    //uint8_t frame[3] = {PGA460_SYNC, PGA460_CMD_THRESHOLD_BULK_READ, 0x00};
    //frame[2] = PGA460_CalculateChecksum(&frame[1], 1);
    // Send command
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_THRESHOLD_BULK_READ].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Threshold Bulk Read Request Failed!\n", sensorID);
        return HAL_ERROR;
    }
    HAL_Delay(10);
    // Receive 32 bytes of threshold data
    uint8_t localBuffer[32];
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, localBuffer, 32, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Threshold Bulk Read Failed!\n", sensorID);
        return HAL_ERROR;
    }
    memcpy(thresholdData, localBuffer, 32);
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_GetEchoDataDump(const uint8_t sensorID, uint8_t *dataBuffer) {
    //uint8_t frame[3] = {PGA460_SYNC, PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP, 0x00};
    //frame[2] = PGA460_CalculateChecksum(&frame[1], 1);
    // Send command
    if (HAL_UART_Transmit(myUltraSonicArray[sensorID].uartPort, commands[PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP].cmdData, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Echo Data Dump Request Failed!\n", sensorID);
        return HAL_ERROR;
    }
    HAL_Delay(10);
    // Receive 128 bytes of echo data
    uint8_t localBuffer[128];
    if (HAL_UART_Receive(myUltraSonicArray[sensorID].uartPort, localBuffer, 128, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Echo Data Dump Failed!\n", sensorID);
        return HAL_ERROR;
    }
    memcpy(dataBuffer, localBuffer, 128);
    return HAL_OK;
}

void setBandpassFilterBandwidth(const uint8_t sensorID, uint8_t bandWidth) {
    myUltraSonicArray[sensorID].PGA460_Data.INIT_GAIN.Val.BitField.BPF_BW = (bandWidth - 2) / 2;
}

uint8_t getBandpassFilterBandwidth(const uint8_t sensorID) {
    return 2 * (myUltraSonicArray[sensorID].PGA460_Data.INIT_GAIN.Val.BitField.BPF_BW + 1);
}

void setInitialGain(const uint8_t sensorID, uint8_t gain) {
    int gainValue = 2 * (gain - myUltraSonicArray[sensorID].PGA460_Data.DECPL_TEMP.Val.BitField.AFE_GAIN_RNG);
    myUltraSonicArray[sensorID].PGA460_Data.INIT_GAIN.Val.BitField.GAIN_INIT = (gainValue > 0) ? gainValue - 1 : 0;
}

uint8_t getInitialGain(const uint8_t sensorID) {
    return (0.5 * (myUltraSonicArray[sensorID].PGA460_Data.INIT_GAIN.Val.BitField.GAIN_INIT + 1)) +
           myUltraSonicArray[sensorID].PGA460_Data.DECPL_TEMP.Val.BitField.AFE_GAIN_RNG;
}

void setFrequency(const uint8_t sensorID, uint8_t frequency) {
    myUltraSonicArray[sensorID].PGA460_Data.FREQ = (uint8_t)((frequency - 30) / 0.2);
    if (myUltraSonicArray[sensorID].PGA460_Data.FREQ > 250) myUltraSonicArray[sensorID].PGA460_Data.FREQ = 250;
}

uint8_t getFrequency(const uint8_t sensorID) {
    return (uint8_t)(0.2 * myUltraSonicArray[sensorID].PGA460_Data.FREQ + 30);
}

void setDeglitchPeriod(const uint8_t sensorID, uint8_t deglitchPeriod) {
    myUltraSonicArray[sensorID].PGA460_Data.DEADTIME.Val.BitField.THR_CMP_DEGLTCH = deglitchPeriod / 8;
}

uint8_t getDeglitchPeriod(const uint8_t sensorID) {
    return 8 * myUltraSonicArray[sensorID].PGA460_Data.DEADTIME.Val.BitField.THR_CMP_DEGLTCH;
}

void setBurstPulseDeadTime(const uint8_t sensorID, uint8_t burstPulseDeadTime) {
    myUltraSonicArray[sensorID].PGA460_Data.DEADTIME.Val.BitField.PULSE_DT = burstPulseDeadTime / 0.0625;
}

uint8_t getBurstPulseDeadTime(const uint8_t sensorID) {
    return myUltraSonicArray[sensorID].PGA460_Data.DEADTIME.Val.BitField.PULSE_DT * 0.0625;
}

void setBurstPulseP1(const uint8_t sensorID, uint8_t burstPulseP1) {
    if (burstPulseP1 > 31) burstPulseP1 = 31;
    myUltraSonicArray[sensorID].PGA460_Data.PULSE_P1.Val.BitField.P1_PULSE = burstPulseP1;
}

void setBurstPulseP2(const uint8_t sensorID, uint8_t burstPulseP2) {
    if (burstPulseP2 > 31) burstPulseP2 = 31;
    myUltraSonicArray[sensorID].PGA460_Data.PULSE_P2.Val.BitField.P2_PULSE = burstPulseP2;
}

void setAddress(const uint8_t sensorID, uint8_t address) {
    if (address > 7) address = 7;
    myUltraSonicArray[sensorID].PGA460_Data.PULSE_P2.Val.BitField.UART_ADDR = address;
}

void setCurrentLimitStatus(const uint8_t sensorID, uint8_t currentLimitStatus) {
    myUltraSonicArray[sensorID].PGA460_Data.CURR_LIM_P1.Val.BitField.DIS_CL = currentLimitStatus ? 1 : 0;
}

void setSleepModeTimer(const uint8_t sensorID, uint8_t sleepModeTimer) {
    if (sleepModeTimer > 3) sleepModeTimer = 3;
    myUltraSonicArray[sensorID].PGA460_Data.FVOLT_DEC.Val.BitField.LPM_TMR = sleepModeTimer;
}

void setDecoupleTimeOrTemperature(const uint8_t sensorID, uint8_t decoupleValue) {
    if (decoupleValue > 15) decoupleValue = 15;
    myUltraSonicArray[sensorID].PGA460_Data.DECPL_TEMP.Val.BitField.DECPL_T = decoupleValue;
}

void setNoiseLevel(const uint8_t sensorID, uint8_t noiseLevel) {
    if (noiseLevel > 31) noiseLevel = 31;
    myUltraSonicArray[sensorID].PGA460_Data.DSP_SCALE.Val.BitField.NOISE_LVL = noiseLevel;
}

void setTemperatureScaleOffset(const uint8_t sensorID, int8_t tempOffset) {
    if (tempOffset < -8) tempOffset = -8;
    else if (tempOffset > 7) tempOffset = 7;
    myUltraSonicArray[sensorID].PGA460_Data.TEMP_TRIM.Val.BitField.TEMP_OFF = tempOffset & 0x0F;
}

int8_t getTemperatureScaleOffset(const uint8_t sensorID) {
    int8_t offset = myUltraSonicArray[sensorID].PGA460_Data.TEMP_TRIM.Val.BitField.TEMP_OFF;
    return (offset & 0x08) ? (offset | 0xF0) : offset;
}

// Set Temperature Scale Gain (signed value range -8 to 7)
void setTemperatureScaleGain(const uint8_t sensorID, int8_t tempGain) {
    if (tempGain < -8) tempGain = -8;
    else if (tempGain > 7) tempGain = 7;
    myUltraSonicArray[sensorID].PGA460_Data.TEMP_TRIM.Val.BitField.TEMP_GAIN = tempGain & 0x0F;
}

// Get Temperature Scale Gain
int8_t getTemperatureScaleGain(const uint8_t sensorID) {
    int8_t gain = myUltraSonicArray[sensorID].PGA460_Data.TEMP_TRIM.Val.BitField.TEMP_GAIN;
    return (gain & 0x08) ? (gain | 0xF0) : gain;
}
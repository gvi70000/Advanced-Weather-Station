// TO BE USED WITH PGA460.html to generate registers values

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "usart.h"
#include "PGA460.h"
#include "debug.h"

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif

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

uint8_t EEPROM_Stat = 0;

const uint8_t pga460_cmd_burst_and_listen_preset1[PGA_CMD_SIZE] =       {PGA460_SYNC, PGA460_CMD_BURST_AND_LISTEN_PRESET1,       255 - PGA460_CMD_BURST_AND_LISTEN_PRESET1};
const uint8_t pga460_cmd_burst_and_listen_preset2[PGA_CMD_SIZE] =       {PGA460_SYNC, PGA460_CMD_BURST_AND_LISTEN_PRESET2,       255 - PGA460_CMD_BURST_AND_LISTEN_PRESET2};
const uint8_t pga460_cmd_listen_only_preset1[PGA_CMD_SIZE] =            {PGA460_SYNC, PGA460_CMD_LISTEN_ONLY_PRESET1,            255 - PGA460_CMD_LISTEN_ONLY_PRESET1};
const uint8_t pga460_cmd_listen_only_preset2[PGA_CMD_SIZE] =            {PGA460_SYNC, PGA460_CMD_LISTEN_ONLY_PRESET2,            255 - PGA460_CMD_LISTEN_ONLY_PRESET2};
const uint8_t pga460_cmd_temp_and_noise_measurement[PGA_CMD_SIZE] =     {PGA460_SYNC, PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT,     255 - PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT};
const uint8_t pga460_cmd_ultrasonic_measurement_result[PGA_CMD_SIZE] =  {PGA460_SYNC, PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT,  255 - PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT};
const uint8_t pga460_cmd_temp_and_noise_result[PGA_CMD_SIZE] =          {PGA460_SYNC, PGA460_CMD_TEMP_AND_NOISE_RESULT,          255 - PGA460_CMD_TEMP_AND_NOISE_RESULT};
const uint8_t pga460_cmd_transducer_echo_data_dump[PGA_CMD_SIZE] =      {PGA460_SYNC, PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP,      255 - PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP};
const uint8_t pga460_cmd_system_diagnostics[PGA_CMD_SIZE] =             {PGA460_SYNC, PGA460_CMD_SYSTEM_DIAGNOSTICS,             255 - PGA460_CMD_SYSTEM_DIAGNOSTICS};
const uint8_t pga460_cmd_register_read[PGA_CMD_SIZE] =                  {PGA460_SYNC, PGA460_CMD_REGISTER_READ,                  255 - PGA460_CMD_REGISTER_READ};
const uint8_t pga460_cmd_register_write[PGA_CMD_SIZE] =                 {PGA460_SYNC, PGA460_CMD_REGISTER_WRITE,                 255 - PGA460_CMD_REGISTER_WRITE};
const uint8_t pga460_cmd_eeprom_bulk_read[PGA_CMD_SIZE] =               {PGA460_SYNC, PGA460_CMD_EEPROM_BULK_READ,               255 - PGA460_CMD_EEPROM_BULK_READ};
// const uint8_t pga460_cmd_eeprom_bulk_write[PGA_CMD_SIZE] =              {PGA460_SYNC, PGA460_CMD_EEPROM_BULK_WRITE,              255 - PGA460_CMD_EEPROM_BULK_WRITE};
const uint8_t pga460_cmd_tvg_bulk_read[PGA_CMD_SIZE] =                  {PGA460_SYNC, PGA460_CMD_TVG_BULK_READ,                  255 - PGA460_CMD_TVG_BULK_READ};
// const uint8_t pga460_cmd_tvg_bulk_write[PGA_CMD_SIZE] =                 {PGA460_SYNC, PGA460_CMD_TVG_BULK_WRITE,                 255 - PGA460_CMD_TVG_BULK_WRITE};
const uint8_t pga460_cmd_threshold_bulk_read[PGA_CMD_SIZE] =            {PGA460_SYNC, PGA460_CMD_THRESHOLD_BULK_READ,            255 - PGA460_CMD_THRESHOLD_BULK_READ};
const uint8_t pga460_cmd_threshold_bulk_write[PGA_CMD_SIZE] =           {PGA460_SYNC, PGA460_CMD_THRESHOLD_BULK_WRITE,           255 - PGA460_CMD_THRESHOLD_BULK_WRITE};

// Broadcast Command Arrays
// const uint8_t pga460_cmd_broadcast_burst_and_listen_p1[PGA_CMD_SIZE]  = {PGA460_SYNC, PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1,  255 - PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1};
// const uint8_t pga460_cmd_broadcast_burst_and_listen_p2[PGA_CMD_SIZE]  = {PGA460_SYNC, PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2,  255 - PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2};
// const uint8_t pga460_cmd_broadcast_listen_only_p1[PGA_CMD_SIZE]       = {PGA460_SYNC, PGA460_CMD_BROADCAST_LISTEN_ONLY_P1,       255 - PGA460_CMD_BROADCAST_LISTEN_ONLY_P1};
// const uint8_t pga460_cmd_broadcast_listen_only_p2[PGA_CMD_SIZE]       = {PGA460_SYNC, PGA460_CMD_BROADCAST_LISTEN_ONLY_P2,       255 - PGA460_CMD_BROADCAST_LISTEN_ONLY_P2};
// const uint8_t pga460_cmd_broadcast_temp_noise_measure[PGA_CMD_SIZE]   = {PGA460_SYNC, PGA460_CMD_BROADCAST_TEMP_NOISE_MEASURE,   255 - PGA460_CMD_BROADCAST_TEMP_NOISE_MEASURE};
// const uint8_t pga460_cmd_broadcast_register_write[PGA_CMD_SIZE]       = {PGA460_SYNC, PGA460_CMD_BROADCAST_REGISTER_WRITE,       255 - PGA460_CMD_BROADCAST_REGISTER_WRITE};
// const uint8_t pga460_cmd_broadcast_eeprom_bulk_write[PGA_CMD_SIZE]    = {PGA460_SYNC, PGA460_CMD_BROADCAST_EEPROM_BULK_WRITE,    255 - PGA460_CMD_BROADCAST_EEPROM_BULK_WRITE};
// const uint8_t pga460_cmd_broadcast_tvg_bulk_write[PGA_CMD_SIZE]       = {PGA460_SYNC, PGA460_CMD_BROADCAST_TVG_BULK_WRITE,       255 - PGA460_CMD_BROADCAST_TVG_BULK_WRITE};
// const uint8_t pga460_cmd_broadcast_threshold_bulk_write[PGA_CMD_SIZE] = {PGA460_SYNC, PGA460_CMD_BROADCAST_THRESHOLD_BULK_WRITE, 255 - PGA460_CMD_BROADCAST_THRESHOLD_BULK_WRITE};

static const uint16_t time_lookup[16] = {100, 200, 300, 400, 600, 800, 1000, 1200, 1400, 2000, 2400, 3200, 4000, 5200, 6400, 8000};

const PGA460_Regs_t s1 = {
  .EEData.UserData = {0},    // 0x00..0x13
  .EEData.TVG = {            // keep your proven TVG shape
    .TVGAIN0 = 68, .TVGAIN1 = 17, .TVGAIN2 = 52,
    .TVGAIN3 = 3,  .TVGAIN4 = 63, .TVGAIN5 = 127, .TVGAIN6 = 252
  },
  .EEData.Sett = {
    .INIT_GAIN    = 0x00,          // low initial gain (near target)
    .FREQUENCY    = 0x8C,          // 58.0 kHz (0.2*140+30)  ? CUSA center freq
    .DEADTIME     = 0x00,          // deglitch=32 µs, pulse DT=0
    .PULSE_P1     = 0x04,          // IO_IF_SEL=0 (time-based), DIAG=1, P1=4 pulses
    .PULSE_P2     = 0x08,          // P2=8 pulses (addr=0)
    .CURR_LIM_P1  = 0x08,          // keep your hardware-safe limits
    .CURR_LIM_P2  = 0x08,
    .REC_LENGTH   = 0x00,          // 4.096 ms window per preset
    .FREQ_DIAG    = 0x20,          // standard diag enable
    .SAT_FDIAG_TH = 0xEE,          // TI typical
    .FVOLT_DEC    = 0x7C,          // TI typical
    .DECPL_TEMP   = 0x4F,          // TI typical
    .DSP_SCALE    = 0x00,
    .TEMP_TRIM    = 0x88,
    .P1_GAIN_CTRL = 0x00,
    .P2_GAIN_CTRL = 0x00
  },
  .eeCtrl = { .Val.Value = 0 },
  .Filters = {                   // fine to leave as you had them
    .BPF_A2_MSB=37, .BPF_A2_LSB=137,
    .BPF_A3_MSB=38, .BPF_A3_LSB=252,
    .BPF_B1_MSB=1,  .BPF_B1_LSB=153,
    .LPF_A2_MSB=128,.LPF_A2_LSB=205,
    .LPF_B1_MSB=0,  .LPF_B1_LSB=103
  },
  .TestMux = { .Val.Value = 0 },
  .Stat0   = { .Val.Value = 0 },
  .Stat1   = { .Val.Value = 0 },
  .THR = {
    // keep your short-range threshold profile
    .P1_THR = {0, 0, 0, 0, 0, 0, 24, 198, 49, 140, 98, 19, 20, 20, 20, 7},
    .P2_THR = {0, 0, 0, 0, 0, 0, 24, 198, 49, 140, 99, 22, 24, 25, 25, 7}
  }
};
const PGA460_Regs_t s2 = {
  .EEData.UserData = {0},    // 0x00..0x13
  .EEData.TVG = {            // keep your proven TVG shape
    .TVGAIN0 = 68, .TVGAIN1 = 17, .TVGAIN2 = 52,
    .TVGAIN3 = 3,  .TVGAIN4 = 63, .TVGAIN5 = 127, .TVGAIN6 = 252
  },
  .EEData.Sett = {
    .INIT_GAIN    = 0x00,          // low initial gain (near target)
    .FREQUENCY    = 0x8C,          // 58.0 kHz (0.2*140+30)  ? CUSA center freq
    .DEADTIME     = 0x00,          // deglitch=32 µs, pulse DT=0
    .PULSE_P1     = 0x04,          // IO_IF_SEL=0 (time-based), DIAG=1, P1=4 pulses
    .PULSE_P2     = 0x08,          // P2=8 pulses (addr=0)
    .CURR_LIM_P1  = 0x08,          // keep your hardware-safe limits
    .CURR_LIM_P2  = 0x08,
    .REC_LENGTH   = 0x00,          // 4.096 ms window per preset
    .FREQ_DIAG    = 0x20,          // standard diag enable
    .SAT_FDIAG_TH = 0xEE,          // TI typical
    .FVOLT_DEC    = 0x7C,          // TI typical
    .DECPL_TEMP   = 0x4F,          // TI typical
    .DSP_SCALE    = 0x00,
    .TEMP_TRIM    = 0x88,
    .P1_GAIN_CTRL = 0x00,
    .P2_GAIN_CTRL = 0x00
  },
  .eeCtrl = { .Val.Value = 0 },
  .Filters = {                   // fine to leave as you had them
    .BPF_A2_MSB=37, .BPF_A2_LSB=137,
    .BPF_A3_MSB=38, .BPF_A3_LSB=252,
    .BPF_B1_MSB=1,  .BPF_B1_LSB=153,
    .LPF_A2_MSB=128,.LPF_A2_LSB=205,
    .LPF_B1_MSB=0,  .LPF_B1_LSB=103
  },
  .TestMux = { .Val.Value = 0 },
  .Stat0   = { .Val.Value = 0 },
  .Stat1   = { .Val.Value = 0 },
  .THR = {
    // keep your short-range threshold profile
    .P1_THR = {0, 0, 0, 0, 0, 0, 24, 132, 49, 140, 99, 25, 11, 10, 25, 7},
    .P2_THR = {0, 0, 0, 0, 0, 0, 24, 198, 49, 136, 99, 25, 11, 12, 11, 7}
  }
};
const PGA460_Regs_t s3 = {
  .EEData.UserData = {0},    // 0x00..0x13
  .EEData.TVG = {            // keep your proven TVG shape
    .TVGAIN0 = 68, .TVGAIN1 = 17, .TVGAIN2 = 52,
    .TVGAIN3 = 3,  .TVGAIN4 = 63, .TVGAIN5 = 127, .TVGAIN6 = 252
  },
  .EEData.Sett = {
    .INIT_GAIN    = 0x00,          // low initial gain (near target)
    .FREQUENCY    = 0x8C,          // 58.0 kHz (0.2*140+30)  ? CUSA center freq
    .DEADTIME     = 0x00,          // deglitch=32 µs, pulse DT=0
    .PULSE_P1     = 0x04,          // IO_IF_SEL=0 (time-based), DIAG=1, P1=4 pulses
    .PULSE_P2     = 0x08,          // P2=8 pulses (addr=0)
    .CURR_LIM_P1  = 0x08,          // keep your hardware-safe limits
    .CURR_LIM_P2  = 0x08,
    .REC_LENGTH   = 0x00,          // 4.096 ms window per preset
    .FREQ_DIAG    = 0x20,          // standard diag enable
    .SAT_FDIAG_TH = 0xEE,          // TI typical
    .FVOLT_DEC    = 0x7C,          // TI typical
    .DECPL_TEMP   = 0x4F,          // TI typical
    .DSP_SCALE    = 0x00,
    .TEMP_TRIM    = 0x88,
    .P1_GAIN_CTRL = 0x00,
    .P2_GAIN_CTRL = 0x00
  },
  .eeCtrl = { .Val.Value = 0 },
  .Filters = {                   // fine to leave as you had them
    .BPF_A2_MSB=37, .BPF_A2_LSB=137,
    .BPF_A3_MSB=38, .BPF_A3_LSB=252,
    .BPF_B1_MSB=1,  .BPF_B1_LSB=153,
    .LPF_A2_MSB=128,.LPF_A2_LSB=205,
    .LPF_B1_MSB=0,  .LPF_B1_LSB=103
  },
  .TestMux = { .Val.Value = 0 },
  .Stat0   = { .Val.Value = 0 },
  .Stat1   = { .Val.Value = 0 },
  .THR = {
    // keep your short-range threshold profile
    .P1_THR = {0, 0, 0, 0, 0, 0, 24, 132, 33, 136, 67, 19, 17, 17, 16, 7},
    .P2_THR = {0, 0, 0, 0, 0, 0, 24, 198, 49, 140, 67, 25, 23, 19, 18, 7}
  }
};

// --- Array with 3 elements(sensors): [0] TVG, [1] Settings, [2] Thresholds ---
PGA460_Sensor_t sensors[ULTRASONIC_SENSOR_COUNT] = {
	{ .uartPort = &huart1, .Registers = s1 },
	{ .uartPort = &huart4, .Registers = s2 },
	{ .uartPort = &huart5, .Registers = s3 }
};

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

static void PGA460_ReadAndPrintEEPROM(uint8_t sensorID) {
    printf("Reading and printing EEPROM registers for Sensor %u individually...\n", sensorID);
    uint8_t regValue = 0;
    // EEPROM registers range from 0x00 to 0x2B
    for (uint8_t regAddr = 0x00; regAddr <= 0x2B; ++regAddr) {
        // Read the single register
        if (PGA460_RegisterRead(sensorID, regAddr, &regValue) == HAL_OK) {
            const char* regName;
            switch (regAddr) {
                case REG_USER_DATA1: regName = "USER_DATA1"; break;
                case REG_USER_DATA2: regName = "USER_DATA2"; break;
                case REG_USER_DATA3: regName = "USER_DATA3"; break;
                case REG_USER_DATA4: regName = "USER_DATA4"; break;
                case REG_USER_DATA5: regName = "USER_DATA5"; break;
                case REG_USER_DATA6: regName = "USER_DATA6"; break;
                case REG_USER_DATA7: regName = "USER_DATA7"; break;
                case REG_USER_DATA8: regName = "USER_DATA8"; break;
                case REG_USER_DATA9: regName = "USER_DATA9"; break;
                case REG_USER_DATA10: regName = "USER_DATA10"; break;
                case REG_USER_DATA11: regName = "USER_DATA11"; break;
                case REG_USER_DATA12: regName = "USER_DATA12"; break;
                case REG_USER_DATA13: regName = "USER_DATA13"; break;
                case REG_USER_DATA14: regName = "USER_DATA14"; break;
                case REG_USER_DATA15: regName = "USER_DATA15"; break;
                case REG_USER_DATA16: regName = "USER_DATA16"; break;
                case REG_USER_DATA17: regName = "USER_DATA17"; break;
                case REG_USER_DATA18: regName = "USER_DATA18"; break;
                case REG_USER_DATA19: regName = "USER_DATA19"; break;
                case REG_USER_DATA20: regName = "USER_DATA20"; break;
                case REG_TVGAIN0: regName = "REG_TVGAIN0"; break;
                case REG_TVGAIN1: regName = "REG_TVGAIN1"; break;
                case REG_TVGAIN2: regName = "REG_TVGAIN2"; break;
                case REG_TVGAIN3: regName = "REG_TVGAIN3"; break;
                case REG_TVGAIN4: regName = "REG_TVGAIN4"; break;
                case REG_TVGAIN5: regName = "REG_TVGAIN5"; break;
                case REG_TVGAIN6: regName = "REG_TVGAIN6"; break;
                case REG_INIT_GAIN: regName = "REG_INIT_GAIN"; break;
                case REG_FREQUENCY: regName = "REG_FREQUENCY"; break;
                case REG_DEADTIME: regName = "REG_DEADTIME"; break;
                case REG_PULSE_P1: regName = "REG_PULSE_P1"; break;
                case REG_PULSE_P2: regName = "REG_PULSE_P2"; break;
                case REG_CURR_LIM_P1: regName = "REG_CURR_LIM_P1"; break;
                case REG_CURR_LIM_P2: regName = "REG_CURR_LIM_P2"; break;
                case REG_REC_LENGTH: regName = "REG_REC_LENGTH"; break;
                case REG_FREQ_DIAG: regName = "REG_FREQ_DIAG"; break;
                case REG_SAT_FDIAG_TH: regName = "REG_SAT_FDIAG_TH"; break;
                case REG_FVOLT_DEC: regName = "REG_FVOLT_DEC"; break;
                case REG_DECPL_TEMP: regName = "REG_DECPL_TEMP"; break;
                case REG_DSP_SCALE: regName = "DSP_SCALE"; break;
                case REG_TEMP_TRIM: regName = "REG_TEMP_TRIM"; break;
                case REG_P1_GAIN_CTRL: regName = "REG_P1_GAIN_CTRL"; break;
                case REG_P2_GAIN_CTRL: regName = "REG_P2_GAIN_CTRL"; break;
                case REG_EE_CRC: regName = "EE_CRC"; break;
                default: regName = "UNKNOWN"; break;
            }
            printf("  %s = %d;\n", regName, regValue);
        } else {
            printf("  Address 0x%02X: FAILED to read\n", regAddr);
        }
        HAL_Delay(10);
    }
    printf("EEPROM individual read complete.\n");
}
	
static void PrintThresholdStruct(const Thresholds_t *thr) {
    printf("P1: ");
    for (int i = 0; i < 16; ++i) {
        printf("0x%02X%s", thr->P1_THR[i], (i < 15) ? ", " : "\n");
    }
    printf("P2: ");
    for (int i = 0; i < 16; ++i) {
        printf("0x%02X%s", thr->P2_THR[i], (i < 15) ? ", " : "\n");
    }
	printf("-------\n");
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

static HAL_StatusTypeDef PGA460_SetEEPROMAccess(const uint8_t sensorID, const uint8_t unlock) {
    uint8_t accessByte = unlock ? PGA460_UNLOCK_EEPROM : PGA460_LOCK_EEPROM;
    uint8_t cmd[5] = {PGA460_SYNC, PGA460_CMD_REGISTER_WRITE, REG_EE_CTRL, accessByte, 0x00};
    cmd[4] = PGA460_CalculateChecksum(&cmd[1], 3); // Checksum over CMD, ADDR, DATA
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, cmd, sizeof(cmd), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM %s failed\n", sensorID, unlock ? "unlock" : "lock");
        return HAL_ERROR;
    }
    // Update status flag on success
    EEPROM_Stat = unlock ? 1 : 0;
    return HAL_OK;
}

// Transmit a RUN command without any post-wait/reads.
static HAL_StatusTypeDef PGA460_RunCmd_NoWait(uint8_t sensorID, PGA460_Command_t cmd){
    const uint8_t *frame = NULL;
    switch (cmd) {
        case PGA460_CMD_LISTEN_ONLY_PRESET1:           frame = pga460_cmd_listen_only_preset1; break;
        case PGA460_CMD_BURST_AND_LISTEN_PRESET1:      frame = pga460_cmd_burst_and_listen_preset1; break;
        case PGA460_CMD_LISTEN_ONLY_PRESET2:           frame = pga460_cmd_listen_only_preset2; break;
        /* add others as needed */
        default: return HAL_ERROR;
    }
    return HAL_UART_Transmit(sensors[sensorID].uartPort, frame, PGA_CMD_SIZE, UART_TIMEOUT);
}

// Function to initialize 3x PGA460 sensors
HAL_StatusTypeDef PGA460_Init(uint8_t burnEEPROM) {
    for (uint8_t i = 0; i < ULTRASONIC_SENSOR_COUNT; ++i) {
        // Step 1: Assign UART and transducer config and reset measures
        if (HAL_UART_Init(sensors[i].uartPort) != HAL_OK) {
            DEBUG("Sensor %u: UART init failed\n", i);
            return HAL_ERROR;
        }
        HAL_Delay(100);
        sensors[i].Measures.tof_us  = 0;
        sensors[i].Measures.width     = 0;
        sensors[i].Measures.amplitude = 0;
        // ---- Step 2: Threshold write (uses current sensors[i].Registers.THR) ----
        if (PGA460_SetThresholds(i) != HAL_OK) {
            DEBUG("Sensor %u: Threshold write failed!\n", i);
            return HAL_ERROR;
        }
        // ---- Step 2: EEPROM bulk write (shadow -> device) ----
        if (PGA460_EEPROMBulkWrite(i) != HAL_OK) {
            DEBUG("Sensor %u: EEPROM bulk write failed!\n", i);
            return HAL_ERROR;
        }
				PGA460_ReadAndPrintEEPROM(i);
				if(burnEEPROM) {
					// ---- Step 3: Optional EEPROM Burn ----
					if (PGA460_BurnEEPROM(i) != HAL_OK) {
							DEBUG("Sensor %u: EEPROM burn failed!\n", i);
							return HAL_ERROR;
					}
				}
        // (Optional) Auto-threshold pass
//        if (PGA460_AutoThreshold(i, 10, 0, 12, 16) != HAL_OK) {
//            DEBUG("Sensor %u: Auto THR failed!\n", i);
//            return HAL_ERROR;
//        }
        // ---- Step 3: TVG write (push current TVG from mirror) ----
//        if (PGA460_SetTVG(i, PGA460_GAIN_58_90dB, &sensors[i].Registers.EEData.TVG) != HAL_OK) {
//            DEBUG("Sensor %u: TVG bulk write failed!\n", i);
//            return HAL_ERROR;
//        }
        // ---- Step 5: Diagnostics/status ----
        if (PGA460_CheckStatus(i) != HAL_OK) {
            DEBUG("Sensor %u: Status check failed!\n", i);
            return HAL_ERROR;
        }
        // Frequency (Hz) via system diagnostics (diag=0)
        float diagVal = 0.0f;
        if (PGA460_GetSystemDiagnostics(i, 1, 0, &diagVal) == HAL_OK)
            DEBUG("Sensor %u: Frequency Diagnostic = %.2f kHz\n", i, diagVal);
        if (PGA460_GetSystemDiagnostics(i, 0, 1, &diagVal) == HAL_OK)
            DEBUG("Sensor %u: Decay Period Diagnostic = %.2f us\n", i, diagVal);
        if (PGA460_GetSystemDiagnostics(i, 0, 2, &diagVal) == HAL_OK)
            DEBUG("Sensor %u: Die Temperature = %.2f C\n", i, diagVal);
        if (PGA460_GetSystemDiagnostics(i, 0, 3, &diagVal) == HAL_OK)
            DEBUG("Sensor %u: Noise Level = %.0f\n", i, diagVal);
        // ---- Step 7: Optional Echo Data Dump for verification ----
        uint8_t echoBuf[128];
        if (PGA460_GetEchoDataDump(i, PGA460_CMD_BURST_AND_LISTEN_PRESET1, echoBuf) != HAL_OK) {
            DEBUG("Sensor %u: Echo Data Dump failed!\n", i);
            return HAL_ERROR;
        }
        DEBUG("Sensor %u: Echo Data Dump:\n", i);
        for (uint8_t j = 0; j < 128; ++j) {
            DEBUG("%u%s", echoBuf[j], (j < 127) ? "," : "\n");
        }
        DEBUG("-----\n");
        HAL_Delay(1000);
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
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, tx, sizeof(tx), UART_TIMEOUT) != HAL_OK) return HAL_ERROR;
    // --- RX frame: [DIAG][DATA][CHK] ---
    uint8_t rx[3] = {0};
    if (HAL_UART_Receive(sensors[sensorID].uartPort, rx, sizeof(rx), UART_TIMEOUT) != HAL_OK) return HAL_ERROR;
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
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, tx, sizeof(tx), UART_TIMEOUT) != HAL_OK) return HAL_ERROR;
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

// Seems that this is not working no matter what i do :D
HAL_StatusTypeDef PGA460_EEPROMBulkRead(uint8_t sensorID) {
    // --- Step 1: Send EEPROM Bulk Read command ---
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, pga460_cmd_eeprom_bulk_read, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Bulk Read Command Failed!\n", sensorID);
        return HAL_ERROR;
    }
    HAL_Delay(5); //10 device needs a short time to prepare reply
    // --- Step 2: Receive response ---
    uint8_t rx[45] = {0}; // [DIAG][EEPROM bytes 0x00..0x2B]
    if (HAL_UART_Receive(sensors[sensorID].uartPort, rx, sizeof(rx), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Bulk Read Failed!\n", (int)sensorID);
        return HAL_ERROR;
    }
    // Detect idle flood (all 0xFF)
//    uint8_t idle = 1;
//    for (uint32_t i = 0; i < sizeof(rx); i++) {
//        if (rx[i] != 0xFF) { idle = 0u; break; }
//    }
//    if (idle) {
//        DEBUG("Sensor %d: EEPROM Read response was idle (0xFF)\n", (int)sensorID);
//        return HAL_ERROR;
//    }
    // --- Step 3: Copy EEPROM content (skip rx[0] = DIAG) ---
    memcpy(&sensors[sensorID].Registers.EEData, &rx[1], sizeof(EEImage_t));
    // --- Step 4: Validate EE_CRC ---
    const uint8_t *ee = (const uint8_t *)&sensors[sensorID].Registers.EEData;
    uint8_t expected_crc = PGA460_CalculateEEPROM_CRC(ee, 43); // bytes 0..42
    if (ee[43] != expected_crc) {
        DEBUG("Sensor %d: EEPROM CRC mismatch! Expected 0x%02X, Got 0x%02X\n", (int)sensorID, expected_crc, ee[43]);
        return HAL_ERROR;
    }
    DEBUG("Sensor %d: EEPROM Bulk Read Successful.\n", (int)sensorID);
    return HAL_OK;
}

// EEPROM bulk write: send 44-byte EEPROM image (0x00..0x2B) to device shadow
HAL_StatusTypeDef PGA460_EEPROMBulkWrite(uint8_t sensorID) {
    // Step 1: Unlock EEPROM if needed
    if (EEPROM_Stat == 0) {
        if (PGA460_SetEEPROMAccess(sensorID, 1) != HAL_OK) {
            DEBUG("Sensor %d: EEPROM unlock failed before bulk write\n", sensorID);
            return HAL_ERROR;
        }
    }
    // Step 2: Construct UART frame
	uint8_t frame[46] = {0};
    frame[0] = PGA460_SYNC;
    frame[1] = PGA460_CMD_EEPROM_BULK_WRITE;
    memcpy(&frame[2], &sensors[sensorID].Registers.EEData, 44);  // 44 bytes
    frame[45] = PGA460_CalculateChecksum(&frame[1], 44);  // Checksum over CMD + EEPROM bytes
    // Step 4: Transmit
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, frame, sizeof(frame), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Bulk Write Failed!\n", sensorID);
        return HAL_ERROR;
    }
    HAL_Delay(100);  // Wait for EEPROM write
    if (PGA460_SetEEPROMAccess(sensorID, 0) != HAL_OK) {
		DEBUG("Sensor %d: EEPROM lock failed after bulk write\n", sensorID);
		return HAL_ERROR;
	}
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_BurnEEPROM(uint8_t sensorID) {
    // Step 1: Unlock EEPROM (EE_UNLCK=0xD, EE_PRGM=0) => 0x68
    if (PGA460_RegisterWrite(sensorID, REG_EE_CTRL, PGA460_UNLOCK_EEPROM) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Burn Step 1 (0x68) Failed!\n", sensorID);
        return HAL_ERROR;
    }
    HAL_Delay(1); // short delay per datasheet
    // Step 2: Trigger EEPROM burn (EE_UNLCK=0xD, EE_PRGM=1) => 0x69
    if (PGA460_RegisterWrite(sensorID, REG_EE_CTRL, PGA460_LOCK_EEPROM) != HAL_OK) {
        DEBUG("Sensor %d: EEPROM Burn Step 2 (0x69) Failed!\n", sensorID);
        return HAL_ERROR;
    }
    // EEPROM programming time TYP 600ms
    HAL_Delay(650);
    // Step 3: Read back EE_CTRL to verify
    uint8_t ctrlVal = 0;
    if (PGA460_RegisterRead(sensorID, REG_EE_CTRL, &ctrlVal) != HAL_OK) {
        DEBUG("Sensor %d: Failed to read EE_CTRL after burn!\n", sensorID);
        return HAL_ERROR;
    }
    sensors[sensorID].Registers.eeCtrl.Val.Value = ctrlVal;
    // Step 4: Check EE_PRGM_OK (bit 2)
    if (sensors[sensorID].Registers.eeCtrl.Val.BitField.EE_PRGM_OK) {
        if(PGA460_RegisterWrite(sensorID, REG_EE_CTRL, 0x00) != HAL_OK)
			return HAL_ERROR;
		return HAL_OK;
    } else {
        DEBUG("Sensor %d: EEPROM Burn Failed (EE_CTRL = 0x%02X)\n", sensorID, ctrlVal);
        return HAL_ERROR;
    }
}

// TVG bulk read: fetch TVGAIN0..TVGAIN6 into sensors[sid].Registers.EEData.TVG
HAL_StatusTypeDef PGA460_GetTVG(uint8_t sensorID) {
    // Send prebuilt command [SYNC][CMD][CHK]
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, pga460_cmd_tvg_bulk_read, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    // --- Step 2: Receive DIAG + 7 TVG bytes ---
    // RX: [DIAG][TVGAIN0..TVGAIN6] => 1 + 7 = 8 bytes
    uint8_t rx[8] = {0};
    if (HAL_UART_Receive(sensors[sensorID].uartPort, rx, sizeof(rx), UART_TIMEOUT) != HAL_OK) {
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

// Set AFE gain range (DECPL_TEMP[7:6]) and write TVG (TVGAIN0..6) via bulk write.
HAL_StatusTypeDef PGA460_SetTVG(uint8_t sensorID, PGA460_GainRange_t gain_range, const TVG_t *tvg) {
    // 1) Update DECPL_TEMP[7:6] = AFE_GAIN_RNG, preserve other bits.
    sensors[sensorID].Registers.EEData.Sett.DECPL_TEMP = (sensors[sensorID].Registers.EEData.Sett.DECPL_TEMP & 0x3Fu) | gain_range;
    if (PGA460_RegisterWrite(sensorID, REG_DECPL_TEMP, sensors[sensorID].Registers.EEData.Sett.DECPL_TEMP) != HAL_OK) {
        return HAL_ERROR;
    }
    // 2) TVG bulk write: [SYNC][CMD=0x0E][7 TVG bytes][CHK]
    uint8_t frame[10];
    frame[0] = PGA460_SYNC;
    frame[1] = PGA460_CMD_TVG_BULK_WRITE;
    memcpy(&frame[2], tvg, sizeof(TVG_t));                        // 7 bytes payload
    frame[9] = PGA460_CalculateChecksum(&frame[1], 1 + sizeof(TVG_t)); // checksum over CMD+DATA
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, frame, sizeof(frame), UART_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    // Mirror locally on success
    memcpy(&sensors[sensorID].Registers.EEData.TVG, tvg, sizeof(TVG_t));
    HAL_Delay(70);
    return HAL_OK;
}

// Read P1/P2 threshold maps + CRC using bulk command into sensors[sid].Registers.THR
HAL_StatusTypeDef PGA460_GetThresholds(uint8_t sensorID) {
    // 1) Send prebuilt command [SYNC][CMD][CHK]
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, pga460_cmd_threshold_bulk_read, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    // 2) Receive DIAG (1) + THR block (33) = 34 bytes total
    uint8_t rx[34] = {0};  // rx[0]=DIAG, rx[1..32]=P1/P2 (32 bytes), rx[33]=THR_CRC
    if (HAL_UART_Receive(sensors[sensorID].uartPort, rx, sizeof(rx), UART_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    // 3) Copy 33 bytes (P1_THR[16], P2_THR[16], THR_CRC) into mirror (skip DIAG)
    memcpy(&sensors[sensorID].Registers.THR, &rx[1], sizeof(Thresholds_t));
    return HAL_OK;
}

// Write current thresholds from sensors[sid].Registers.THR (no presets)
HAL_StatusTypeDef PGA460_SetThresholds(uint8_t sensorID) {
    // Build a 33-byte image (P1[16] + P2[16] + CRC) without mutating shadow yet
    uint8_t frame[35] = {0};
    frame[0] = PGA460_SYNC;
    frame[1] = PGA460_CMD_THRESHOLD_BULK_WRITE;
    memcpy(&frame[2], &sensors[sensorID].Registers.THR, 32);
    // Frame: [SYNC][CMD=0x10][33 bytes][CHK] => total 36 bytes
    frame[34] = PGA460_CalculateChecksum(&frame[1], 33); // checksum over CMD + 33 data bytes

    if (HAL_UART_Transmit(sensors[sensorID].uartPort, frame, sizeof(frame), UART_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(70);
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_UltrasonicCmd(const uint8_t sensorID, const PGA460_Command_t cmd) {
    uint8_t frame[4];
    uint16_t len = 0;
    frame[0] = PGA460_SYNC;         // SYNC
    frame[1] = (uint8_t)cmd;        // CMD
    switch (cmd) {
        // 4-byte frames: [SYNC][CMD][OBJ][CHK]
        case PGA460_CMD_BURST_AND_LISTEN_PRESET1:
        case PGA460_CMD_BURST_AND_LISTEN_PRESET2:
        case PGA460_CMD_LISTEN_ONLY_PRESET1:
        case PGA460_CMD_LISTEN_ONLY_PRESET2:
            frame[2] = PGA_OBJECTS_TRACKED;                 // OBJ
            frame[3] = PGA460_CalculateChecksum(&frame[1], 2); // CMD+OBJ
            len = 4;
            break;
        // 3-byte frames: [SYNC][CMD][CHK]
        case PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT:
        case PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT:
        case PGA460_CMD_TEMP_AND_NOISE_RESULT:
        case PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP:
				case PGA460_CMD_SYSTEM_DIAGNOSTICS:
            frame[2] = PGA460_CalculateChecksum(&frame[1], 1); // CMD
            len = 3;
            break;

        default:
            DEBUG("Sensor %d: Unsupported CMD 0x%02X\n", sensorID, (unsigned)cmd);
            return HAL_ERROR;
    }
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, frame, len, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Ultrasonic TX failed (CMD=0x%02X)\n", sensorID, frame[1]);
        return HAL_ERROR;
    }
    // Only burst/listen & listen-only start a capture; give them time to run.
    if (len == 4) {
        HAL_Delay(70); // up to ~65 ms capture time
    }
    return HAL_OK;
}

// Run auto-threshold on P1 then P2, commit to device
HAL_StatusTypeDef PGA460_AutoThreshold(uint8_t sensorID, uint8_t noiseMargin, uint8_t windowIndex, uint8_t autoMax, uint8_t loops) {
    Thresholds_t *thr = &sensors[sensorID].Registers.THR;
    HAL_StatusTypeDef status;
    // TI's approach is to read all 32 bytes of thresholds first to preserve the other preset's values.
    if (PGA460_GetThresholds(sensorID) != HAL_OK)
		return status;
    // Run auto-tuning for Preset 1 and then Preset 2
    for (uint8_t preset = 1; preset <= 2; preset++) {
        PGA460_Command_t cmd = (preset == 1) ? PGA460_CMD_BURST_AND_LISTEN_PRESET1 : PGA460_CMD_BURST_AND_LISTEN_PRESET2;
        uint8_t *thrStart = (preset == 1) ? thr->P1_THR : thr->P2_THR;
        uint8_t echoSum[128] = {0};
        uint8_t echoBuf[128];
        uint8_t maxVals[12] = {0};
        uint8_t eddMarkers[13] = {0};
        uint16_t recTime_us = 0;
        int8_t thrOffset = 0;
        // 1) Read REC_LENGTH and convert to time ---
        uint8_t rec = 0;
        if (PGA460_RegisterRead(sensorID, REG_REC_LENGTH, &rec) != HAL_OK)
			return HAL_ERROR;
        const uint8_t recNibble = (preset == 1) ? ((rec >> 4) & 0x0F) : (rec & 0x0F);
        recTime_us = (uint16_t)(recNibble + 1) * 4096;
        // 2) Set threshold times based on windowIndex ---
        uint16_t thrTime_us = 0;
        eddMarkers[0] = 0;
        if (windowIndex < 16) {
            // Overwrite existing threshold times with the new windowIndex
            for (uint8_t i = 0; i < 6; i++) {
                thrStart[i] = (uint8_t)((windowIndex << 4) | (windowIndex & 0x0F));
            }
        }
        // Calculate EDD markers based on the chosen/existing threshold times
        uint8_t numPoints = 0;
        for (uint8_t i = 0; i < 12; i++) {
            uint8_t index_nibble = 0;
            if (i < 1) {
                index_nibble = (thrStart[0] >> 4) & 0x0F;
            } else if (i < 2) {
                index_nibble = thrStart[0] & 0x0F;
            } else {
                index_nibble = (thrStart[i/2] >> (4 * ((i+1)%2))) & 0x0F;
            }
            thrTime_us += time_lookup[index_nibble];
            uint32_t marker = (uint32_t)thrTime_us * 128 / recTime_us;
            eddMarkers[i+1] = (uint8_t)((marker > 127) ? 127 : marker);
            if (thrTime_us >= recTime_us) {
                numPoints = i + 1;
                break;
            }
        }
        if (numPoints == 0)
			numPoints = 12;
        // 3) Average N loops for current preset ---
        for (uint8_t l = 0; l < loops; l++) {
            if (PGA460_GetEchoDataDump(sensorID, cmd, echoBuf) != HAL_OK)
				return HAL_ERROR;
            for (uint8_t i = 0; i < 128; i++)
				echoSum[i] += echoBuf[i];
        }
        for (uint8_t i = 0; i < 128; i++)
			echoSum[i] = (uint8_t)(echoSum[i] / loops);
        // 4) Window maxima and quantization per TI logic ---
        memset(maxVals, 0, sizeof(maxVals));
        if (autoMax > 12)
			autoMax = 12;
        for (uint8_t p = 0; p < autoMax; p++) {
            uint8_t start = eddMarkers[p];
            uint8_t stop = eddMarkers[p+1];
            for (uint8_t i = start; i < stop; i++) {
                if (echoSum[i] > maxVals[p])
					maxVals[p] = echoSum[i];
            }
        }
        for (uint8_t i = 0; i < autoMax; i++) {
            if (i < 8) {
                while ((maxVals[i] % 8 != 0) && (maxVals[i] < 248))
					maxVals[i]++;
            }
            uint16_t v = (uint16_t)maxVals[i] + (uint16_t)noiseMargin;
            if (v >= 248) {
                v = 248;
                if (noiseMargin >= 7) {
                    thrOffset = 7;
                } else {
                    thrOffset = noiseMargin;
                }
            }
            maxVals[i] = (uint8_t)v;
        }
        for (uint8_t i = 0; i < autoMax && i < 8; i++) {
            maxVals[i] = (uint8_t)(maxVals[i] / 8);
        }
        // 5) Pack 16-byte threshold block using read-modify-write ---
        if (autoMax > 0)
			thrStart[6] = (thrStart[6] & ~0xF8) | (maxVals[0] << 3);
        if (autoMax > 1) {
            thrStart[6] = (thrStart[6] & ~0x07) | (maxVals[1] >> 2);
            thrStart[7] = (thrStart[7] & ~0xC0) | (maxVals[1] << 6);
        }
        if (autoMax > 2)
			thrStart[7] = (thrStart[7] & ~0x3E) | (maxVals[2] << 1);
        if (autoMax > 3) {
            thrStart[7] = (thrStart[7] & ~0x01) | (maxVals[3] >> 4);
            thrStart[8] = (thrStart[8] & ~0xF0) | (maxVals[3] << 4);
        }
        if (autoMax > 4) {
            thrStart[8] = (thrStart[8] & ~0x0F) | (maxVals[4] >> 1);
            thrStart[9] = (thrStart[9] & ~0x80) | (maxVals[4] << 7);
        }
        if (autoMax > 5)
			thrStart[9] = (thrStart[9] & ~0x7C) | (maxVals[5] << 2);
        if (autoMax > 6) {
            thrStart[9] = (thrStart[9] & ~0x03) | (maxVals[6] >> 3);
            thrStart[10] = (thrStart[10] & ~0xE0) | (maxVals[6] << 5);
        }
        if (autoMax > 7)
			thrStart[10] = (thrStart[10] & ~0x1F) | (maxVals[7]);
        if (autoMax > 8)
			thrStart[11] = maxVals[8];
        if (autoMax > 9)
			thrStart[12] = maxVals[9];
        if (autoMax > 10)
			thrStart[13] = maxVals[10];
        if (autoMax > 11)
			thrStart[14] = maxVals[11];
        if (thrOffset != 0)
			thrStart[15] = (uint8_t)(thrOffset & 0x0F);
        DEBUG("Sensor %d: Final Threshold Block (P%u):\n", sensorID, preset);
        for (uint8_t i = 0; i < 16; i++)
			DEBUG("%u%s", thrStart[i], (i == 15) ? "" : ", ");
        DEBUG("\n");
    }
    // 6) Push to device via threshold bulk write ---
    if (PGA460_SetThresholds(sensorID) != HAL_OK) {
        DEBUG("Sensor %d: Failed to apply new thresholds!\n", sensorID);
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_GetUltrasonicMeasurement(const uint8_t sensorID) {
    uint8_t localBuffer[PGA_OBJ_DATA_SIZE] = {0};
    // Step 1: Send UMR command (0x05)
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, pga460_cmd_ultrasonic_measurement_result, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Measurement Result Request Failed!\n", sensorID);
        return HAL_ERROR;
    }
    // Step 2: Receive result: [2B header][N * 4B objects]
    if (HAL_UART_Receive(sensors[sensorID].uartPort, localBuffer, sizeof(localBuffer), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Measurement Result Retrieval Failed!\n", sensorID);
        return HAL_ERROR;
    }
    // Step 3: Parse each object; store first valid into sensors[sid].Measures
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t stored = 0;
    for (uint8_t i = 0; i < PGA_OBJECTS_TRACKED; i++) {
        const uint8_t base = 2 + (i * 4);  // skip 2-byte diagnostic header
        const uint16_t rawTOF   = ((uint16_t)localBuffer[base] << 8) | localBuffer[base + 1];
        const uint8_t  widthRaw = localBuffer[base + 2];
        const uint8_t  amplitude = localBuffer[base + 3];
        DEBUG("Sensor %d: Raw TOF = %04X\n", sensorID, rawTOF);
        if (rawTOF > 0 && rawTOF != 0xFFFF) {
            // distance = (TOF/2)*1e-6*soundSpeed + burstOffset
            // PULSE_P1 cycles @ frequency[sensorID]
            //float burstOffset_m = (soundSpeed * ((float)sensors[sensorID].Registers.EEData.Sett.PULSE_P1 / frequency[sensorID])) * 0.5f;
            float distance_m = ((float)rawTOF * 0.5f) * 1e-6f * soundSpeed;// + burstOffset_m;
            uint16_t width_us = (uint16_t)widthRaw * 16u;
            if (!stored) {
                sensors[sensorID].Measures.tof_us  = rawTOF;
                sensors[sensorID].Measures.width     = (uint8_t)((width_us > 255) ? 255 : width_us); // keep field type
                sensors[sensorID].Measures.amplitude = amplitude;
                stored = 1;
            }
            DEBUG("Sensor %d: Obj %u -> %.3f m, Width = %u us, Amplitude = %u\n",sensorID, (unsigned)(i + 1), distance_m, width_us, amplitude);
            status = HAL_OK; // at least one valid result
        } else {
            DEBUG("Sensor %d: Obj %u -> Invalid (TOF = 0x%04X)\n", sensorID, (unsigned)(i + 1), rawTOF);
        }
    }
    return status;
}

HAL_StatusTypeDef PGA460_GetEchoDataDump(uint8_t sensorID, uint8_t preset, uint8_t *echoOut) {
    HAL_StatusTypeDef ret = HAL_OK;
    uint8_t response[ECHO_DATA_TOTAL_BYTES] = {0}; // 2-byte header + 128 bins
    // 1) Enable Echo Data Dump Mode (EE_CTRL.DATADUMP_EN = 1)
    if (PGA460_RegisterWrite(sensorID, REG_EE_CTRL, ECHO_DATA_DUMP_ENABLE) != HAL_OK) {
        DEBUG("Sensor %d: Failed to enable Echo Data Dump mode!\n", sensorID);
        return HAL_ERROR;
    }
    HAL_Delay(10);
    // 2) Trigger the requested preset (burst+listen or listen-only)
    if (PGA460_UltrasonicCmd(sensorID, (PGA460_Command_t)preset) != HAL_OK) {
        DEBUG("Sensor %d: Echo trigger preset failed!\n", sensorID);
        ret = HAL_ERROR;
        goto cleanup_disable_dump;
    }
    HAL_Delay(70); // allow full capture (~65 ms)
    // 3) Request Bulk Echo Data Dump (CMD = 0x07)
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, pga460_cmd_transducer_echo_data_dump, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Echo Data Dump request TX failed!\n", sensorID);
        ret = HAL_ERROR;
        goto cleanup_disable_dump;
    }
    // 4) Receive Echo Data Dump (2-byte header + 128 samples)
    if (HAL_UART_Receive(sensors[sensorID].uartPort, response, sizeof(response), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Echo Data Dump RX failed!\n", sensorID);
        ret = HAL_ERROR;
        goto cleanup_disable_dump;
    }
    // 5) Copy only the 128 echo bins (skip the 2-byte header)
    memcpy(echoOut, &response[2], 128);
cleanup_disable_dump:
    // 6) Disable Echo Data Dump Mode (best-effort)
    // The previous status should be preserved. The return value should not be overwritten.
    if (PGA460_RegisterWrite(sensorID, REG_EE_CTRL, ECHO_DATA_DUMP_DISABLE) != HAL_OK) {
        DEBUG("Sensor %d: Failed to disable Echo Data Dump mode!\n", sensorID);
    }
    // Return the status from the main operation, not the cleanup operation.
    return ret;
}

HAL_StatusTypeDef PGA460_SetTemperatureOffset(uint8_t sensorID, float externalTempC) {
    // 1) Read internal temperature
    float internalTempC = PGA460_ReadTemperatureOrNoise(sensorID, PGA460_CMD_GET_TEMP);
    if (internalTempC == PGA460_TEMP_ERR) {
        DEBUG("Sensor %d: Failed to read internal temperature!\n", sensorID);
        return HAL_ERROR;
    }
    DEBUG("Sensor %d: Internal Temperature = %.2f C\n", sensorID, internalTempC);
    // 2) Compute signed offset (external - internal), clamp to [-64, +63]
    int8_t offset = (int8_t)roundf(externalTempC - internalTempC);
    if (offset >  63) offset =  63;
    if (offset < -64) offset = -64;
    // 3) Read current TEMP_TRIM (0x28)
    uint8_t tempTrim = 0;
    if (PGA460_RegisterRead(sensorID, REG_TEMP_TRIM, &tempTrim) != HAL_OK) {
        DEBUG("Sensor %d: Failed to read TEMP_TRIM register!\n", sensorID);
        return HAL_ERROR;
    }
    DEBUG("Sensor %d: Original TEMP_TRIM = 0x%02X\n", sensorID, tempTrim);
    // 4) Build sign-magnitude byte: bit7 = sign, bits6:0 = magnitude
    uint8_t magnitude = (uint8_t)((offset < 0) ? -offset : offset);
    uint8_t signedOffset = ((offset < 0) ? 0x80u : 0x00u) | (magnitude & 0x7Fu);
    // If TEMP_TRIM has no reserved bits, write the whole byte:
    tempTrim = signedOffset;
    // 5) Write updated TEMP_TRIM
    if (PGA460_RegisterWrite(sensorID, REG_TEMP_TRIM, tempTrim) != HAL_OK) {
        DEBUG("Sensor %d: Failed to write TEMP_TRIM register!\n", sensorID);
        return HAL_ERROR;
    }
    // Mirror locally on success
    sensors[sensorID].Registers.EEData.Sett.TEMP_TRIM = tempTrim;
    DEBUG("Sensor %d: New TEMP_TRIM = 0x%02X (Offset: %+d C)\n", sensorID, tempTrim, offset);
    return HAL_OK;
}

HAL_StatusTypeDef PGA460_GetSystemDiagnostics(const uint8_t sensorID, const uint8_t run, const uint8_t diag, float *diagResult) {
    uint8_t response[4] = {0};
    // Optional: start a capture using P1 before running diagnostics
    if (run) {
        if (PGA460_UltrasonicCmd(sensorID, PGA460_CMD_BURST_AND_LISTEN_PRESET1) != HAL_OK) {
            DEBUG("Sensor %d: Burst-and-Listen Command Failed!\n", sensorID);
            return HAL_ERROR;
        }
        HAL_Delay(100); // allow measurement to complete
    }
    // Send System Diagnostics (0x08) — 3B command [SYNC][CMD][CHK]
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, pga460_cmd_system_diagnostics, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: System Diagnostics Request Failed!\n", sensorID);
        return HAL_ERROR;
    }
    // Receive 4B response: [DIAG][FREQ_TICKS][DECAY_TICKS][PCHK]
    if (HAL_UART_Receive(sensors[sensorID].uartPort, response, sizeof(response), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: System Diagnostics Retrieval Failed!\n", sensorID);
        return HAL_ERROR;
    }
    // Special cases handled via temp/noise helper
    if (diag == 2) {  // Temperature (°C)
        *diagResult = PGA460_ReadTemperatureOrNoise(sensorID, PGA460_CMD_GET_TEMP);
        if (*diagResult == PGA460_TEMP_ERR) {
            DEBUG("Sensor %d: Failed to Retrieve Temperature!\n", sensorID);
            return HAL_ERROR;
        }
        return HAL_OK;
    }
    if (diag == 3) {  // Noise (raw 8-bit)
        *diagResult = PGA460_ReadTemperatureOrNoise(sensorID, PGA460_CMD_GET_NOISE);
        if (*diagResult == PGA460_TEMP_ERR) {
            DEBUG("Sensor %d: Failed to Retrieve Noise!\n", sensorID);
            return HAL_ERROR;
        }
        return HAL_OK;
    }
    // Standard diagnostics
    switch (diag) {
        case 0: { // Frequency (Hz) from ticks of 0.5 µs
            const uint8_t ticks = response[1];
            if (ticks == 0) {
                DEBUG("Sensor %d: Invalid Frequency Value (0 ticks)!\n", sensorID);
                return HAL_ERROR;
            }
            *diagResult = 1.0f / (ticks * 0.5e-6f); // Hz
            break;
        }
        case 1: { // Decay period (µs), tick = 16 µs
            *diagResult = (float)response[2] * 16.0f; // µs
            break;
        }
        default:
            DEBUG("Sensor %d: Invalid Diagnostic Type (%u)!\n", sensorID, (unsigned)diag);
            return HAL_ERROR;
    }
    //DEBUG("Sensor %d: Diagnostic [%u] Result: %.2f\n", sensorID, (unsigned)diag, *diagResult);
    return HAL_OK;
}

float PGA460_ReadTemperatureOrNoise(const uint8_t sensorID, const PGA460_CmdType_t mode) {
    const uint8_t isNoise = (mode == PGA460_CMD_GET_NOISE);
    float result = PGA460_TEMP_ERR;  // default invalid
    // Step 1: Request Temperature or Noise Measurement
    // [SYNC][CMD=0x04][SEL(0=temp,1=noise)][CHK(CMD+SEL)]
    uint8_t tx[4];
    tx[0] = PGA460_SYNC; // SYNC
    tx[1] = PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT; // CMD
    tx[2] = isNoise ? 1u : 0u;                        // selector
    tx[3] = PGA460_CalculateChecksum(&tx[1], 2);      // over CMD+SEL
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, tx, sizeof(tx), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Temp/Noise Measurement Command Failed!\n", sensorID);
        return result;
    }
    HAL_Delay(15);  // small settle for measurement to complete
    // Step 2: Send command to read the result (fixed 3-byte command)
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, pga460_cmd_temp_and_noise_result, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Result Request Command Failed!\n", sensorID);
        return result;
    }
    // Step 3: Read 4-byte result frame: [DIAG][TEMP][NOISE][PCHK]
    uint8_t rx[4] = {0};
    if (HAL_UART_Receive(sensors[sensorID].uartPort, rx, sizeof(rx), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Failed to Receive Temp/Noise Data!\n", sensorID);
        return result;
    }
    // Step 4: Parse result
    if (!isNoise) {
        result = ((float)rx[1] - 64.0f) / 1.5f;  // Temperature in °C
        //DEBUG("Sensor %d: Temperature (C) = %.2f\n", sensorID, result);
    } else {
        result = (float)rx[2];                   // Noise level (raw 8-bit)
        //DEBUG("Sensor %d: Noise Level = %.2f\n", sensorID, result);
    }
    return result;
}

// ============================================================================
// Helper functions for PGA460_AutoTune  (dedup + centralized THR builder)
// ============================================================================

// --- tiny utils ---
static inline uint8_t u8_clamp(int v, int lo, int hi) {
    if (v < lo)
		return (uint8_t)lo;
    if (v > hi)
		return (uint8_t)hi;
    return (uint8_t)v;
}

static uint8_t median5_u8(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e) {
    uint8_t s[5] = {a, b, c, d, e};
    for (int i = 0; i < 5; i++) {
        for (int j = i + 1; j < 5; j++) {
            if (s[j] < s[i]) {
				uint8_t t = s[i];
				s[i] = s[j];
				s[j] = t;
			}
        }
    }
    return s[2];
}

// Uses your file-scope: static const uint16_t time_lookup[16] = {...};
static uint8_t time_code_from_us(uint16_t us) {
    uint8_t best_code = 0;
    uint32_t best_err = 0xFFFFFFFF;
    for (uint8_t i = 0; i < 16; i++) {
        uint32_t lut = (uint32_t)time_lookup[i];
        uint32_t err = (lut > us) ? (lut - us) : (us - lut);
        if (err < best_err) {
			best_err = err;
			best_code = i;
		}
    }
    return best_code;
}

static uint8_t tvg_code_from_factor(float factor) {
    if (factor < 0.0625f)
		factor = 0.0625f;
    if (factor > 16.0f)
		factor = 16.0f;
    int code = (int)lroundf(16.0f * log2f(factor));   // 0..63
    return u8_clamp(code, 0, 63);
}

// ============================================================================
// Centralized THR builder (P1/P2): sets windows, builds markers, quantizes, packs
// Call this anywhere you need to compute a 16-byte THR block from a 128-bin echo.
// ============================================================================

static void _thr_set_uniform_windows(uint8_t *thrStart, uint8_t windowIndex) {
    for (uint8_t i = 0; i < 6; i++)
        thrStart[i] = (uint8_t)((windowIndex << 4) | (windowIndex & 0x0F));
}

static uint8_t _thr_build_markers(const uint8_t *thrStart, uint16_t recTime_us, uint8_t markers[13]) {
    uint16_t acc_us = 0;
    markers[0] = 0;
    uint8_t i;
    for (i = 0; i < 12; i++) {
        // nibble order: T1_hi, T1_lo, T2_hi, T2_lo, ...
        const uint8_t code = (i == 0) ? ((thrStart[0] >> 4) & 0x0F) : (i == 1) ? (thrStart[0] & 0x0F) : ((thrStart[i/2] >> (4 * ((i+1) & 1))) & 0x0F);
        acc_us += time_lookup[code];
        uint32_t m = (uint32_t)acc_us * 128 / recTime_us;
        markers[i+1] = (m > 127) ? 127 : (uint8_t)m;
        if (acc_us >= recTime_us) {
			i++; 
			break;
		}
    }
    // i = number of windows produced (<=12)
    return i;
}

static void _thr_pack_p1(uint8_t *thrStart, const uint8_t q[12], uint8_t n, int8_t thrOffset) {
    if (n > 12) n = 12;
    if (n > 0)
		thrStart[6] = (thrStart[6] & ~0xF8) | ((q[0] & 0x1F) << 3);
    if (n > 1) {
		thrStart[6]  = (thrStart[6] & ~0x07) | ((q[1] >> 2) & 0x07);
        thrStart[7]  = (thrStart[7]  & ~0xC0) | ((q[1] & 0x03) << 6);
	}
    if (n > 2)
		thrStart[7]  = (thrStart[7]  & ~0x3E) | ((q[2] & 0x1F) << 1);
    if (n > 3) {
		thrStart[7]  = (thrStart[7]  & ~0x01) | ((q[3] >> 4) & 0x01);
		thrStart[8]  = (thrStart[8]  & ~0xF0) | ((q[3] & 0x0F) << 4);
	}
    if (n > 4) {
		thrStart[8]  = (thrStart[8]  & ~0x0F) | ((q[4] >> 1) & 0x0F);
        thrStart[9]  = (thrStart[9]  & ~0x80) | ((q[4] & 0x01) << 7);
	}
    if (n > 5)
		thrStart[9]  = (thrStart[9]  & ~0x7C) | ((q[5] & 0x1F) << 2);
    if (n > 6) {
		thrStart[9]  = (thrStart[9]  & ~0x03) | ((q[6] >> 3) & 0x03);
        thrStart[10] = (thrStart[10] & ~0xE0) | ((q[6] & 0x07) << 5);
	}
    if (n > 7)
		thrStart[10] = (thrStart[10] & ~0x1F) | (q[7] & 0x1F);
    if (n > 8)
		thrStart[11] = q[8];
    if (n > 9)
		thrStart[12] = q[9];
    if (n > 10)
		thrStart[13] = q[10];
    if (n > 11)
		thrStart[14] = q[11];
    if (thrOffset)
		thrStart[15] = (uint8_t)(thrOffset & 0x0F);
}

// 16B P1 or P2 block
// 0..15 -> uniform windows
// record window (this preset)
// averaged EDD
// 1..12
// ADC counts
static void PGA460_UpdateThresholdBlock(uint8_t *thrStart, uint8_t windowIndex, uint16_t recTime_us, const uint8_t  echo[128], uint8_t autoMax, uint8_t noiseMargin) {
    if (recTime_us == 0)
		recTime_us = 4096;
    if (autoMax == 0)
		autoMax = 12;
    if (autoMax > 12)
		autoMax = 12;
    if (windowIndex < 16) {
        _thr_set_uniform_windows(thrStart, windowIndex);
    }
    uint8_t markers[13];
    (void)_thr_build_markers(thrStart, recTime_us, markers);
    // window maxima
    uint8_t maxVals[12] = {0};
    for (uint8_t p = 0; p < autoMax; p++) {
        uint8_t s = markers[p], e = markers[p+1];
        if (e <= s)
			e = (uint8_t)(s + 1);
        if (e > 128)
			e = 128;
        for (uint8_t i = s; i < e; i++)
			if (echo[i] > maxVals[p])
				maxVals[p] = echo[i];
    }
    // quantize + margin
    int8_t thrOffset = 0;
    for (uint8_t i = 0; i < autoMax; i++) {
        if (i < 8)
			while ((maxVals[i] % 8) && (maxVals[i] < 248))
				maxVals[i]++;
        uint16_t v = (uint16_t)maxVals[i] + noiseMargin;
        if (v >= 248) {
			v = 248;
			thrOffset = (noiseMargin >= 7) ? 7 : (int8_t)noiseMargin;
		}
        maxVals[i] = (uint8_t)v;
    }
    for (uint8_t i = 0; i < autoMax && i < 8; i++)
		maxVals[i] = (uint8_t)(maxVals[i] / 8);
    // pack into P1/P2 layout (same packing bytes; you pass the proper block)
    _thr_pack_p1(thrStart, maxVals, autoMax, thrOffset);
}

// ============================================================================
// Main auto-tuning function to calibrate thresholds (P1 & P2) and TVG
// ============================================================================
HAL_StatusTypeDef PGA460_AutoTune(uint8_t sensorID, uint8_t noiseMargin, uint8_t windowIndex, uint8_t autoMax, uint8_t loops, uint8_t targetAmp) {
    // 1) Collect a loop-averaged EDD using P1 (reused for P2 timing too)
    if (loops == 0)
		loops = 8;
    uint8_t echoAvg[128] = {0};
	uint8_t echoBuf[128] = {0};
    for (uint8_t l = 0; l < loops; l++) {
        if (PGA460_GetEchoDataDump(sensorID, PGA460_CMD_BURST_AND_LISTEN_PRESET1, echoBuf) != HAL_OK) {
            DEBUG("Sensor %u: Failed to get echo data dump for auto-tuning.\n", sensorID);
            return HAL_ERROR;
        }
        for (int i = 0; i < 128; i++)
			echoAvg[i] += echoBuf[i];
    }
    for (int i = 0; i < 128; i++)
		echoAvg[i] = (uint8_t)(echoAvg[i] / loops);
    // Read current THR image once (so we preserve the other preset each time)
    if (PGA460_GetThresholds(sensorID) != HAL_OK)
		return HAL_ERROR;
    // 2) Build thresholds for P1 and P2 using the centralized helper
    uint8_t recReg = 0;
    if (PGA460_RegisterRead(sensorID, REG_REC_LENGTH, &recReg) != HAL_OK)
		return HAL_ERROR;
    const uint8_t p1_n = (recReg >> 4) & 0x0F;
    const uint8_t p2_n = recReg & 0x0F;
    const uint16_t recTime_us_P1 = (uint16_t)(p1_n + 1) * 4096;
    const uint16_t recTime_us_P2 = (uint16_t)(p2_n + 1) * 4096;
    // P1
    PGA460_UpdateThresholdBlock(sensors[sensorID].Registers.THR.P1_THR, windowIndex, recTime_us_P1, echoAvg, autoMax, noiseMargin);
    // P2 (reuse same echoAvg; different record time)
    PGA460_UpdateThresholdBlock(sensors[sensorID].Registers.THR.P2_THR, windowIndex, recTime_us_P2, echoAvg, autoMax, noiseMargin);
    if (PGA460_SetThresholds(sensorID) != HAL_OK)
		return HAL_ERROR;
    // 3) TVG autotune (keep your existing approach, but reuse helpers)
    TVG_t tvg_new = sensors[sensorID].Registers.EEData.TVG;
    // Smooth echo to estimate noise & ring-down
    uint8_t env[128];
    for (int i = 0; i < 128; i++) {
		env[i] = median5_u8(echoAvg[u8_clamp(i - 2, 0, 127)], echoAvg[u8_clamp(i - 1, 0, 127)], echoAvg[i], echoAvg[u8_clamp(i + 1, 0, 127)], echoAvg[u8_clamp(i + 2, 0, 127)]);
    }
    // noise estimate from last 16 bins
    uint16_t acc = 0;
    for (int i = 112; i < 128; i++)
		acc += env[i];
    const uint8_t noise = (uint8_t)(acc / 16);
    const int thr_env = (int)noise + (int)noiseMargin;
    // find ring-down end (K consecutive bins below threshold)
    const uint8_t K = 6;
    int run = 0, rd_bin = -1;
    for (int i = 0; i < 128; i++) {
        if (env[i] < thr_env) {
			if (++run >= K) {
				rd_bin = i - (K - 1);
				break;
			}
		} else
			run = 0;
    }
    if (rd_bin < 0)
		rd_bin = 8;
    // derive TVG timing and segment gains
    const uint16_t bin_us = 32*(p1_n + 1);
    const uint16_t t0_us = rd_bin * bin_us;
    uint16_t rem_us = recTime_us_P1 - t0_us;
    if (rem_us < 5 * bin_us)
		rem_us = 5 * bin_us;
    const uint16_t dt_us = (uint16_t)lroundf(0.17f * rem_us);
    const uint16_t T0_us_snap = time_lookup[time_code_from_us(t0_us)];
    const uint16_t Td_us_snap = time_lookup[time_code_from_us(dt_us)];
    uint8_t g[5] = {0};
    int start_bin = (int)lroundf(T0_us_snap / bin_us);
    start_bin = u8_clamp(start_bin, 0, 127);
    int seg_bins = (int)lroundf(Td_us_snap / bin_us);
    if (seg_bins < 1)
		seg_bins = 1;
    for (int s = 0; s < 5; s++) {
        int b0 = start_bin + s * seg_bins;
        int b1 = (s == 4) ? 128 : (b0 + seg_bins);
        if (b0 >= 128) {
			g[s] = (s ? g[s-1] : 0);
			continue;
		}
        if (b1 > 128)
			b1 = 128;
        float tmp[80]; int m = 0;
        for (int i = b0; i < b1 && m < 80; i++) {
            float e = (env[i] < 1.f) ? 1.f : (float)env[i];
            tmp[m++] = (float)targetAmp / e;
        }
        // trimmed mean for robustness
        for (int i=1;i<m;i++){
			float key=tmp[i];
			int j=i-1;
			while(j>=0 && tmp[j]>key){
				tmp[j+1]=tmp[j];
				j--;
			}
			tmp[j+1]=key;
		}
        int trim = (m>=10)? (m/10) : 0;
		if (2*trim >= m)
			trim = 0;
        double sum=0.0; int cnt=0;
        for (int i=trim;i<m-trim;i++){
			sum+=tmp[i];
			cnt++;
		}
        float mean = (cnt? (float)(sum/(double)cnt) : 1.0f);
        g[s] = tvg_code_from_factor(mean);
    }
    tvg_new.TVGAIN0 = (uint8_t)((time_code_from_us(T0_us_snap) << 4) | (time_code_from_us(Td_us_snap) & 0x0F));
    tvg_new.TVGAIN1 = (uint8_t)((time_code_from_us(Td_us_snap) << 4) | (time_code_from_us(Td_us_snap) & 0x0F));
    tvg_new.TVGAIN2 = (uint8_t)((time_code_from_us(Td_us_snap) << 4) | (time_code_from_us(Td_us_snap) & 0x0F));
    tvg_new.TVGAIN3 = (uint8_t)((g[0] & 0x3F) << 2) | ((g[1] >> 4) & 0x03);
    tvg_new.TVGAIN4 = (uint8_t)((g[1] & 0x0F) << 4) | ((g[2] >> 2) & 0x0F);
    tvg_new.TVGAIN5 = (uint8_t)((g[2] & 0x03) << 6) | (g[3] & 0x3F);
    tvg_new.TVGAIN6 = (uint8_t)((g[4] & 0x3F) << 2) | (sensors[sensorID].Registers.EEData.TVG.TVGAIN6 & 0x01);
    if (PGA460_SetTVG(sensorID, PGA460_GAIN_58_90dB, &tvg_new) != HAL_OK)
		return HAL_ERROR;
    return HAL_OK;
}
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

#define INV_SQRT3    0.57735026919f          // 1/sqrt(3)
#define DEG_PER_RAD  (57.29577951308232f)    // 180/pi
#define EPS_TOF      (1e-6f)                 // 1 µs minimal one-way ToF (tune if needed)
#define MAX_TOF      (0.050f)                // 50 ms max one-way (tune)
#define MAX_WIND_MS  (80.0f)                 // sanity cap on reported wind speed

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

const PGA460_Regs_t s1 = {
.EEData.UserData = {0},    // 0x00..0x13
.EEData.TVG = {            // keep your proven TVG shape
	.TVGAIN0 = 131, .TVGAIN1 = 15, .TVGAIN2 = 85,
	.TVGAIN3 = 3,  .TVGAIN4 = 255, .TVGAIN5 = 255, .TVGAIN6 = 252
},
.EEData.Sett = {
	.INIT_GAIN    = 0,          // low initial gain (near target)
	.FREQUENCY    = 144,          // 58.0 kHz (0.2*140+30)  ? CUSA center freq
	.DEADTIME     = 0,          // deglitch=32 µs, pulse DT=0
	.PULSE_P1     = 68,          // IO_IF_SEL=0 (time-based), DIAG=1, P1=4 pulses
	.PULSE_P2     = 8,          // P2=8 pulses (addr=0)
	.CURR_LIM_P1  = 2,          // keep your hardware-safe limits
	.CURR_LIM_P2  = 2,
	.REC_LENGTH   = 0,          // 4.096 ms window per preset
	.FREQ_DIAG    = 32,          // standard diag enable
	.SAT_FDIAG_TH = 238,          // TI typical
	.FVOLT_DEC    = 124,          // TI typical
	.DECPL_TEMP   = 79,          // TI typical
	.DSP_SCALE    = 0,
	.TEMP_TRIM    = 136,
	.P1_GAIN_CTRL = 0,
	.P2_GAIN_CTRL = 0
},
.eeCtrl = { .Val.Value = 0 },
.Filters = {                   // fine to leave as you had them
	.BPF_A2_MSB=137, .BPF_A2_LSB=97,
	.BPF_A3_MSB=252, .BPF_A3_LSB=206,
	.BPF_B1_MSB=1,  .BPF_B1_LSB=153,
	.LPF_A2_MSB=128,.LPF_A2_LSB=205,
	.LPF_B1_MSB=0,  .LPF_B1_LSB=103
},
.TestMux = { .Val.Value = 0 },
.Stat0   = { .Val.Value = 0 },
.Stat1   = { .Val.Value = 0 },
.THR = {
	// keep your short-range threshold profile
	.P1_THR = {6, 48, 0, 0, 0, 0, 255, 254, 16, 132, 33, 16, 16, 16, 16, 7},
	.P2_THR = {6, 48, 0, 0, 0, 0, 255, 254, 16, 132, 33, 16, 16, 16, 16, 7}
}
};
const PGA460_Regs_t s2 = {
.EEData.UserData = {0},    // 0x00..0x13
.EEData.TVG = {            // keep your proven TVG shape
	.TVGAIN0 = 131, .TVGAIN1 = 15, .TVGAIN2 = 85,
	.TVGAIN3 = 3,  .TVGAIN4 = 255, .TVGAIN5 = 255, .TVGAIN6 = 252
},
.EEData.Sett = {
	.INIT_GAIN    = 0,          // low initial gain (near target)
	.FREQUENCY    = 144,          // 58.0 kHz (0.2*140+30)  ? CUSA center freq
	.DEADTIME     = 0,          // deglitch=32 µs, pulse DT=0
	.PULSE_P1     = 68,          // IO_IF_SEL=0 (time-based), DIAG=1, P1=4 pulses
	.PULSE_P2     = 8,          // P2=8 pulses (addr=0)
	.CURR_LIM_P1  = 8,          // keep your hardware-safe limits
	.CURR_LIM_P2  = 8,
	.REC_LENGTH   = 0,          // 4.096 ms window per preset
	.FREQ_DIAG    = 32,          // standard diag enable
	.SAT_FDIAG_TH = 238,          // TI typical
	.FVOLT_DEC    = 124,          // TI typical
	.DECPL_TEMP   = 79,          // TI typical
	.DSP_SCALE    = 0,
	.TEMP_TRIM    = 136,
	.P1_GAIN_CTRL = 0,
	.P2_GAIN_CTRL = 0
},
.eeCtrl = { .Val.Value = 0 },
.Filters = {                   // fine to leave as you had them
	.BPF_A2_MSB=137, .BPF_A2_LSB=97,
	.BPF_A3_MSB=252, .BPF_A3_LSB=206,
	.BPF_B1_MSB=1,  .BPF_B1_LSB=153,
	.LPF_A2_MSB=128,.LPF_A2_LSB=205,
	.LPF_B1_MSB=0,  .LPF_B1_LSB=103
},
.TestMux = { .Val.Value = 0 },
.Stat0   = { .Val.Value = 0 },
.Stat1   = { .Val.Value = 0 },
.THR = {
	// keep your short-range threshold profile
	.P1_THR = {6, 48, 0, 0, 0, 0, 255, 254, 16, 132, 33, 16, 16, 16, 16, 7},
	.P2_THR = {6, 48, 0, 0, 0, 0, 255, 254, 16, 132, 33, 16, 16, 16, 16, 7}
}
};
const PGA460_Regs_t s3 = {
.EEData.UserData = {0},    // 0x00..0x13
.EEData.TVG = {            // keep your proven TVG shape
	.TVGAIN0 = 131, .TVGAIN1 = 15, .TVGAIN2 = 85,
	.TVGAIN3 = 3,  .TVGAIN4 = 255, .TVGAIN5 = 255, .TVGAIN6 = 252
},
.EEData.Sett = {
	.INIT_GAIN    = 0,          // low initial gain (near target)
	.FREQUENCY    = 144,          // 58.0 kHz (0.2*140+30)  ? CUSA center freq
	.DEADTIME     = 0,          // deglitch=32 µs, pulse DT=0
	.PULSE_P1     = 68,          // IO_IF_SEL=0 (time-based), DIAG=1, P1=4 pulses
	.PULSE_P2     = 8,          // P2=8 pulses (addr=0)
	.CURR_LIM_P1  = 8,          // keep your hardware-safe limits
	.CURR_LIM_P2  = 8,
	.REC_LENGTH   = 0,          // 4.096 ms window per preset
	.FREQ_DIAG    = 32,          // standard diag enable
	.SAT_FDIAG_TH = 238,          // TI typical
	.FVOLT_DEC    = 124,          // TI typical
	.DECPL_TEMP   = 79,          // TI typical
	.DSP_SCALE    = 0,
	.TEMP_TRIM    = 136,
	.P1_GAIN_CTRL = 0,
	.P2_GAIN_CTRL = 0
},
.eeCtrl = { .Val.Value = 0 },
.Filters = {                   // fine to leave as you had them
	.BPF_A2_MSB=137, .BPF_A2_LSB=97,
	.BPF_A3_MSB=252, .BPF_A3_LSB=206,
	.BPF_B1_MSB=1,  .BPF_B1_LSB=153,
	.LPF_A2_MSB=128,.LPF_A2_LSB=205,
	.LPF_B1_MSB=0,  .LPF_B1_LSB=103
},
.TestMux = { .Val.Value = 0 },
.Stat0   = { .Val.Value = 0 },
.Stat1   = { .Val.Value = 0 },
.THR = {
	// keep your short-range threshold profile
	.P1_THR = {6, 48, 0, 0, 0, 0, 255, 254, 16, 132, 33, 16, 16, 16, 16, 7},
	.P2_THR = {6, 48, 0, 0, 0, 0, 255, 254, 16, 132, 33, 16, 16, 16, 16, 7}
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

PGA460_EnvData_t externalData = {
    .Height      = 58.0f,     // meters
    .Temperature = 23.63f,    // °C
    .RH          = 62.22f,    // %
    .Pressure    = 1027.7f,   // hPa
    .SoundSpeed  = 0.0f       // will be computed
};

float soundSpeed = 343.0f;

static float PGA460_ComputeSoundSpeed(PGA460_EnvData_t *env)
{
    if (!env) return 343.0f;

    // --- Temperature ---
    const double T_C = (double)env->Temperature;
    const double T_K = T_C + KELVIN_OFFSET;

    // --- Pressure (hPa). If not provided, estimate from height with std. atmosphere (0–11 km) ---
    double P_hPa = (double)env->Pressure;
    if (P_hPa <= 0.0) {
        const double h = (double)env->Height; // meters
        // Barometric formula (troposphere):
        // P = P0 * (1 - L*h/T0)^(g*M/(R*L))  [Pa]
        const double term = 1.0 - (L * h) / T0;
        const double expo = (G * M) / (R * L);
        double P_Pa = P0 * pow(term, expo);
        if (P_Pa < 1.0) P_Pa = 1.0; // guard
        P_hPa = P_Pa * 0.01;        // convert Pa ? hPa
        env->Pressure = (float)P_hPa;
    }

    // --- Relative humidity as fraction ---
    const double RH_frac = ((double)env->RH) / RH_DIVISOR;

    // --- Saturation vapor pressure over water (Tetens, hPa) ---
    // es = 6.1078 * exp(17.2693882 * T_C / (T_C + 237.3))
    const double es = SATURATION_CONSTANT * exp(17.2693882 * T_C / (T_C + 237.3));

    // Actual vapor pressure (hPa)
    const double e = RH_frac * es;

    // Mixing ratio (kg/kg); 0.62198 ˜ Rd/Rv ratio factor
    const double w = 0.62198 * e / fmax(1e-6, (P_hPa - e));

    // Specific humidity
    const double q = w / (1.0 + w);

    // Virtual temperature Tv = T * (1 + (Rv/Rd - 1)*q); WATER_VAPOR_EFFECT ˜ 0.6077
    const double Tv = T_K * (1.0 + WATER_VAPOR_EFFECT * q);

    // Speed of sound in moist air: c = sqrt(gamma * Rd * Tv)
    const double c = sqrt(GAMMA * R_D * Tv);

    env->SoundSpeed = (float)c;
    return (float)c;
}

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
		HAL_Delay(PGA460_CAPTURE_DELAY_MS);
	}
	printf("EEPROM individual read complete.\n");
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


static HAL_StatusTypeDef PGA460_SetEEPROMAccess(const uint8_t sensorID, const uint8_t unlock) {
	uint8_t accessByte = unlock ? PGA460_UNLOCK_EEPROM : PGA460_LOCK_EEPROM;
	uint8_t cmd[5] = {PGA460_SYNC, PGA460_CMD_REGISTER_WRITE, REG_EE_CTRL, accessByte, 0x00};
	cmd[4] = PGA460_CalculateChecksum(&cmd[1], 3); // Checksum over CMD, ADDR, DATA
	if (HAL_UART_Transmit(sensors[sensorID].uartPort, cmd, sizeof(cmd), UART_TIMEOUT) != HAL_OK) {
		DEBUG("Sensor %d: EEPROM %s failed\n", sensorID, unlock ? "unlock" : "lock");
		return HAL_ERROR;
	}
	// Update status flag on success
	sensors[sensorID].Registers.eepromUnlocked = unlock ? 1 : 0;
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
		HAL_Delay(PGA460_EEPROM_WRITE_DELAY_MS);
		sensors[i].Measures.tof_us  = 0;
		sensors[i].Measures.width     = 0;
		sensors[i].Measures.amplitude = 0;
		// Step 2: Threshold write (uses current sensors[i].Registers.THR) !!! Mandatory step to start the sensor
		if (PGA460_SetThresholds(i) != HAL_OK) {
			DEBUG("Sensor %u: Threshold write failed!\n", i);
			return HAL_ERROR;
		}
//		PGA460_ReadAndPrintEEPROM(i);
//		// Step 2: EEPROM bulk write (shadow -> device)
//		if (PGA460_EEPROMBulkWrite(i) != HAL_OK) {
//			DEBUG("Sensor %u: EEPROM bulk write failed!\n", i);
//			return HAL_ERROR;
//		}
//		PGA460_ReadAndPrintEEPROM(i);
//		if(burnEEPROM) {
//			// Step 3: Optional EEPROM Burn
//			if (PGA460_BurnEEPROM(i) != HAL_OK) {
//				DEBUG("Sensor %u: EEPROM burn failed!\n", i);
//				return HAL_ERROR;
//			}
//		}
//		// Step 4: TVG write (push current TVG from mirror)
//		if (PGA460_SetTVG(i, PGA460_GAIN_58_90dB, &sensors[i].Registers.EEData.TVG) != HAL_OK) {
//				DEBUG("Sensor %u: TVG bulk write failed!\n", i);
//				return HAL_ERROR;
//		}
		// Step 5: Diagnostics/status
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
		// Step 7: Optional Echo Data Dump for verification
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
		HAL_Delay(PGA460_EEPROM_WRITE_DELAY_MS);
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
	if (HAL_UART_Transmit(sensors[sensorID].uartPort, tx, sizeof(tx), UART_TIMEOUT) != HAL_OK)
		return HAL_ERROR;
	// Small settle time (keep if you observed the device needing it)
	HAL_Delay(10);
	return HAL_OK;
}


HAL_StatusTypeDef PGA460_CheckStatus(uint8_t sensorID) {
	// Read both DEV_STAT registers
	if (PGA460_RegisterRead(sensorID, REG_DEV_STAT0, &sensors[sensorID].Registers.Stat0.Val.Value) != HAL_OK || PGA460_RegisterRead(sensorID, REG_DEV_STAT1, &sensors[sensorID].Registers.Stat1.Val.Value) != HAL_OK) {
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


// EEPROM bulk write: send 44-byte EEPROM image (0x00..0x2B) to device shadow
HAL_StatusTypeDef PGA460_EEPROMBulkWrite(uint8_t sensorID) {
	// Step 1: Unlock EEPROM if needed
	if (sensors[sensorID].Registers.eepromUnlocked == 0) {
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
	HAL_Delay(PGA460_EEPROM_WRITE_DELAY_MS);  // Wait for EEPROM write
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
	uint8_t idle = 1;
	for (uint32_t i = 0; i < sizeof(rx); i++) {
		if (rx[i] != 0xFF) {
			idle = 0;
			break;
		}
	}
	if (idle) return HAL_ERROR;
	// --- Step 3: Copy TVG bytes into mirror (skip rx[0] = DIAG) ---
	memcpy(&sensors[sensorID].Registers.EEData.TVG, &rx[1], sizeof(TVG_t));
	return HAL_OK;
}


// Set AFE gain range (DECPL_TEMP[7:6]) and write TVG (TVGAIN0..6) via bulk write.
HAL_StatusTypeDef PGA460_SetTVG(uint8_t sensorID, PGA460_GainRange_t gain_range, const TVG_t *tvg) {
	// 1) Update DECPL_TEMP[7:6] = AFE_GAIN_RNG, preserve other bits.
	sensors[sensorID].Registers.EEData.Sett.DECPL_TEMP = (sensors[sensorID].Registers.EEData.Sett.DECPL_TEMP & 0x3F) | gain_range;
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
	HAL_Delay(PGA460_CAPTURE_DELAY_MS);
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
	HAL_Delay(PGA460_CAPTURE_DELAY_MS);
	return HAL_OK;
}


HAL_StatusTypeDef PGA460_UltrasonicCmd(const uint8_t sensorID, const PGA460_Command_t cmd, const uint8_t noWait) {
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
	if (!noWait && (len == 4)) {
		HAL_Delay(PGA460_CAPTURE_DELAY_MS); // up to ~65 ms capture time
	}
	return HAL_OK;
}


HAL_StatusTypeDef PGA460_GetUltrasonicMeasurement(const uint8_t sensorID)
{
    uint8_t localBuffer[PGA_OBJ_DATA_SIZE] = {0};

    // 1) Request UMR
    if (HAL_UART_Transmit(sensors[sensorID].uartPort, pga460_cmd_ultrasonic_measurement_result, PGA_CMD_SIZE, UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Measurement Result Request Failed!\n", sensorID);
        return HAL_ERROR;
    }

    // 2) Read response
    if (HAL_UART_Receive(sensors[sensorID].uartPort, localBuffer, sizeof(localBuffer), UART_TIMEOUT) != HAL_OK) {
        DEBUG("Sensor %d: Measurement Result Retrieval Failed!\n", sensorID);
        return HAL_ERROR;
    }

    // Update speed of sound from environment
    PGA460_ComputeSoundSpeed(&externalData);

    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t stored = 0;

    // 3) Parse each object, store first valid into sensors[sensorID].Measures
    for (uint8_t i = 0; i < PGA_OBJECTS_TRACKED; i++) {
        const uint8_t base = 2 + (i * 4); // skip 2-byte diagnostic header
        const uint16_t rawTOF   = ((uint16_t)localBuffer[base] << 8) | localBuffer[base + 1];
        const uint8_t  widthRaw = localBuffer[base + 2];
        const uint8_t  amplitude = localBuffer[base + 3];

        //DEBUG("Sensor %d: Raw TOF = %04X\n", sensorID, rawTOF);

        if (rawTOF > 0 && rawTOF != 0xFFFF) {
            const uint16_t width_us = (uint16_t)widthRaw * 16;
            const float distance_m  = (float)rawTOF * TOF_US_TO_S * externalData.SoundSpeed;
            if (!stored) {
                sensors[sensorID].Measures.tof_us    = rawTOF;
                sensors[sensorID].Measures.width     = (uint8_t)((width_us > 255) ? 255 : width_us);
                sensors[sensorID].Measures.amplitude = amplitude;
                sensors[sensorID].Measures.distance  = distance_m;
                stored = 1;
            }
            //DEBUG("Sensor %d: Obj %u -> %.3f m, Width = %u us, Amplitude = %u\n", sensorID, (unsigned)(i + 1), distance_m, width_us, amplitude);
            status = HAL_OK;
        } else {
            //DEBUG("Sensor %d: Obj %u -> Invalid (TOF = 0x%04X)\n", sensorID, (unsigned)(i + 1), rawTOF);
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
	if (PGA460_UltrasonicCmd(sensorID, (PGA460_Command_t)preset, 0) != HAL_OK) {
		DEBUG("Sensor %d: Echo trigger preset failed!\n", sensorID);
		ret = HAL_ERROR;
		goto cleanup_disable_dump;
	}
	HAL_Delay(PGA460_CAPTURE_DELAY_MS); // allow full capture (~65 ms)
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
	if (offset >  63)
		offset =  63;
	if (offset < -64)
		offset = -64;
	// 3) Read current TEMP_TRIM (0x28)
	uint8_t tempTrim = 0;
	if (PGA460_RegisterRead(sensorID, REG_TEMP_TRIM, &tempTrim) != HAL_OK) {
		DEBUG("Sensor %d: Failed to read TEMP_TRIM register!\n", sensorID);
		return HAL_ERROR;
	}
	DEBUG("Sensor %d: Original TEMP_TRIM = 0x%02X\n", sensorID, tempTrim);
	// 4) Build sign-magnitude byte: bit7 = sign, bits6:0 = magnitude
	uint8_t magnitude = (uint8_t)((offset < 0) ? -offset : offset);
	uint8_t signedOffset = ((offset < 0) ? 0x80 : 0x00) | (magnitude & 0x7F);
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
		if (PGA460_UltrasonicCmd(sensorID, PGA460_CMD_BURST_AND_LISTEN_PRESET1, 0) != HAL_OK) {
			DEBUG("Sensor %d: Burst-and-Listen Command Failed!\n", sensorID);
			return HAL_ERROR;
		}
		HAL_Delay(PGA460_EEPROM_WRITE_DELAY_MS); // allow measurement to complete
	}
	// Send System Diagnostics (0x08) ? 3B command [SYNC][CMD][CHK]
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
	tx[2] = isNoise ? 1 : 0;                        // selector
	tx[3] = PGA460_CalculateChecksum(&tx[1], 2);      // over CMD+SEL
	if (HAL_UART_Transmit(sensors[sensorID].uartPort, tx, sizeof(tx), UART_TIMEOUT) != HAL_OK) {
		DEBUG("Sensor %d: Temp/Noise Measurement Command Failed!\n", sensorID);
		return result;
	}
	HAL_Delay(PGA460_CAPTURE_DELAY_MS);  // small settle for measurement to complete
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

// --- Small helper: measure one directed path TX->RX, returns t [s] -----------
static HAL_StatusTypeDef measure_direct_tof_s(uint8_t tx, uint8_t rx, float *t_s) {
    if (!t_s) return HAL_ERROR;

    // RX in ListenOnly first (window armed), then TX Burst&Listen
    if (PGA460_RunCmd_NoWait(rx, PGA460_CMD_LISTEN_ONLY_PRESET1) != HAL_OK) return HAL_ERROR;
    // tiny arm lead; most UART stacks need a short guard
    HAL_Delay(1);
    if (PGA460_RunCmd_NoWait(tx, PGA460_CMD_BURST_AND_LISTEN_PRESET1) != HAL_OK) return HAL_ERROR;

    // recording window; long enough for ~0.2–1.0 ms paths
    HAL_Delay(PGA460_CAPTURE_DELAY_MS);

    if (PGA460_GetUltrasonicMeasurement(rx) != HAL_OK) return HAL_ERROR;

    const uint16_t tof_us = sensors[rx].Measures.tof_us;  // first object; direct arrival
    if (tof_us == 0 || tof_us == 0xFFFF) return HAL_ERROR;
    *t_s = (float)tof_us * 1e-6f;
    return HAL_OK;
}

/*
 Wind solver for triangular array (Sensor 0 at North, 1 at +120° on East side, 2 at +240° on West side)
 We measure all six directed times:
 t01 (0->1), t10 (1->0), t02 (0->2), t20 (2->0), t12 (1->2), t21 (2->1)
 For each edge AB of length L, the wind component along that edge is:
 w_AB = (L/2) * (1/t_AB - 1/t_BA)           [m/s]   (positive along A?B)
 With your node bearings, the three edge axes have bearings:
 0<->1 at 150°/330°, 0<->2 at 210°/30°, 1<->2 at 270°/90°
 Resolving to East/North components gives a super simple closed form:
 u_E = -w_12
 u_N = -(w_01 + w_02) / v3
 Wind speed  = v(u_E² + u_N²)
 Direction FROM = bearing(towards) + 180°   (0/360=N, 90=E, 180=S, 270=W)
 */
HAL_StatusTypeDef PGA460_MeasureWind(PGA460_Wind_t *out)
{
    if (!out) return HAL_ERROR;

    float t01, t10, t02, t20, t12, t21;
    if (measure_direct_tof_s(0, 1, &t01) != HAL_OK) return HAL_ERROR;
    if (measure_direct_tof_s(1, 0, &t10) != HAL_OK) return HAL_ERROR;
    if (measure_direct_tof_s(0, 2, &t02) != HAL_OK) return HAL_ERROR;
    if (measure_direct_tof_s(2, 0, &t20) != HAL_OK) return HAL_ERROR;
    if (measure_direct_tof_s(1, 2, &t12) != HAL_OK) return HAL_ERROR;
    if (measure_direct_tof_s(2, 1, &t21) != HAL_OK) return HAL_ERROR;

    // Basic validation (seconds, one-way)
    #define VALID_TOF(t) ((t) > EPS_TOF && (t) < MAX_TOF)
    if (!VALID_TOF(t01) || !VALID_TOF(t10) || !VALID_TOF(t02) || !VALID_TOF(t20) || !VALID_TOF(t12) || !VALID_TOF(t21)) {
        return HAL_ERROR;
    }

    // Precompute reciprocals
    const float r01 = 1.0f / t01;
    const float r10 = 1.0f / t10;
    const float r02 = 1.0f / t02;
    const float r20 = 1.0f / t20;
    const float r12 = 1.0f / t12;
    const float r21 = 1.0f / t21;

    // Axial wind components along each edge (positive along A?B)
    const float w01 = S_BASELINE_HALF_M * (r01 - r10);
    const float w02 = S_BASELINE_HALF_M * (r02 - r20);
    const float w12 = S_BASELINE_HALF_M * (r12 - r21);

    // Resolve to ENU components (bearing basis per your comment)
    const float u_E = -w12;
    const float u_N = -(w01 + w02) * INV_SQRT3;

    // Speed (towards) and bearing (towards, 0=North, CW+)
    float speed = sqrtf(u_E*u_E + u_N*u_N);
    float bearing_to_deg = atan2f(u_E, u_N) * DEG_PER_RAD;  // atan2(E, N)
    if (bearing_to_deg < 0.0f) bearing_to_deg += 360.0f;

    // Convert to FROM direction
    float dir_from = bearing_to_deg + 180.0f;
    if (dir_from >= 360.0f) dir_from -= 360.0f;

    // Sanity clamp (optional)
    if (speed > MAX_WIND_MS) speed = MAX_WIND_MS;

    out->Speed     = speed;
    out->Direction = dir_from;
    return HAL_OK;
}

/**
 * Measure distance with 3 transducers:
 *  - TX=0, RX={1,2}; TX=1, RX={0,2}; TX=2, RX={0,1}
 * For each TX, we measure directed one-way times t0a,t0b (seconds) and estimate range:
 *   R ˜ (c/4) * (t0a + t0b)
 * Returns the average of all valid ranges in meters.
 * Returns -1.0f if all attempts failed.
 */
float PGA460_MeasureDistance_Avg(void) {
    // Update speed of sound from environment
    PGA460_ComputeSoundSpeed(&externalData);

    float sumR = 0.0f;
    int   cnt  = 0;

    // Local validity check (seconds, one-way)
    #define VALID_TOF(t) ((t) > EPS_TOF && (t) < MAX_TOF)

    // Try one TX cycle: TX -> RXa and TX -> RXb (both one-way directed)
    #define TRY_TX(tx, rxA, rxB) do {                             \
        float t0a = 0.0f, t0b = 0.0f;                             \
        if (measure_direct_tof_s((tx), (rxA), &t0a) == HAL_OK &&  \
            measure_direct_tof_s((tx), (rxB), &t0b) == HAL_OK &&  \
            VALID_TOF(t0a) && VALID_TOF(t0b)) {                   \
            const float z = 0.25f * externalData.SoundSpeed * (t0a + t0b);              \
            sumR += z;                                            \
            cnt++;                                                \
        }                                                         \
    } while (0)

    TRY_TX(0, 1, 2);
    TRY_TX(1, 0, 2);
    TRY_TX(2, 0, 1);

    #undef TRY_TX
    #undef VALID_TOF

    if (cnt == 0) return -1.0f;     // no valid measurement
    return sumR / (float)cnt;       // average meters
}

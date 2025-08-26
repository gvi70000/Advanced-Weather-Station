// TO BE USED WITH PGA460.html to generate registers values
#ifndef PGA460_H
#define PGA460_H

#include "stm32f3xx_hal.h"

	#define PGA_CMD_SIZE			3
	#define PGA_READ_SIZE			4
	#define PGA_WRITE_SIZE			5
	#define PGA_CMD_COUNT			26
	// PGA460 supports up to 8 objects per measurement
	#define PGA_MAX_OBJECTS			8
	// Number of objects we want to track (Must be = PGA_MAX_OBJECTS)
	#define PGA_OBJECTS_TRACKED		1  

	#if PGA_OBJECTS_TRACKED > PGA_MAX_OBJECTS
			#error "PGA_OBJECTS_TRACKED cannot be greater than PGA_MAX_OBJECTS!"
	#endif
	#define PGA_OBJ_DATA_SIZE (2 + (PGA_OBJECTS_TRACKED * 4))

	// Number of ultrasonic sensors in the array
	#define ULTRASONIC_SENSOR_COUNT	3	// Number of sensors
	// Error value for temperature or noise
	#define PGA460_TEMP_ERR			999.0f
	
	// Address 0h-2Bh: EEPROM nonvolatile memory.
	// Content in these registers is preserved during power cycle and low-power mode.
	// Address 40h-4Dh and address 5Fh-7Fh: Register-based volatile memory.
	// Content in these registers is lost during power cycle and low-power mode.
	// Address 2Ch-3Fh and address 4Eh-5Eh are reserved for Texas Instruments internal use and are not accessible to the user.

	// Sync byte for baud rate auto-detection
	#define PGA460_SYNC							0x55  
	#define PGA460_UNLOCK_EEPROM		0x68
	#define PGA460_LOCK_EEPROM			0x69
	#define ECHO_DATA_DUMP_ENABLE		0x80
	#define ECHO_DATA_DUMP_DISABLE	0x00
	#define ECHO_DATA_TOTAL_BYTES		130  // 2 bytes header + 128 data

	// EEPROM Register Macros (addresses 0x00 to 0x2B)
	#define REG_USER_DATA1      0x00  // R/W - Reset: 0x00
	#define REG_USER_DATA2      0x01  // R/W - Reset: 0x00
	#define REG_USER_DATA3      0x02  // R/W - Reset: 0x00
	#define REG_USER_DATA4      0x03  // R/W - Reset: 0x00
	#define REG_USER_DATA5      0x04  // R/W - Reset: 0x00
	#define REG_USER_DATA6      0x05  // R/W - Reset: 0x00
	#define REG_USER_DATA7      0x06  // R/W - Reset: 0x00
	#define REG_USER_DATA8      0x07  // R/W - Reset: 0x00
	#define REG_USER_DATA9      0x08  // R/W - Reset: 0x00
	#define REG_USER_DATA10     0x09  // R/W - Reset: 0x00
	#define REG_USER_DATA11     0x0A  // R/W - Reset: 0x00
	#define REG_USER_DATA12     0x0B  // R/W - Reset: 0x00
	#define REG_USER_DATA13     0x0C  // R/W - Reset: 0x00
	#define REG_USER_DATA14     0x0D  // R/W - Reset: 0x00
	#define REG_USER_DATA15     0x0E  // R/W - Reset: 0x00
	#define REG_USER_DATA16     0x0F  // R/W - Reset: 0x00
	#define REG_USER_DATA17     0x10  // R/W - Reset: 0x00
	#define REG_USER_DATA18     0x11  // R/W - Reset: 0x00
	#define REG_USER_DATA19     0x12  // R/W - Reset: 0x00
	#define REG_USER_DATA20     0x13  // R/W - Reset: 0x00

	#define REG_TVGAIN0         0x14  // R/W - Reset: 0xAF
	#define REG_TVGAIN1         0x15  // R/W - Reset: 0xFF
	#define REG_TVGAIN2         0x16  // R/W - Reset: 0xFF
	#define REG_TVGAIN3         0x17  // R/W - Reset: 0x2D
	#define REG_TVGAIN4         0x18  // R/W - Reset: 0x68
	#define REG_TVGAIN5         0x19  // R/W - Reset: 0x36
	#define REG_TVGAIN6         0x1A  // R/W - Reset: 0xFC

	#define REG_INIT_GAIN       0x1B  // R/W - Reset: 0xC0
	#define REG_FREQUENCY       0x1C  // R/W - Reset: 0x8C
	#define REG_DEADTIME        0x1D  // R/W - Reset: 0x00
	#define REG_PULSE_P1        0x1E  // R/W - Reset: 0x01
	#define REG_PULSE_P2        0x1F  // R/W - Reset: 0x12
	#define REG_CURR_LIM_P1     0x20  // R/W - Reset: 0x47
	#define REG_CURR_LIM_P2     0x21  // R/W - Reset: 0xFF
	#define REG_REC_LENGTH      0x22  // R/W - Reset: 0x1C
	#define REG_FREQ_DIAG       0x23  // R/W - Reset: 0x00
	#define REG_SAT_FDIAG_TH    0x24  // R/W - Reset: 0xEE
	#define REG_FVOLT_DEC       0x25  // R/W - Reset: 0x7C
	#define REG_DECPL_TEMP      0x26  // R/W - Reset: 0x0A
	#define REG_DSP_SCALE       0x27  // R/W - Reset: 0x00
	#define REG_TEMP_TRIM       0x28  // R/W - Reset: 0x00
	#define REG_P1_GAIN_CTRL    0x29  // R/W - Reset: 0x05
	#define REG_P2_GAIN_CTRL    0x2A  // R/W - Reset: 0x05
	#define REG_EE_CRC          0x2B  // R (Auto calculated on EEPROM burn) - Reset: N/A

	// Register-based Volatile Memory (addresses 0x40 to 0x7F)
	#define REG_EE_CTRL       	0x40  // R/W - Reset: 0x00
	#define REG_BPF_A2_MSB      0x41  // R/W - Reset: 0x00
	#define REG_BPF_A2_LSB			0x42  // R/W - Reset: 0x00
	#define REG_BPF_A3_MSB      0x43  // R/W - Reset: 0x00
	#define REG_BPF_A3_LSB			0x44  // R/W - Reset: 0x00
	#define REG_BPF_B1_MSB      0x45  // R/W - Reset: 0x00
	#define REG_BPF_B1_LSB			0x46  // R/W - Reset: 0x00
	#define REG_LPF_A2_MSB      0x47  // R/W - Reset: 0x00
	#define REG_LPF_A2_LSB			0x48  // R/W - Reset: 0x00
	#define REG_LPF_B1_MSB      0x49  // R/W - Reset: 0x00
	#define REG_LPF_B1_LSB			0x4A  // R/W - Reset: 0x00
	#define REG_TEST_MUX				0x4B  // R/W - Reset: 0x00
	#define REG_DEV_STAT0				0x4C  // R/W - Reset: 0x84
	#define REG_DEV_STAT1				0x4D  // R/W - Reset: 0x00
	#define REG_P1_THR_0        0x5F  // R/W - Reset: 0x00
	#define REG_P1_THR_1        0x60  // R/W - Reset: 0x00
	#define REG_P1_THR_2        0x61  // R/W - Reset: 0x00
	#define REG_P1_THR_3        0x62  // R/W - Reset: 0x00
	#define REG_P1_THR_4        0x63  // R/W - Reset: 0x00
	#define REG_P1_THR_5        0x64  // R/W - Reset: 0x00
	#define REG_P1_THR_6        0x65  // R/W - Reset: 0x00
	#define REG_P1_THR_7        0x66  // R/W - Reset: 0x00
	#define REG_P1_THR_8        0x67  // R/W - Reset: 0x00
	#define REG_P1_THR_9        0x68  // R/W - Reset: 0x00
	#define REG_P1_THR_10       0x69  // R/W - Reset: 0x00
	#define REG_P1_THR_11       0x6A  // R/W - Reset: 0x00
	#define REG_P1_THR_12       0x6B  // R/W - Reset: 0x00
	#define REG_P1_THR_13       0x6C  // R/W - Reset: 0x00
	#define REG_P1_THR_14       0x6D  // R/W - Reset: 0x00
	#define REG_P1_THR_15       0x6E  // R/W - Reset: 0x00
	#define REG_P2_THR_0        0x6F  // R/W - Reset: 0x00
	#define REG_P2_THR_1        0x70  // R/W - Reset: 0x00
	#define REG_P2_THR_2        0x71  // R/W - Reset: 0x00
	#define REG_P2_THR_3        0x72  // R/W - Reset: 0x00
	#define REG_P2_THR_4        0x73  // R/W - Reset: 0x00
	#define REG_P2_THR_5        0x74  // R/W - Reset: 0x00
	#define REG_P2_THR_6        0x75  // R/W - Reset: 0x00
	#define REG_P2_THR_7        0x76  // R/W - Reset: 0x00
	#define REG_P2_THR_8        0x77  // R/W - Reset: 0x00
	#define REG_P2_THR_9        0x78  // R/W - Reset: 0x00
	#define REG_P2_THR_10       0x79  // R/W - Reset: 0x00
	#define REG_P2_THR_11       0x7A  // R/W - Reset: 0x00
	#define REG_P2_THR_12       0x7B  // R/W - Reset: 0x00
	#define REG_P2_THR_13       0x7C  // R/W - Reset: 0x00
	#define REG_P2_THR_14       0x7D  // R/W - Reset: 0x00
	#define REG_P2_THR_15       0x7E  // R/W - Reset: 0x00
	#define REG_THR_CRC         0x7F  // R/W - Reset: 0x00
	
// Enum for PGA460 commands
typedef enum {
    // UART Command Codes
    PGA460_CMD_BURST_AND_LISTEN_PRESET1       = 0x00,
    PGA460_CMD_BURST_AND_LISTEN_PRESET2       = 0x01,
    PGA460_CMD_LISTEN_ONLY_PRESET1            = 0x02,
    PGA460_CMD_LISTEN_ONLY_PRESET2            = 0x03,
    PGA460_CMD_TEMP_AND_NOISE_MEASUREMENT     = 0x04,
    PGA460_CMD_ULTRASONIC_MEASUREMENT_RESULT  = 0x05,
    PGA460_CMD_TEMP_AND_NOISE_RESULT          = 0x06,
    PGA460_CMD_TRANSDUCER_ECHO_DATA_DUMP      = 0x07,
    PGA460_CMD_SYSTEM_DIAGNOSTICS             = 0x08,
    PGA460_CMD_REGISTER_READ                  = 0x09,
    PGA460_CMD_REGISTER_WRITE                 = 0x0A,
    PGA460_CMD_EEPROM_BULK_READ               = 0x0B,
    PGA460_CMD_EEPROM_BULK_WRITE              = 0x0C,
    PGA460_CMD_TVG_BULK_READ                  = 0x0D,
    PGA460_CMD_TVG_BULK_WRITE                 = 0x0E,
    PGA460_CMD_THRESHOLD_BULK_READ            = 0x0F,
    PGA460_CMD_THRESHOLD_BULK_WRITE           = 0x10 //,
    // Broadcast Commands - not used 
    // PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1  = 0x11,
    // PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2  = 0x12,
    // PGA460_CMD_BROADCAST_LISTEN_ONLY_P1       = 0x13,
    // PGA460_CMD_BROADCAST_LISTEN_ONLY_P2       = 0x14,
    // PGA460_CMD_BROADCAST_TEMP_NOISE_MEASURE   = 0x15,
    // PGA460_CMD_BROADCAST_REGISTER_WRITE       = 0x16,
    // PGA460_CMD_BROADCAST_EEPROM_BULK_WRITE    = 0x17,
    // PGA460_CMD_BROADCAST_TVG_BULK_WRITE       = 0x18,
    // PGA460_CMD_BROADCAST_THRESHOLD_BULK_WRITE = 0x19
} PGA460_Command_t;

// --- TVG structure ---
typedef struct __attribute__((packed)) {
    uint8_t TVGAIN0; // REG_TVGAIN0 (0x14)
    uint8_t TVGAIN1; // REG_TVGAIN1 (0x15)
    uint8_t TVGAIN2; // REG_TVGAIN2 (0x16)
    uint8_t TVGAIN3; // REG_TVGAIN3 (0x17)
    uint8_t TVGAIN4; // REG_TVGAIN4 (0x18)
    uint8_t TVGAIN5; // REG_TVGAIN5 (0x19)
    uint8_t TVGAIN6; // REG_TVGAIN6 (0x1A)
} TVG_t;

// --- Settings structure ---
typedef struct __attribute__((packed)) {
    // --- General EEPROM Settings ---
    uint8_t INIT_GAIN;       // 0x1B
    uint8_t FREQUENCY;       // 0x1C
    uint8_t DEADTIME;        // 0x1D
    uint8_t PULSE_P1;        // 0x1E
    uint8_t PULSE_P2;        // 0x1F
    uint8_t CURR_LIM_P1;     // 0x20
    uint8_t CURR_LIM_P2;     // 0x21
    uint8_t REC_LENGTH;      // 0x22
    uint8_t FREQ_DIAG;       // 0x23
    uint8_t SAT_FDIAG_TH;    // 0x24
    uint8_t FVOLT_DEC;       // 0x25
    uint8_t DECPL_TEMP;      // 0x26
    uint8_t DSP_SCALE;       // 0x27
    uint8_t TEMP_TRIM;       // 0x28
    uint8_t P1_GAIN_CTRL;    // 0x29
    uint8_t P2_GAIN_CTRL;    // 0x2A
		//uint8_t EE_CRC;			// 0x2B
} Settings_t;

	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t EE_PRGM      : 1; // EEPROM Program Trigger (bit 0)
																				// 0b = Disabled
																				// 1b = Program Data to EEPROM
							uint8_t EE_RLOAD     : 1; // EEPROM Reload Trigger (bit 1)
																				// 0b = Disabled
																				// 1b = Reload Data from EEPROM
							uint8_t EE_PRGM_OK   : 1; // EEPROM Programming Status (bit 2)
																				// 0b = EEPROM was not programmed successfully
																				// 1b = EEPROM was programmed successfully
							uint8_t EE_UNLCK     : 4; // EEPROM Program Enable Unlock Passcode (bits 3:6)
																				// EEPROM program enable unlock passcode register: The valid passcode for enabling EEPROM programming is 0xD.
							uint8_t DATADUMP_EN  : 1; // Data Dump Enable (bit 7)
																				// 0b = Disabled
																				// 1b = Enabled
					} BitField;
			} Val;
	} EE_CNTRL_t;
	
typedef struct __attribute__((packed)) {
    // --- Band-Pass Filter (BPF) Coefficients ---
    uint8_t BPF_A2_MSB;  // 0x41
    uint8_t BPF_A2_LSB;  // 0x42
    uint8_t BPF_A3_MSB;  // 0x43
    uint8_t BPF_A3_LSB;  // 0x44
    uint8_t BPF_B1_MSB;  // 0x45
    uint8_t BPF_B1_LSB;  // 0x46
    // --- Low-Pass Filter (LPF) Coefficients ---
    uint8_t LPF_A2_MSB;  // 0x47
    uint8_t LPF_A2_LSB;  // 0x48
    uint8_t LPF_B1_MSB;  // 0x49
    uint8_t LPF_B1_LSB;  // 0x4A
} Filters_t;

	// Enum for TEST_MUX field (bits 7:5)
	typedef enum {
		TEST_MUX_GND		= 0x0,	// GND ("Mux Off")
		TEST_MUX_AFE_OUTPUT	= 0x1,	// Analog Front End output
		TEST_MUX_RESERVED1	= 0x2,	// Reserved
		TEST_MUX_RESERVED2	= 0x3,	// Reserved
		TEST_MUX_8MHZ_CLOCK	= 0x4,	// 8MHz clock
		TEST_MUX_ADC_SAMPLE	= 0x5	// ADC sample output clock
	} TEST_MUX_Enum_t;

	// Enum for DP_MUX field (bits 2:0)
	typedef enum {
		DP_MUX_OFF			= 0x0, // Disabled
		DP_MUX_LPF			= 0x1, // Analog Front End output
		DP_MUX_RECTIFIER	= 0x2, // Rectifier output
		DP_MUX_BPF			= 0x3, // BPF output
		DP_MUX_ADC			= 0x4 // ADC output
	} DP_MUX_t;

	// Structure for TEST_MUX register (Address 0x4B)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							DP_MUX_t DP_MUX        : 3; // Data path multiplexer source select (bits 2:0)
							uint8_t SAMPLE_SEL     : 1; // Data path sample select (bit 3)
														// 0b = 8-bit sample output at 1 µs per sample
														// 1b = 12-bit sample output at 2 µs per sample
							uint8_t RESERVED       : 1; // Reserved bit (bit 4)
							TEST_MUX_Enum_t TEST_MUX : 3; // Multiplexer output on the TEST Pin (bits 7:5)
					} BitField;
			} Val;
	} TEST_MUX_t;

	// Structure for DEV_STAT0 register (Address 0x4C)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TRIM_CRC_ERR   : 1; // User EEPROM space trim data CRC error status (bit 0)
																					// 0 = No error
																					// 1 = CRC error detected
							uint8_t EE_CRC_ERR     : 1; // User EEPROM space data CRC error status (bit 1)
																					// 0 = No error
																					// 1 = CRC error detected
							uint8_t THR_CRC_ERR    : 1; // Threshold map configuration register data CRC error status (bit 2)
																					// 0 = No error
																					// 1 = CRC error detected
							uint8_t CMW_WU_ERR     : 1; // Wakeup Error indicator (bit 3)
																					// 0 = No error
																					// 1 = User tried to send a command before the wakeup sequence is done
							uint8_t OPT_ID         : 2; // Device Option Identification (bits 4:5)
							uint8_t REV_ID         : 2; // Device Revision Identification (bits 6:7)
					} BitField;
			} Val;
	} DEV_STAT0_t;

	// Structure for DEV_STAT1 register (Address 0x4D)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t VPWR_UV        : 1; // VPWR pin under voltage status (bit 0)
																					// 0 = No error
																					// 1 = VPWR under voltage error
							uint8_t VPWR_OV        : 1; // VPWR pin over voltage status (bit 1)
																					// 0 = No error
																					// 1 = VPWR over voltage error
							uint8_t AVDD_UV        : 1; // AVDD pin under voltage status (bit 2)
																					// 0 = No error
																					// 1 = AVDD under voltage error
							uint8_t AVDD_OV        : 1; // AVDD pin over voltage status (bit 3)
																					// 0 = No error
																					// 1 = AVDD over voltage error
							uint8_t IOREG_UV       : 1; // IOREG pin under voltage status (bit 4)
																					// 0 = No error
																					// 1 = IOREG under voltage error
							uint8_t IOREG_OV       : 1; // IOREG pin over voltage status (bit 5)
																					// 0 = No error
																					// 1 = IOREG over voltage error
							uint8_t TSD_PROT       : 1; // Thermal shut-down protection status (bit 6)
																					// 0 = No thermal shutdown has occurred
																					// 1 = Thermal shutdown has occurred
							uint8_t RESERVED       : 1; // Reserved bit (bit 7)
					} BitField;
			} Val;
	} DEV_STAT1_t;
	
// --- Thresholds structure ---
typedef struct __attribute__((packed)) {
    uint8_t P1_THR[16]; // 0x5F to 0x6E
    uint8_t P2_THR[16]; // 0x6F to 0x7E
    //uint8_t THR_CRC;    // 0x7F
} Thresholds_t;

typedef struct __attribute__((packed)) {
    uint16_t tof_us;   // Time-of-flight in microseconds (2 bytes)
    uint8_t width;		// Echo width (1 byte)
    uint8_t amplitude;	// Echo amplitude (1 byte)
} PGA460_Measure_t;

// View of EEPROM bulk image [0x00..0x2B] = userData + TVG + Settings (44 bytes)
typedef struct __attribute__((packed)) {
    uint8_t			UserData[20];	// 0x00..0x13
    TVG_t				TVG;					// 0x14..0x1A
    Settings_t	Sett;					// 0x1B..0x2B
} EEImage_t;

typedef struct __attribute__((packed)) {
    EEImage_t     EEData;     // 0x00..0x2B
    EE_CNTRL_t    eeCtrl;       // 0x40
    Filters_t     Filters;      // 0x41..0x4A
    TEST_MUX_t    TestMux;      // 0x4B
    DEV_STAT0_t   Stat0;      // 0x4C
    DEV_STAT1_t   Stat1;      // 0x4D
    Thresholds_t  THR;   // 0x5F..0x7F
} PGA460_Regs_t;

typedef struct __attribute__((packed)) {
  PGA460_Regs_t Registers; // PGA460 registers
	PGA460_Measure_t Measures; 
	UART_HandleTypeDef *uartPort;  // Matches the type of huart1, huart4, huart5
} PGA460_Sensor_t;

// AFE gain range codes for DECPL_TEMP[7:6] (per datasheet):
// 00b = 58–90 dB, 01b = 52–84 dB, 10b = 46–78 dB, 11b = 32–64 dB
typedef enum {
    PGA460_GAIN_32_64dB = 0xC0,  // 11b << 6
    PGA460_GAIN_46_78dB = 0x80,  // 10b << 6
    PGA460_GAIN_52_84dB = 0x40,  // 01b << 6
    PGA460_GAIN_58_90dB = 0x00   // 00b << 6
} PGA460_GainRange_t;

typedef enum {
    PGA460_CMD_GET_TEMP = 0x00,
    PGA460_CMD_GET_NOISE = 0x01
} PGA460_CmdType_t;

typedef struct __attribute__((packed)) { 
    float Speed;      // m/s
    float Direction;  // deg FROM (0/360=N, 90=E, 180=S, 270=W)
} PGA460_Wind_t;

// Public Function Prototypes
void calculateSpeedOfSound(void);
void calculateWind(uint32_t ToF_up[3], uint32_t ToF_down[3], float speed_of_sound, float *wind_speed, float *wind_direction);


/* -------------------------------------------------
 * Initialization Functions
 * ------------------------------------------------- */
HAL_StatusTypeDef PGA460_Init(uint8_t burnEEPROM);

/* -------------------------------------------------
 * Register Read/Write Functions
 * ------------------------------------------------- */
HAL_StatusTypeDef PGA460_RegisterRead(uint8_t sensorID, uint8_t regAddr, uint8_t *regValue);
HAL_StatusTypeDef PGA460_RegisterWrite(uint8_t sensorID, uint8_t regAddr, uint8_t regValue);

HAL_StatusTypeDef PGA460_CheckStatus(const uint8_t sensorID);

/* -------------------------------------------------
 * EEPROM Functions
 * ------------------------------------------------- */
HAL_StatusTypeDef PGA460_EEPROMBulkRead(const uint8_t sensorID);
HAL_StatusTypeDef PGA460_EEPROMBulkWrite(uint8_t sensorID);
HAL_StatusTypeDef PGA460_BurnEEPROM(uint8_t sensorID);

HAL_StatusTypeDef PGA460_GetTVG(uint8_t sensorID);
HAL_StatusTypeDef PGA460_SetTVG(uint8_t sensorID, PGA460_GainRange_t gain_range, const TVG_t *tvg);

HAL_StatusTypeDef PGA460_GetThresholds(uint8_t sensorID);
HAL_StatusTypeDef PGA460_SetThresholds(uint8_t sensorID);

HAL_StatusTypeDef PGA460_UltrasonicCmd(const uint8_t sensorID, const PGA460_Command_t cmd);

HAL_StatusTypeDef PGA460_AutoThreshold(uint8_t sensorID, uint8_t noiseMargin, uint8_t windowIndex, uint8_t autoMax, uint8_t loops);
HAL_StatusTypeDef PGA460_AutoThreshold_Internal(uint8_t sensorID, PGA460_Command_t cmd, uint8_t noiseMargin, uint8_t windowIndex, uint8_t autoMax, uint8_t loops, Thresholds_t *thr);

HAL_StatusTypeDef PGA460_GetUltrasonicMeasurement(const uint8_t sensorID);
HAL_StatusTypeDef PGA460_GetEchoDataDump(uint8_t sensorID, uint8_t preset, uint8_t *echoOut);

HAL_StatusTypeDef PGA460_SetTemperatureOffset(uint8_t sensorID, float externalTempC);
HAL_StatusTypeDef PGA460_GetSystemDiagnostics(const uint8_t sensorID, const uint8_t run, const uint8_t diag, float *diagResult);

float PGA460_ReadTemperatureOrNoise(const uint8_t sensorID, const PGA460_CmdType_t mode);

TVG_t PGA460_AutoTVG(uint8_t sensorID, PGA460_Command_t preset, uint8_t targetAmp, uint8_t noiseGuard, uint8_t loops);
HAL_StatusTypeDef PGA460_AutoTune(uint8_t sensorID, uint8_t noiseMargin, uint8_t windowIndex, uint8_t autoMax, uint8_t loops, uint8_t targetAmp);
HAL_StatusTypeDef PGA460_AutoWindSetup(uint8_t loops, uint8_t windowIndex, uint8_t noiseMargin, uint8_t autoMax, uint8_t targetAmp);
HAL_StatusTypeDef PGA460_MeasureWind(PGA460_Wind_t *out);

#endif // PGA460_H
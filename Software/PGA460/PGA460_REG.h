#ifndef PGA460_REG_H
#define PGA460_REG_H

	#include <stdint.h>
	#include "usart.h"

	#define PGA_CMD_SIZE 3
	#define PGA_READ_SIZE 4
	#define PGA_WRITE_SIZE 5
	#define PGA_CMD_COUNT 26
	// PGA460 supports up to 8 objects per measurement
	#define PGA_MAX_OBJECTS					8
	// Number of objects we want to track (Must be ≤ PGA_MAX_OBJECTS)
	#define PGA_OBJECTS_TRACKED    8  

	#if PGA_OBJECTS_TRACKED > PGA_MAX_OBJECTS
			#error "PGA_OBJECTS_TRACKED cannot be greater than PGA_MAX_OBJECTS!"
	#endif
	#define PGA_OBJ_DATA_SIZE (2 + (PGA_OBJECTS_TRACKED * 4))

	// Number of ultrasonic sensors in the array
	#define ULTRASONIC_SENSOR_COUNT 3	// Number os sensors
	// Error value for temperature or noise
	#define PGA460_TEMP_ERR					999.0f

	//#define USE_MA4S4S_R				// Murata MA4S4S/R Configuration
	//#define USE_MA40H1SR				// Murata MA40H1SR Configuration
	//#define USE_MA58MF14_7N			// Murata MA58MF14-7N Configuration
	//#define USE_UTR_1440K_TT_R	// PUI Audio UTR-1440K-TT-R configuration
	
	// Address 0h-2Bh: EEPROM nonvolatile memory.
	// Content in these registers is preserved during power cycle and low-power mode.
	// Address 40h-4Dh and address 5Fh-7Fh: Register-based volatile memory.
	// Content in these registers is lost during power cycle and low-power mode.
	// Address 2Ch-3Fh and address 4Eh-5Eh are reserved for Texas Instruments internal use and are not accessible to the user.

	// Sync byte for baud rate auto-detection
	#define PGA460_SYNC						0x55  
	#define PGA460_UNLOCK_EEPROM	0x68
	#define PGA460_LOCK_EEPROM		0x69
	#define ECHO_DATA_DUMP_ENABLE  0x80
	#define ECHO_DATA_DUMP_DISABLE 0x00
	#define ECHO_DATA_TOTAL_BYTES  130  // 2 bytes header + 128 data
	
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

	// Enumeration for Gain Time Points
	typedef enum {
		GAIN_TIME_100us = 0x0,
		GAIN_TIME_200us,
		GAIN_TIME_300us,
		GAIN_TIME_400us,
		GAIN_TIME_600us,
		GAIN_TIME_800us,
		GAIN_TIME_1000us,
		GAIN_TIME_1200us,
		GAIN_TIME_1400us,
		GAIN_TIME_2000us,
		GAIN_TIME_2400us,
		GAIN_TIME_3200us,
		GAIN_TIME_4000us,
		GAIN_TIME_5200us,
		GAIN_TIME_6400us,
		GAIN_TIME_8000us
	} GainTime_t;

	// Structure for TVGAIN0 register (Address 0x14)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TVG_T0 : 4; // Time-varying gain start time (bits 7:4)
							GainTime_t TVG_T1 : 4; // Time-varying gain T0/T1 delta time (bits 3:0)
					} BitField;
			} Val;
	} TVGAIN0_t;

	// Structure for TVGAIN1 register (Address 0x15)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TVG_T2 : 4; // Time-varying gain T1/T2 delta time (bits 7:4)
							GainTime_t TVG_T3 : 4; // Time-varying gain T2/T3 delta time (bits 3:0)
					} BitField;
			} Val;
	} TVGAIN1_t;

	// Structure for TVGAIN2 register (Address 0x16)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TVG_T4 : 4; // Time-varying gain T3/T4 delta time (bits 7:4)
							GainTime_t TVG_T5 : 4; // Time-varying gain T4/T5 delta time (bits 3:0)
					} BitField;
			} Val;
	} TVGAIN2_t;

	// Structure for TVGAIN3 register (Address 0x17)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TVG_G1 : 6; // Gain value at point 1 (bits 7:2) Gain = 0.5 × (TVG_G1 + 1) + value(AFE_GAIN_RNG) [dB]
							uint8_t RESERVED : 2; // Reserved bits (bits 1:0)
					} BitField;
			} Val;
	} TVGAIN3_t;

	// Structure for TVGAIN4 register (Address 0x18)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TVG_G2 : 4; // Gain value at point 2 (bits 7:4) Gain = 0.5 × (TVG_G2 +1) + value(AFE_GAIN_RNG) [dB]
							uint8_t TVG_G3 : 4; // Gain value at point 3 (bits 3:0) Gain = 0.5 × (TVG_G3 + 1) + value(AFE_GAIN_RNG) [dB]
					} BitField;
			} Val;
	} TVGAIN4_t;

	// Structure for TVGAIN5 register (Address 0x19)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TVG_G3 : 2; // Gain value at point 3 (bits 7:6) Gain = 0.5 × (TVG_G3 +1) + value(AFE_GAIN_RNG) [dB]
							uint8_t TVG_G4 : 6; // Gain value at point 4 (bits 5:0) Gain = 0.5 × (TVG_G4 + 1) + value(AFE_GAIN_RNG) [dB]
					} BitField;
			} Val;
	} TVGAIN5_t;

	// Structure for TVGAIN6 register (Address 0x1A)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TVG_G5    : 6; // Gain value at point 5 (bits 7:2) Gain = 0.5 × (TVG_G5 +1) + value(AFE_GAIN_RNG) [dB]
							uint8_t RESERVED  : 1; // Reserved bit (bit 1)
							uint8_t FREQ_SHIFT: 1; // Burst frequency range shift (bit 0) 0b = Disabled 1b = Enabled
					} BitField;
			} Val;
	} TVGAIN6_t;

	// Structure for INIT_GAIN register (Address 0x1B)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t GAIN_INIT : 6; // Initial AFE Gain (bits 5:0) Init_Gain = 0.5 × (GAIN_INIT+1) + value(AFE_GAIN_RNG) [dB]
							uint8_t BPF_BW    : 2; // Digital bandpass filter bandwidth (bits 7:6) BandWidth = 2 × (BPF_BW + 1) [kHz]
					} BitField;
			} Val;
	} INIT_GAIN_t;

	// Structure for DEADTIME register (Address 0x1D)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t PULSE_DT        : 4; // Burst Pulse Dead-Time (bits 3:0) DeadTime = 0.0625 × PULSE_DT[μs]
							uint8_t THR_CMP_DEGLTCH : 4; // Threshold level comparator deglitch period (bits 7:4) deglitch period = (THR_CMP_DEGLITCH × 8) [μs]
					} BitField;
			} Val;
	} DEADTIME_t;

	// Structure for PULSE_P1 register (Address 0x1E)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t P1_PULSE   : 5; // Number of burst pulses for Preset1 (bits 4:0) 0h means one pulse is generated on OUTA only
							uint8_t IO_DIS     : 1; // Disable IO pin transceiver (bit 5) 0b = IO transceiver enabled 1b = IO transceiver disabled Note: Available only if IO_IF_SEL = 0
							uint8_t UART_DIAG  : 1; // UART Diagnostic Page Selection (bit 6) 0b = Diagnostic bits related to UART interface 1b = Diagnostic bits related to System Diagnostics
							uint8_t IO_IF_SEL  : 1; // Interface Selection on IO pin (bit 7) 0b = Time-Based Interface 1b = One-Wire UART Interface
					} BitField;
			} Val;
	} PULSE_P1_t;

	// Structure for PULSE_P2 register (Address 0x1F)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t P2_PULSE  : 5; // Number of burst pulses for Preset2 (bits 4:0) 0h means one pulse is generated on OUTA only
							uint8_t UART_ADDR : 3; // UART interface address (bits 7:5)
					} BitField;
			} Val;
	} PULSE_P2_t;

	// Structure for CURR_LIM_P1 register (Address 0x20)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t CURR_LIM1 : 6; // Driver Current Limit for Preset1 (bits 5:0) Current_Limit = 7 × CURR_LIM1 + 50 [mA]
							uint8_t Reserved  : 1; // Reserved bit
							uint8_t DIS_CL    : 1; // Disable Current Limit for Preset1 and Preset2 (bit 7) 0b = current limit enabled 1b = current limit disabled
					} BitField;
			} Val;
	} CURR_LIM_P1_t;

	// Structure for CURR_LIM_P2 register (Address 0x21)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t CURR_LIM2 : 6; // Driver Current Limit for Preset2 (bits 5:0) Current limit = 7 × CURR_LIM2 + 50 [mA]
							uint8_t LPF_CO    : 2; // Lowpass filter cutoff frequency (bits 7:6) Cut off frequency = LPF_CO + 1 [kHz]
					} BitField;
			} Val;
	} CURR_LIM_P2_t;

	// Structure for REC_LENGTH register (Address 0x22)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t P2_REC : 4; // Preset2 record time length (bits 3:0) Record time = 4.096 × (P2_REC + 1) [ms]
							uint8_t P1_REC : 4; // Preset1 record time length (bits 7:4) Record time = 4.096 × (P1_REC + 1) [ms]
					} BitField;
			} Val;
	} REC_LENGTH_t;

	// Structure for FREQ_DIAG register (Address 0x23)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t FDIAG_START : 4; // Frequency diagnostic start time (bits 3:0) Start time = 100 × FDIAG_START [μs]
							uint8_t FDIAG_LEN   : 4; // Frequency diagnostic window length (bits 7:4) 0h, the diagnostic is disabled. 0 to Fh 3 × FDIAG_LEN [Signal Periods]
					} BitField;
			} Val;
	} FREQ_DIAG_t;

	// Structure for SAT_FDIAG_TH register (Address 0x24)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t P1_NLS_EN    : 1; // Enable Preset1 non-linear scaling (bit 0) Set high to enable Preset1 non-linear scaling
							uint8_t SAT_TH       : 4; // Saturation diagnostic threshold level (bits 4:1)
							uint8_t FDIAG_ERR_TH : 3; // Frequency diagnostic absolute error time threshold (bits 7:5) threshold = (FDIAG_ERR_TH + 1) [μs]
					} BitField;
			} Val;
	} SAT_FDIAG_TH_t;

	// Enum for VPWR_OV_TH field (bits 6:5)
	typedef enum {
		VPWR_OV_12_3V = 0x0,	// 12.3 V threshold
		VPWR_OV_17_7V = 0x1,	// 17.7 V threshold
		VPWR_OV_22_8V = 0x2,	// 22.8 V threshold
		VPWR_OV_28_3V = 0x3		// 28.3 V threshold
	} VPWR_OV_TH_t;

	// Enum for LPM_TMR field (bits 4:3)
	typedef enum {
		LPM_250ms	= 0x0,	// 250 ms
		LPM_500ms	= 0x1,	// 500 ms
		LPM_1s		= 0x2,	// 1 s
		LPM_4s		= 0x3		// 4 s
	} LPM_TMR_t;

	// Enum for FVOLT_ERR_TH field (bits 2:0)
	typedef enum {
		FVOLT_ERR_1 = 0x0, // Error threshold level 1
		FVOLT_ERR_2 = 0x1, // Error threshold level 2
		FVOLT_ERR_3 = 0x2, // Error threshold level 3
		FVOLT_ERR_4 = 0x3, // Error threshold level 4
		FVOLT_ERR_5 = 0x4, // Error threshold level 5
		FVOLT_ERR_6 = 0x5, // Error threshold level 6
		FVOLT_ERR_7 = 0x6, // Error threshold level 7
		FVOLT_ERR_8 = 0x7  // Error threshold level 8
	} FVOLT_ERR_TH_t;

	// Structure for FVOLT_DEC register (Address 0x25)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							FVOLT_ERR_TH_t FVOLT_ERR_TH : 3; // Voltage diagnostic threshold (bits 2:0)
							LPM_TMR_t LPM_TMR           : 2; // Sleep mode timer (bits 4:3)
							VPWR_OV_TH_t VPWR_OV_TH     : 2; // VPWR overvoltage threshold select (bits 6:5)
							uint8_t P2_NLS_EN           : 1; // Enable Preset2 non-linear scaling (bit 7)
					} BitField;
			} Val;
	} FVOLT_DEC_t;

	// Enum for AFE_GAIN_RNG field (bits 7:6)
	typedef enum {
		AFE_GAIN_58_90dB = 0x0, // 58 to 90 dB
		AFE_GAIN_52_84dB = 0x1, // 52 to 84 dB
		AFE_GAIN_46_78dB = 0x2, // 46 to 78 dB
		AFE_GAIN_32_64dB = 0x3  // 32 to 64 dB
	} AFE_GAIN_RNG_t;

	// Structure for DECPL_TEMP register (Address 0x26)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t DECPL_T          : 4; // Decouple time and temperature value (bits 3:0)
																						// If DECPL_TEMP_SEL = 0 (Time Decouple): Time = 4096 × (DECPL_T + 1) [μs]
																						// If DECPL_TEMP_SEL = 1 (Temperature Decouple): Temperature = 10 × DECPL_T - 40 [degC]
							uint8_t DECPL_TEMP_SEL   : 1; // Decouple time and temperature select (bit 4)
																						// 0b = Time Decouple
																						// 1b = Temperature Decouple
							uint8_t LPM_EN           : 1; // Low-power mode enable (bit 5)
																						// 0b = Low power mode is disabled
																						// 1b = Low power mode is enabled
							AFE_GAIN_RNG_t AFE_GAIN_RNG : 2; // AFE gain range (bits 7:6)
					} BitField;
			} Val;
	} DECPL_TEMP_t;

	// Enum for SCALE_N field (bits 1:0)
	typedef enum {
		SCALE_TH9		= 0x0,	// Starting threshold level at TH9
		SCALE_TH10	= 0x1,	// Starting threshold level at TH10
		SCALE_TH11	= 0x2,	// Starting threshold level at TH11
		SCALE_TH12	= 0x3		// Starting threshold level at TH12
	} SCALE_N_t;

	// Structure for DSP_SCALE register (Address 0x27)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t NOISE_LVL : 5; // Non-linear scaling noise level (bits 4:0)
																		 // Value ranges from 0 to 31 with 1 LSB steps for digital gain values (Px_DIG_GAIN_LR) less than 8
																		 // If digital gain (Px_DIG_GAIN_LR) is larger than 8, then multiply the NOISE_LVL by Px_DIG_GAIN_LR/8
							uint8_t SCALE_K   : 1; // Non-linear scaling exponent (bit 5)
																		 // 0b = 1.50
																		 // 1b = 2.00
							SCALE_N_t SCALE_N : 2; // Non-linear scaling time offset (bits 7:6)
					} BitField;
			} Val;
	} DSP_SCALE_t;

	// Structure for TEMP_TRIM register (Address 0x28)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TEMP_OFF  : 4; // Temperature-scale offset (bits 3:0)
																		 // Signed value can range from -8 (1000b) to 7 (0111b)
																		 // Used for measured temperature value compensation
							uint8_t TEMP_GAIN : 4; // Temperature-scale gain (bits 7:4)
																		 // Signed value can range from -8 (1000b) to 7 (0111b)
																		 // Used for measured temperature value compensation
					} BitField;
			} Val;
	} TEMP_TRIM_t;

	// Enum for P1_DIG_GAIN_LR_ST, P2_DIG_GAIN_LR_ST field (bits 7:6) 
	typedef enum {
		DIG_GAIN_LR_ST_TH9	= 0x0,	// Starting threshold level at TH9
		DIG_GAIN_LR_ST_TH10 = 0x1,	// Starting threshold level at TH10
		DIG_GAIN_LR_ST_TH11 = 0x2,	// Starting threshold level at TH11
		DIG_GAIN_LR_ST_TH12 = 0x3		// Starting threshold level at TH12
	} DIG_GAIN_LR_ST_t;

	// Enum for P1_DIG_GAIN_LR, P1_DIG_GAIN_SR, P2_DIG_GAIN_LR, P2_DIG_GAIN_SR fields
	typedef enum {
		DIG_GAIN_VAL_MULT_1		= 0x0,	// Multiplied by 1
		DIG_GAIN_VAL_MULT_2		= 0x1,	// Multiplied by 2
		DIG_GAIN_VAL_MULT_4		= 0x2,	// Multiplied by 4
		DIG_GAIN_VAL_MULT_8		= 0x3,	// Multiplied by 8
		DIG_GAIN_VAL_MULT_16	= 0x4,	// Multiplied by 16
		DIG_GAIN_VAL_MULT_32	= 0x5,	// Multiplied by 32
		DIG_GAIN_VAL_INVALID	= 0x6,	// Invalid
		DIG_GAIN_VAL_INVALID2 = 0x7		// Invalid
	} DIG_GAIN_VAL_t;

	// Structure for P1_GAIN_CTRL register (Address 0x29)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							DIG_GAIN_VAL_t P1_DIG_GAIN_SR   : 3; // P1 digital short-range gain (bits 2:0)
							DIG_GAIN_VAL_t P1_DIG_GAIN_LR   : 3; // P1 digital long-range gain (bits 5:3)
							DIG_GAIN_LR_ST_t P1_DIG_GAIN_LR_ST : 2; // P1 digital gain start threshold (bits 7:6)
					} BitField;
			} Val;
	} P1_GAIN_CTRL_t;

	// Structure for P2_GAIN_CTRL register (Address 0x2A)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							DIG_GAIN_VAL_t P2_DIG_GAIN_SR   : 3; // P2 digital short-range gain (bits 2:0)
							DIG_GAIN_VAL_t P2_DIG_GAIN_LR   : 3; // P2 digital long-range gain (bits 5:3)
							DIG_GAIN_LR_ST_t P2_DIG_GAIN_LR_ST : 2; // P2 digital gain start threshold (bits 7:6)
					} BitField;
			} Val;
	} P2_GAIN_CTRL_t;

	// Start of Volatile data registers

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

	// Enum for TEST_MUX field (bits 7:5)
	typedef enum {
		TEST_MUX_GND				= 0x0,	// GND ("Mux Off")
		TEST_MUX_AFE_OUTPUT	= 0x1,	// Analog Front End output
		TEST_MUX_RESERVED1	= 0x2,	// Reserved
		TEST_MUX_RESERVED2	= 0x3,	// Reserved
		TEST_MUX_8MHZ_CLOCK	= 0x4,	// 8MHz clock
		TEST_MUX_ADC_SAMPLE	= 0x5,	// ADC sample output clock
		TEST_MUX_RESERVED3	= 0x6,	// Reserved
		TEST_MUX_RESERVED4	= 0x7		// Reserved
	} TEST_MUX_Enum_t;

	// Enum for DP_MUX field (bits 2:0)
	typedef enum {
		DP_MUX_OFF				= 0x0, // Disabled
		DP_MUX_LPF				= 0x1, // Analog Front End output
		DP_MUX_RECTIFIER	= 0x2, // Rectifier output
		DP_MUX_BPF				= 0x3, // BPF output
		DP_MUX_ADC				= 0x4, // ADC output
		DP_MUX_NOT_USED1	= 0x5, // Not used
		DP_MUX_NOT_USED2	= 0x6, // Not used
		DP_MUX_NOT_USED3	= 0x7  // Not used
	} DP_MUX_t;

	// Structure for TEST_MUX register (Address 0x4B)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							DP_MUX_t DP_MUX        : 3; // Data path multiplexer source select (bits 2:0)
							uint8_t SAMPLE_SEL     : 1; // Data path sample select (bit 3)
																				 // 0b = 8-bit sample output at 1 μs per sample
																				 // 1b = 12-bit sample output at 2 μs per sample
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

	// Structure for P1_THR_0 register (Address 0x5F)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TH_P1_T1 : 4; // Preset1 Threshold T1 absolute time (bits 7:4)
							GainTime_t TH_P1_T2 : 4; // Preset1 Threshold T2 delta time (bits 3:0)
					} BitField;
			} Val;
	} P1_THR_0_t;

	// Structure for P1_THR_1 register (Address 0x60)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TH_P1_T3 : 4; // Preset1 Threshold T3 absolute time (bits 7:4)
							GainTime_t TH_P1_T4 : 4; // Preset1 Threshold T4 delta time (bits 3:0)
					} BitField;
			} Val;
	} P1_THR_1_t;

	// Structure for P1_THR_2 register (Address 0x61)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TH_P1_T5 : 4; // Preset1 Threshold T5 absolute time (bits 7:4)
							GainTime_t TH_P1_T6 : 4; // Preset1 Threshold T6 delta time (bits 3:0)
					} BitField;
			} Val;
	} P1_THR_2_t;

	// Structure for P1_THR_3 register (Address 0x62)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TH_P1_T7 : 4; // Preset1 Threshold T7 absolute time (bits 7:4)
							GainTime_t TH_P1_T8 : 4; // Preset1 Threshold T8 delta time (bits 3:0)
					} BitField;
			} Val;
	} P1_THR_3_t;

	// Structure for P1_THR_4 register (Address 0x63)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TH_P1_T9  : 4; // Preset1 Threshold T9 absolute time (bits 7:4)
							GainTime_t TH_P1_T10 : 4; // Preset1 Threshold T10 delta time (bits 3:0)
					} BitField;
			} Val;
	} P1_THR_4_t;

	// Structure for P1_THR_5 register (Address 0x64)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TH_P1_T11 : 4; // Preset1 Threshold T11 absolute time (bits 7:4)
							GainTime_t TH_P1_T12 : 4; // Preset1 Threshold T12 delta time (bits 3:0)
					} BitField;
			} Val;
	} P1_THR_5_t;

	// Structure for P1_THR_6 register (Address 0x65)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TH_P1_L1 : 5; // Preset1 Threshold L1 level (bits 7:3)
							uint8_t TH_P1_L2 : 3; // Preset1 Threshold L2 level bits [Bit4 to Bit2]. (bits 2:0)
					} BitField;
			} Val;
	} P1_THR_6_t;

	// Structure for P1_THR_7 register (Address 0x66)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TH_P1_L2 : 2; // Preset1 Threshold L2 level [Bit1 to Bit0]. (bits 7:6)
							uint8_t TH_P1_L3 : 5; // Preset1 Threshold L3 level (bits 5:1)
							uint8_t TH_P1_L4 : 1; // Preset1 Threshold L4 level [Bit4]. (bit 0)
					} BitField;
			} Val;
	} P1_THR_7_t;

	// Structure for P1_THR_8 register (Address 0x67)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TH_P1_L4 : 4; // Preset1 Threshold L4 level [Bits3 to Bit0]. (bits 7:4)
							uint8_t TH_P1_L5 : 4; // Preset1 Threshold L5 level [Bit4 to Bit1]. (bits 3:0)
					} BitField;
			} Val;
	} P1_THR_8_t;

	// Structure for P1_THR_9 register (Address 0x68)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TH_P1_L5 : 1; // Preset1 Threshold L5 level [Bit0]. (bit 7)
							uint8_t TH_P1_L6 : 5; // Preset1 Threshold L6 level (bits 6:2)
							uint8_t TH_P1_L7 : 2; // Preset1 Threshold L7 level [Bits4 to Bit3] (bits 1:0)
					} BitField;
			} Val;
	} P1_THR_9_t;

	// Structure for P1_THR_10 register (Address 0x69)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TH_P1_L7 : 3; // Preset1 Threshold L7 Level [Bit2 to Bit0]. (bits 7:5)
							uint8_t TH_P1_L8 : 5; // Preset1 Threshold L8 level (bits 4:0)
					} BitField;
			} Val;
	} P1_THR_10_t;

	// Structure for P1_THR_15 register (Address 0x6E)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t Reserved : 4;  // (bits 7:4)
							uint8_t TH_P1_OFF : 4; // Preset1 Threshold level Offset with values from 7 to -8 using signed magnitude representation with MSB as the sign bit (bits 3:0)
					} BitField;
			} Val;
	} P1_THR_15_t;

	// Structure for P2_THR_0 register (Address 0x6F)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TH_P2_T1 : 4; // Preset2 Threshold T1 absolute time (bits 7:4)
							GainTime_t TH_P2_T2 : 4; // Preset2 Threshold T2 delta time (bits 3:0)
					} BitField;
			} Val;
	} P2_THR_0_t;

	// Structure for P2_THR_1 register (Address 0x70)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TH_P2_T3 : 4; // Preset2 Threshold T3 absolute time (bits 7:4)
							GainTime_t TH_P2_T4 : 4; // Preset2 Threshold T4 delta time (bits 3:0)
					} BitField;
			} Val;
	} P2_THR_1_t;

	// Structure for P2_THR_2 register (Address 0x71)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TH_P2_T5 : 4; // Preset2 Threshold T5 absolute time (bits 7:4)
							GainTime_t TH_P2_T6 : 4; // Preset2 Threshold T6 delta time (bits 3:0)
					} BitField;
			} Val;
	} P2_THR_2_t;


	// Structure for P2_THR_3 register (Address 0x72)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TH_P2_T7 : 4; // Preset2 Threshold T7 absolute time (bits 7:4)
							GainTime_t TH_P2_T8 : 4; // Preset2 Threshold T8 delta time (bits 3:0)
					} BitField;
			} Val;
	} P2_THR_3_t;

	// Structure for P2_THR_4 register (Address 0x73)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TH_P2_T9  : 4; // Preset2 Threshold T9 absolute time (bits 7:4)
							GainTime_t TH_P2_T10 : 4; // Preset2 Threshold T10 delta time (bits 3:0)
					} BitField;
			} Val;
	} P2_THR_4_t;

	// Structure for P2_THR_5 register (Address 0x74)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							GainTime_t TH_P2_T11 : 4; // Preset2 Threshold T11 absolute time (bits 7:4)
							GainTime_t TH_P2_T12 : 4; // Preset2 Threshold T12 delta time (bits 3:0)
					} BitField;
			} Val;
	} P2_THR_5_t;

	// Structure for P2_THR_6 register (Address 0x75)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TH_P2_L1 : 5; // Preset2 Threshold L1 level (bits 7:3)
							uint8_t TH_P2_L2 : 3; // Preset2 Threshold L2 level [Bit4 to Bit2]. (bits 2:0)
					} BitField;
			} Val;
	} P2_THR_6_t;

	// Structure for P2_THR_7 register (Address 0x76)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TH_P2_L2 : 2; // Preset2 Threshold L2 level [Bit1 to Bit0]. (bits 7:6)
							uint8_t TH_P2_L3 : 5; // Preset2 Threshold L3 level (bits 5:1)
							uint8_t TH_P2_L4 : 1; // Preset2 Threshold L4 level [Bit4]. (bit 0)
					} BitField;
			} Val;
	} P2_THR_7_t;

	// Structure for P2_THR_8 register (Address 0x77)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TH_P2_L4 : 4; // Preset2 Threshold L4 level [Bit3 to Bit0]. (bits 7:4)
							uint8_t TH_P2_L5 : 4; // Preset2 Threshold L5 level [Bit4 to Bit1]. (bits 3:0)
					} BitField;
			} Val;
	} P2_THR_8_t;

	// Structure for P2_THR_9 register (Address 0x78)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TH_P2_L5 : 1; // Preset2 Threshold L5 level [Bit0]. (bit 7)
							uint8_t TH_P2_L6 : 5; // Preset2 Threshold L6 level (bits 6:2)
							uint8_t TH_P2_L7 : 2; // Preset2 Threshold L7 level [Bit4 to Bit3]. (bits 1:0)
					} BitField;
			} Val;
	} P2_THR_9_t;

	// Structure for P2_THR_10 register (Address 0x79)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t TH_P2_L7 : 3; // Preset2 Threshold L7 level [Bit2 to Bit0]. (bits 7:5)
							uint8_t TH_P2_L8 : 5; // Preset2 Threshold L8 level (bits 4:0)
					} BitField;
			} Val;
	} P2_THR_10_t;

	// Structure for P2_THR_15 register (Address 0x7E)
	typedef struct __attribute__((packed)) {
			union {
					uint8_t Value;
					struct __attribute__((packed)) {
							uint8_t Reserved : 4; // Reserved bits (bits 7:4)
							uint8_t TH_P2_OFF : 4; // Preset2 Threshold level Offset (bits 3:0)
					} BitField;
			} Val;
	} P2_THR_15_t;

	typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t DEVICE_BUSY       : 1; // Bit 0: PGA460 Device Busy
            uint8_t BAUD_RATE_ERROR   : 1; // Bit 1: Sync field bit rate too high/low
            uint8_t SYNC_WIDTH_ERROR  : 1; // Bit 2: Consecutive sync field bit widths do not match
            uint8_t CHECKSUM_ERROR    : 1; // Bit 3: Invalid checksum received
            uint8_t COMMAND_ERROR     : 1; // Bit 4: Invalid command sent from controller
            uint8_t GENERAL_ERROR     : 1; // Bit 5: General communication error (details below)
            uint8_t EEPROM_CRC_ERROR  : 1; // Bit 6: EEPROM CRC error or TRM CRC error
            uint8_t RESERVED          : 1; // Bit 7: Reserved (Logic 0)
        } BitField;
    } Val;
} PGA460_Diagnostic_t;

typedef struct __attribute__((packed)) {
		uint8_t USER_DATA1;						// Address 0x00
		uint8_t USER_DATA2;						// Address 0x01
		uint8_t USER_DATA3;						// Address 0x02
		uint8_t USER_DATA4;						// Address 0x03
		uint8_t USER_DATA5;						// Address 0x04
		uint8_t USER_DATA6;						// Address 0x05
		uint8_t USER_DATA7;						// Address 0x06
		uint8_t USER_DATA8;						// Address 0x07
		uint8_t USER_DATA9;						// Address 0x08
		uint8_t USER_DATA10;					// Address 0x09
		uint8_t USER_DATA11;					// Address 0x0A
		uint8_t USER_DATA12;					// Address 0x0B
		uint8_t USER_DATA13;					// Address 0x0C
		uint8_t USER_DATA14;					// Address 0x0D
		uint8_t USER_DATA15;					// Address 0x0E
		uint8_t USER_DATA16;					// Address 0x0F
		uint8_t USER_DATA17;					// Address 0x10
		uint8_t USER_DATA18;					// Address 0x11
		uint8_t USER_DATA19;					// Address 0x12
		uint8_t USER_DATA20;					// Address 0x13
} PGA460_UserData_t;

typedef struct __attribute__((packed)) {
		TVGAIN0_t TVGAIN0;						// Address 0x14
		TVGAIN1_t TVGAIN1;						// Address 0x15
		TVGAIN2_t TVGAIN2;						// Address 0x16
		TVGAIN3_t TVGAIN3;						// Address 0x17
		TVGAIN4_t TVGAIN4;						// Address 0x18
		TVGAIN5_t TVGAIN5;						// Address 0x19
		TVGAIN6_t TVGAIN6;						// Address 0x1A
} PGA460_TVG_t;

typedef struct __attribute__((packed)) {
		PGA460_UserData_t USER;				// Address 0x00 0x13
		PGA460_TVG_t TGV;							// Address 0x14 0x1A 
		INIT_GAIN_t INIT_GAIN;				// Address 0x1B
		uint8_t FREQ;									// Burst frequency equation parameter (Address 0x1C) Frequency = 0.2 × FREQ + 30 [kHz]
		DEADTIME_t DEADTIME;					// Address 0x1D
		PULSE_P1_t PULSE_P1;					// Address 0x1E
		PULSE_P2_t PULSE_P2;					// Address 0x1F
		CURR_LIM_P1_t CURR_LIM_P1;		// Address 0x20
		CURR_LIM_P2_t CURR_LIM_P2;		// Address 0x21
		REC_LENGTH_t REC_LENGTH;			// Address 0x22
		FREQ_DIAG_t FREQ_DIAG;				// Address 0x23
		SAT_FDIAG_TH_t SAT_FDIAG_TH;	// Address 0x24
		FVOLT_DEC_t FVOLT_DEC;				// Address 0x25
		DECPL_TEMP_t DECPL_TEMP;			// Address 0x26
		DSP_SCALE_t DSP_SCALE;				// Address 0x27
		TEMP_TRIM_t TEMP_TRIM;				// Address 0x28
		P1_GAIN_CTRL_t P1_GAIN_CTRL;	// Address 0x29
		P2_GAIN_CTRL_t P2_GAIN_CTRL;	// Address 0x2A
		uint8_t EE_CRC; 							// User EEPROM space CRC value (bits 7:0) EE_CRC register (Address 0x2B)
} PGA460_EEPROMConfig_t;

typedef struct __attribute__((packed)) {
		uint8_t BPF_A2_MSB;						// Bandpass filter A2 coefficient most-significant byte value (Address 0x41)
		uint8_t BPF_A2_LSB;						// Bandpass filter A2 coefficient least-significant byte value (Address 0x42)
		uint8_t BPF_A3_MSB;						// Bandpass filter A3 coefficient most-significant byte value (Address 0x43)
		uint8_t BPF_A3_LSB;						// Bandpass filter A3 coefficient least-significant byte value (Address 0x44)
		uint8_t BPF_B1_MSB;						// Bandpass filter B1 coefficient most-significant byte value (Address 0x45)
		uint8_t BPF_B1_LSB;						// Bandpass filter B1 coefficient least-significant byte value (Address 0x46)
} PGA460_BPF_t;

typedef struct __attribute__((packed)) {
		uint8_t LPF_A2_MSB;						// Lowpass filter A2 coefficient most-significant byte value (Address 0x47) only 7 bits
		uint8_t LPF_A2_LSB;						// Lowpass filter A2 coefficient least-significant byte value (Address 0x48)
		uint8_t LPF_B1_MSB;						// Lowpass filter A2 coefficient most-significant byte value (Address 0x49) only 7 bits
		uint8_t LPF_B1_LSB;						// Lowpass filter A2 coefficient least-significant byte value (Address 0x4A)
} PGA460_LPF_t;

typedef struct __attribute__((packed)) {
		P1_THR_0_t P1_THR_0;					// P1_THR_0 Register (Address = 5Fh)
		P1_THR_1_t P1_THR_1;					// P1_THR_1 Register (Address = 60h)
		P1_THR_2_t P1_THR_2;					// P1_THR_2 Register (Address = 61h)
		P1_THR_3_t P1_THR_3;					// P1_THR_3 Register (Address = 62h)	
		P1_THR_4_t P1_THR_4;					// P1_THR_4 Register (Address = 63h)
		P1_THR_5_t P1_THR_5;					// P1_THR_5 Register (Address = 64h)	
		P1_THR_6_t P1_THR_6;					// P1_THR_6 Register (Address = 65h)
		P1_THR_7_t P1_THR_7;					// P1_THR_7 Register (Address = 66h)
		P1_THR_8_t P1_THR_8;					// P1_THR_8 Register (Address = 67h)
		P1_THR_9_t P1_THR_9;					// P1_THR_9 Register (Address = 68h)
		P1_THR_10_t P1_THR_10;				// P1_THR_10 Register (Address = 69h)
		uint8_t P1_THR_11;						// P1_THR_11 Register (Address = 6Ah) TH_P1_L9
		uint8_t P1_THR_12;						// P1_THR_12 Register (Address = 6Bh) TH_P1_L10
		uint8_t P1_THR_13;						// P1_THR_13 Register (Address = 6Ch) TH_P1_L11
		uint8_t P1_THR_14;						// P1_THR_13 Register (Address = 6Dh) TH_P1_L12
		P1_THR_15_t P1_THR_15;				// P1_THR_15 Register (Address = 6Eh) TH_P1_OFF
		P2_THR_0_t P2_THR_0;					// P2_THR_0 Register (Address = 6Fh)
		P2_THR_1_t P2_THR_1;					// P2_THR_1 Register (Address = 70h)
		P2_THR_2_t P2_THR_2;					// P2_THR_2 Register (Address = 71h)
		P2_THR_3_t P2_THR_3;					// P2_THR_3 Register (Address = 72h)	
		P2_THR_4_t P2_THR_4;					// P2_THR_4 Register (Address = 73h)
		P2_THR_5_t P2_THR_5;					// P2_THR_5 Register (Address = 74h)	
		P2_THR_6_t P2_THR_6;					// P2_THR_6 Register (Address = 75h)
		P2_THR_7_t P2_THR_7;					// P2_THR_7 Register (Address = 76h)
		P2_THR_8_t P2_THR_8;					// P2_THR_8 Register (Address = 77h)
		P2_THR_9_t P2_THR_9;					// P2_THR_9 Register (Address = 78h)
		P2_THR_10_t P2_THR_10;				// P2_THR_10 Register (Address = 79h)
		uint8_t P2_THR_11;						// P2_THR_11 Register (Address = 7Ah) TH_P2_L9
		uint8_t P2_THR_12;						// P2_THR_12 Register (Address = 7Bh) TH_P2_L10
		uint8_t P2_THR_13;						// P2_THR_13 Register (Address = 7Ch) TH_P2_L11
		uint8_t P2_THR_14;						// P1_THR_13 Register (Address = 7Dh) TH_P2_L12	
		P2_THR_15_t P2_THR_15;				// P2_THR_15 Register (Address = 7Eh) TH_P2_OFF
		uint8_t THR_CRC;							// P1_THR_13 Register (Address = 7Fh) Threshold map configuration registers data CRC value
} PGA460_THR_t;


	typedef struct __attribute__((packed)) {
		PGA460_UserData_t USER_DATA;
		PGA460_EEPROMConfig_t EEPROM;
		EE_CNTRL_t EE_CNTRL; 					// EE_CNTRL register (Address 0x40) User EEPROM control register
		PGA460_BPF_t BPF;
		PGA460_LPF_t LPF;
		TEST_MUX_t TEST_MUX;					// TEST_MUX Register (Address = 4Bh)
		DEV_STAT0_t DEV_STAT0;				// DEV_STAT0 Register (Address = 4Ch) 
		DEV_STAT1_t DEV_STAT1;				// DEV_STAT1 Register (Address = 4Dh)
		PGA460_THR_t THR;
	} PGA460_t;

typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t DeviceBusy             : 1; // Bit 0
            uint8_t SyncRateError          : 1; // Bit 1
            uint8_t SyncWidthMismatch      : 1; // Bit 2
            uint8_t ChecksumMismatch       : 1; // Bit 3
            uint8_t InvalidCommand         : 1; // Bit 4
            uint8_t UARTFrameError         : 1; // Bit 5
            uint8_t DiagnosticBitMarker_L  : 1; // Bit 6 - always 1
            uint8_t DiagnosticBitMarker_H  : 1; // Bit 7 - always 0 (in UART_DIAG=0)
        } BitField;
    };
} PGA460_Diag_t;



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
    PGA460_CMD_THRESHOLD_BULK_WRITE           = 0x10,

    // Broadcast Commands
    PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P1  = 0x11,
    PGA460_CMD_BROADCAST_BURST_AND_LISTEN_P2  = 0x12,
    PGA460_CMD_BROADCAST_LISTEN_ONLY_P1       = 0x13,
    PGA460_CMD_BROADCAST_LISTEN_ONLY_P2       = 0x14,
    PGA460_CMD_BROADCAST_TEMP_NOISE_MEASURE   = 0x15,
    PGA460_CMD_BROADCAST_REGISTER_WRITE       = 0x16,
    PGA460_CMD_BROADCAST_EEPROM_BULK_WRITE    = 0x17,
    PGA460_CMD_BROADCAST_TVG_BULK_WRITE       = 0x18,
    PGA460_CMD_BROADCAST_THRESHOLD_BULK_WRITE = 0x19
} PGA460_Command_t;

typedef struct __attribute__((packed)) {
    uint8_t cmdData[3];
} CommandArray;

typedef enum {
    PGA460_MEAS_DISTANCE	= 0,	// Retrieve distance measurement (in meters)
    PGA460_MEAS_WIDTH			= 1,	// Retrieve width of echo signal (in microseconds)
    PGA460_MEAS_AMPLITUDE	= 2		// Retrieve peak amplitude of echo signal (8-bit raw value)
} PGA460_MeasResult_t;

typedef enum {
    PGA460_CMD_GET_TEMP = 0x00,
    PGA460_CMD_GET_NOISE = 0x01
} PGA460_CmdType_t;

typedef enum {
    PGA460_GAIN_32_64dB = 0xCF,  // 32-64dB
    PGA460_GAIN_46_78dB = 0x8F,  // 46-78dB
    PGA460_GAIN_52_84dB = 0x4F,  // 52-84dB
    PGA460_GAIN_58_90dB = 0x0F   // 58-90dB
} PGA460_GainRange_t;

typedef enum {
    PGA460_TVG_25_PERCENT = 0,  // 25% of range
    PGA460_TVG_50_PERCENT = 1,  // 50% of range
    PGA460_TVG_75_PERCENT = 2,  // 75% of range
		PGA460_TVG_CUSTOM
} PGA460_TVG_Level_t;

typedef enum {
    PGA460_TRH_25 = 0,  // 25% of range
    PGA460_TRH_50 = 1,  // 50% of range
    PGA460_TRH_75 = 2,  // 75% of range
		PGA460_TRH_CC = 3,  // Custom
		PGA460_TRH_DE = 4   // Default
} PGA460_TRH_Level_t;

typedef struct __attribute__((packed)) {
    float distance;		// Time-of-flight distance (2 bytes)
    uint8_t width;		// Echo width (1 byte)
    uint8_t amplitude;	// Echo amplitude (1 byte)
} PGA460_MeasData_t;

typedef struct __attribute__((packed)) {
    UART_HandleTypeDef *uartPort;  // Matches the type of huart1, huart4, huart5
    PGA460_t PGA460_Data;
		PGA460_MeasData_t objects[PGA_MAX_OBJECTS];  // Array for detected objects
} PGA460_Sensor_t;


#endif // PGA460_REG_H

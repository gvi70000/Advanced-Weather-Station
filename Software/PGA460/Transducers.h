#ifndef __Transducers_H
#define __Transducers_H

#include "PGA460_REG.h"

#ifdef USE_MA4S4S_R // Murata MA4S4S/R Configuration
const PGA460_t transducer = (PGA460_t){
		.USER_DATA = {
			.USER_DATA1  = 0x00, .USER_DATA2  = 0x00, .USER_DATA3  = 0x00, .USER_DATA4  = 0x00,
			.USER_DATA5  = 0x00, .USER_DATA6  = 0x00, .USER_DATA7  = 0x00, .USER_DATA8  = 0x00,
			.USER_DATA9  = 0x00, .USER_DATA10 = 0x00, .USER_DATA11 = 0x00, .USER_DATA12 = 0x00,
			.USER_DATA13 = 0x00, .USER_DATA14 = 0x00, .USER_DATA15 = 0x00, .USER_DATA16 = 0x00,
			.USER_DATA17 = 0x00, .USER_DATA18 = 0x00, .USER_DATA19 = 0x00, .USER_DATA20 = 0x00,
		},
		.EEPROM = {
			.TGV = {
				.TVGAIN0.Val.Value = 0xAA,
				.TVGAIN1.Val.Value = 0xAA,
				.TVGAIN2.Val.Value = 0xAA,
				.TVGAIN3.Val.Value = 0x51,
				.TVGAIN4.Val.Value = 0x45,
				.TVGAIN5.Val.Value = 0x14,
				.TVGAIN6.Val.Value = 0x50,
			},
			.INIT_GAIN.Val       = 0x54,
			.FREQ                = 0x32,
			.DEADTIME.Val        = 0xA0,
			.PULSE_P1.Val        = 0x82,
			.PULSE_P2.Val        = 0x10,
			.CURR_LIM_P1.Val     = 0x40,
			.CURR_LIM_P2.Val     = 0x40,
			.REC_LENGTH.Val      = 0x19,
			.FREQ_DIAG.Val       = 0x33,
			.SAT_FDIAG_TH.Val    = 0xFE,
			.FVOLT_DEC.Val       = 0x7C,
			.DECPL_TEMP.Val      = 0x4F,
			.DSP_SCALE.Val       = 0x00,
			.TEMP_TRIM.Val       = 0x00,
			.P1_GAIN_CTRL.Val    = 0x00,
			.P2_GAIN_CTRL.Val    = 0x00,
		}
	};
#elif defined(USE_MA40H1SR) // Murata MA40H1SR Configuration
const PGA460_t 	transducer = (PGA460_t){
		.USER_DATA = {
			.USER_DATA1  = 0x00, .USER_DATA2  = 0x00, .USER_DATA3  = 0x00, .USER_DATA4  = 0x00,
			.USER_DATA5  = 0x00, .USER_DATA6  = 0x00, .USER_DATA7  = 0x00, .USER_DATA8  = 0x00,
			.USER_DATA9  = 0x00, .USER_DATA10 = 0x00, .USER_DATA11 = 0x00, .USER_DATA12 = 0x00,
			.USER_DATA13 = 0x00, .USER_DATA14 = 0x00, .USER_DATA15 = 0x00, .USER_DATA16 = 0x00,
			.USER_DATA17 = 0x00, .USER_DATA18 = 0x00, .USER_DATA19 = 0x00, .USER_DATA20 = 0x00,
		},
		.EEPROM = {
			.TGV = {
				.TVGAIN0.Val.Value = 0xAA,
				.TVGAIN1.Val.Value = 0xAA,
				.TVGAIN2.Val.Value = 0xAA,
				.TVGAIN3.Val.Value = 0x51,
				.TVGAIN4.Val.Value = 0x45,
				.TVGAIN5.Val.Value = 0x14,
				.TVGAIN6.Val.Value = 0x50,
			},
			.INIT_GAIN.Val       = 0x54,
			.FREQ                = 0x32,
			.DEADTIME.Val        = 0xA0,
			.PULSE_P1.Val        = 0x88,
			.PULSE_P2.Val        = 0x10,
			.CURR_LIM_P1.Val     = 0x40,
			.CURR_LIM_P2.Val     = 0x40,
			.REC_LENGTH.Val      = 0x19,
			.FREQ_DIAG.Val       = 0x33,
			.SAT_FDIAG_TH.Val    = 0xFE,
			.FVOLT_DEC.Val       = 0x7C,
			.DECPL_TEMP.Val      = 0x4F,
			.DSP_SCALE.Val       = 0x00,
			.TEMP_TRIM.Val       = 0x00,
			.P1_GAIN_CTRL.Val    = 0x09,
			.P2_GAIN_CTRL.Val    = 0x09,
		}
	};
#elif defined(USE_MA58MF14_7N) // Murata MA58MF14-7N Configuration
const PGA460_t transducer = (PGA460_t){
		.USER_DATA = {
			.USER_DATA1  = 0x00, .USER_DATA2  = 0x00, .USER_DATA3  = 0x00, .USER_DATA4  = 0x00,
			.USER_DATA5  = 0x00, .USER_DATA6  = 0x00, .USER_DATA7  = 0x00, .USER_DATA8  = 0x00,
			.USER_DATA9  = 0x00, .USER_DATA10 = 0x00, .USER_DATA11 = 0x00, .USER_DATA12 = 0x00,
			.USER_DATA13 = 0x00, .USER_DATA14 = 0x00, .USER_DATA15 = 0x00, .USER_DATA16 = 0x00,
			.USER_DATA17 = 0x00, .USER_DATA18 = 0x00, .USER_DATA19 = 0x00, .USER_DATA20 = 0x00,
		},
		.EEPROM = {
			.TGV = {
				.TVGAIN0.Val.Value = 0xAA,
				.TVGAIN1.Val.Value = 0xAA,
				.TVGAIN2.Val.Value = 0xAA,
				.TVGAIN3.Val.Value = 0x82,
				.TVGAIN4.Val.Value = 0x08,
				.TVGAIN5.Val.Value = 0x20,
				.TVGAIN6.Val.Value = 0x80,
			},
			.INIT_GAIN.Val       = 0x60,
			.FREQ                = 0x8F,
			.DEADTIME.Val        = 0xA0,
			.PULSE_P1.Val        = 0x84,
			.PULSE_P2.Val        = 0x10,
			.CURR_LIM_P1.Val     = 0x55,
			.CURR_LIM_P2.Val     = 0x55,
			.REC_LENGTH.Val      = 0x19,
			.FREQ_DIAG.Val       = 0x33,
			.SAT_FDIAG_TH.Val    = 0xEE,
			.FVOLT_DEC.Val       = 0x7C,
			.DECPL_TEMP.Val      = 0x4F,
			.DSP_SCALE.Val       = 0x00,
			.TEMP_TRIM.Val       = 0x00,
			.P1_GAIN_CTRL.Val    = 0x09,
			.P2_GAIN_CTRL.Val    = 0x09,
		}
	};
#elif defined(USE_UTR_1440K_TT_R) // PUI Audio UTR-1440K-TT-R configuration
const PGA460_t transducer = (PGA460_t){
    .USER_DATA = {
        .USER_DATA1  = 0x00, .USER_DATA2  = 0x00, .USER_DATA3  = 0x00, .USER_DATA4  = 0x00,
        .USER_DATA5  = 0x00, .USER_DATA6  = 0x00, .USER_DATA7  = 0x00, .USER_DATA8  = 0x00,
        .USER_DATA9  = 0x00, .USER_DATA10 = 0x00, .USER_DATA11 = 0x00, .USER_DATA12 = 0x00,
        .USER_DATA13 = 0x00, .USER_DATA14 = 0x00, .USER_DATA15 = 0x00, .USER_DATA16 = 0x00,
        .USER_DATA17 = 0x00, .USER_DATA18 = 0x00, .USER_DATA19 = 0x00, .USER_DATA20 = 0x00,
    },
    .EEPROM = {
        .TGV = {
            .TVGAIN0.Val.Value = 0x9D,
            .TVGAIN1.Val.Value = 0xEE,
            .TVGAIN2.Val.Value = 0xEF,
            .TVGAIN3.Val.Value = 0x2D,
            .TVGAIN4.Val.Value = 0xB9,
            .TVGAIN5.Val.Value = 0xEF,
            .TVGAIN6.Val.Value = 0xDC,
        },
        .INIT_GAIN.Val       = 0x03,
        .FREQ                = 0x32,
        .DEADTIME.Val        = 0x80,
        .PULSE_P1.Val        = (comm == 2) ? (0x80 | 0x08) : 0x08,
        .PULSE_P2.Val        = 0x12,
        .CURR_LIM_P1.Val     = 0x72,
        .CURR_LIM_P2.Val     = 0x32,
        .REC_LENGTH.Val      = 0x09,
        .FREQ_DIAG.Val       = 0x33,
        .SAT_FDIAG_TH.Val    = 0xEE,
        .FVOLT_DEC.Val       = 0x7C,
        .DECPL_TEMP.Val      = 0x8F,
        .DSP_SCALE.Val       = 0x00,
        .TEMP_TRIM.Val       = 0x00,
        .P1_GAIN_CTRL.Val    = 0x09,
        .P2_GAIN_CTRL.Val    = 0x29,
    }
};
#else // User Custom XDCR Configuration
const PGA460_t transducer = {
    // User data section
    .USER_DATA = {
        .USER_DATA1  = 0x00, .USER_DATA2  = 0x00, .USER_DATA3  = 0x00, .USER_DATA4  = 0x00,
        .USER_DATA5  = 0x00, .USER_DATA6  = 0x00, .USER_DATA7  = 0x00, .USER_DATA8  = 0x00,
        .USER_DATA9  = 0x00, .USER_DATA10 = 0x00, .USER_DATA11 = 0x00, .USER_DATA12 = 0x00,
        .USER_DATA13 = 0x00, .USER_DATA14 = 0x00, .USER_DATA15 = 0x00, .USER_DATA16 = 0x00,
        .USER_DATA17 = 0x00, .USER_DATA18 = 0x00, .USER_DATA19 = 0x00, .USER_DATA20 = 0x00,
    },
    .EEPROM = {
        // Time-Varying Gain (TVG) based on tested setup
        .TGV = {
            .TVGAIN0.Val.Value = 0x9D,
            .TVGAIN1.Val.Value = 0xEE,
            .TVGAIN2.Val.Value = 0xEF,
            .TVGAIN3.Val.Value = 0x2D,
            .TVGAIN4.Val.Value = 0xB9,
            .TVGAIN5.Val.Value = 0xEF,
            .TVGAIN6.Val.Value = 0xDC, // TVG6 includes TVG and FreqShift control
        },
        // Initial Gain — reduced for short distances
        .INIT_GAIN.Val = 0x10,  // ~17 dB
        // Transmit Frequency (0.2 × 50 + 30 = 40 kHz)
        .FREQ = 0x32,
        // Deadtime + Comparator Deglitch
        // THR_CMP_DEGLITCH = 0x01 (8 µs), PULSE_DT = 0x00 (0 µs)
        .DEADTIME.Val = 0x10,
        // Burst Pulses — minimal for quick decay
        .PULSE_P1.Val = 0x02,  // 2 pulses
        .PULSE_P2.Val = 0x04,  // 4 pulses
        // Current Limits (moderate drive strength)
        // 7×0x28 + 50 = 246 mA
        .CURR_LIM_P1.Val.BitField.CURR_LIM1 = 0x08, //98mA
        .CURR_LIM_P2.Val = 0x16,  // 204mA if needed for farther range
        // Record Length — maximize resolution
        // 4096 µs total ? 32 µs per bin
        .REC_LENGTH.Val = 0x00,
        // Diagnostics and scaling
        .FREQ_DIAG.Val     = 0x33,
        .SAT_FDIAG_TH.Val  = 0xFF,
        .FVOLT_DEC.Val     = 0x02,
        .DECPL_TEMP.Val    = 0x00,
        .DSP_SCALE.Val     = 0x00,
        .TEMP_TRIM.Val     = 0x00,
        // Gain control for P1 and P2
        .P1_GAIN_CTRL.Val = 0x05,
        .P2_GAIN_CTRL.Val = 0x05,
    }
};
#endif

const PGA460_THR_t THRESHOLD_25_STRUCT = {
    .P1_THR_0.Val.Value  = 0x88, .P1_THR_1.Val.Value  = 0x88, .P1_THR_2.Val.Value  = 0x88, .P1_THR_3.Val.Value  = 0x88,
    .P1_THR_4.Val.Value  = 0x88, .P1_THR_5.Val.Value  = 0x88, .P1_THR_6.Val.Value  = 0x42, .P1_THR_7.Val.Value  = 0x10,
    .P1_THR_8.Val.Value  = 0x84, .P1_THR_9.Val.Value  = 0x21, .P1_THR_10.Val.Value = 0x08, .P1_THR_11           = 0x40,
    .P1_THR_12           = 0x40, .P1_THR_13           = 0x40, .P1_THR_14           = 0x40, .P1_THR_15.Val.Value = 0x00,

    .P2_THR_0.Val.Value  = 0x88, .P2_THR_1.Val.Value  = 0x88, .P2_THR_2.Val.Value  = 0x88, .P2_THR_3.Val.Value  = 0x88,
    .P2_THR_4.Val.Value  = 0x88, .P2_THR_5.Val.Value  = 0x88, .P2_THR_6.Val.Value  = 0x42, .P2_THR_7.Val.Value  = 0x10,
    .P2_THR_8.Val.Value  = 0x84, .P2_THR_9.Val.Value  = 0x21, .P2_THR_10.Val.Value = 0x08, .P2_THR_11           = 0x40,
    .P2_THR_12           = 0x40, .P2_THR_13           = 0x40, .P2_THR_14           = 0x40, .P2_THR_15.Val.Value = 0x00,
    .THR_CRC             = 0x87
};

const PGA460_THR_t THRESHOLD_50_STRUCT = {
    .P1_THR_0.Val.Value  = 0x88, .P1_THR_1.Val.Value  = 0x88, .P1_THR_2.Val.Value  = 0x88, .P1_THR_3.Val.Value  = 0x88,
    .P1_THR_4.Val.Value  = 0x88, .P1_THR_5.Val.Value  = 0x88, .P1_THR_6.Val.Value  = 0x84, .P1_THR_7.Val.Value  = 0x21,
    .P1_THR_8.Val.Value  = 0x08, .P1_THR_9.Val.Value  = 0x42, .P1_THR_10.Val.Value = 0x10, .P1_THR_11           = 0x80,
    .P1_THR_12           = 0x80, .P1_THR_13           = 0x80, .P1_THR_14           = 0x80, .P1_THR_15.Val.Value = 0x00,

    .P2_THR_0.Val.Value  = 0x88, .P2_THR_1.Val.Value  = 0x88, .P2_THR_2.Val.Value  = 0x88, .P2_THR_3.Val.Value  = 0x88,
    .P2_THR_4.Val.Value  = 0x88, .P2_THR_5.Val.Value  = 0x88, .P2_THR_6.Val.Value  = 0x84, .P2_THR_7.Val.Value  = 0x21,
    .P2_THR_8.Val.Value  = 0x08, .P2_THR_9.Val.Value  = 0x42, .P2_THR_10.Val.Value = 0x10, .P2_THR_11           = 0x80,
    .P2_THR_12           = 0x80, .P2_THR_13           = 0x80, .P2_THR_14           = 0x80, .P2_THR_15.Val.Value = 0x00,
    .THR_CRC             = 0x85
};

const PGA460_THR_t THRESHOLD_75_STRUCT = {
    .P1_THR_0.Val.Value  = 0x88, .P1_THR_1.Val.Value  = 0x88, .P1_THR_2.Val.Value  = 0x88, .P1_THR_3.Val.Value  = 0x88,
    .P1_THR_4.Val.Value  = 0x88, .P1_THR_5.Val.Value  = 0x88, .P1_THR_6.Val.Value  = 0xC6, .P1_THR_7.Val.Value  = 0x31,
    .P1_THR_8.Val.Value  = 0x8C, .P1_THR_9.Val.Value  = 0x63, .P1_THR_10.Val.Value = 0x18, .P1_THR_11           = 0xC0,
    .P1_THR_12           = 0xC0, .P1_THR_13           = 0xC0, .P1_THR_14           = 0xC0, .P1_THR_15.Val.Value = 0x00,

    .P2_THR_0.Val.Value  = 0x88, .P2_THR_1.Val.Value  = 0x88, .P2_THR_2.Val.Value  = 0x88, .P2_THR_3.Val.Value  = 0x88,
    .P2_THR_4.Val.Value  = 0x88, .P2_THR_5.Val.Value  = 0x88, .P2_THR_6.Val.Value  = 0xC6, .P2_THR_7.Val.Value  = 0x31,
    .P2_THR_8.Val.Value  = 0x8C, .P2_THR_9.Val.Value  = 0x63, .P2_THR_10.Val.Value = 0x18, .P2_THR_11           = 0xC0,
    .P2_THR_12           = 0xC0, .P2_THR_13           = 0xC0, .P2_THR_14           = 0xC0, .P2_THR_15.Val.Value = 0x00,
    .THR_CRC             = 0x83
};

PGA460_THR_t THRESHOLD_CC_STRUCT = {
    .P1_THR_0.Val.Value  = 0xF0,  // bin 0
    .P1_THR_1.Val.Value  = 0xF0,  // bin 1
    .P1_THR_2.Val.Value  = 0xC0,  // bin 2
    .P1_THR_3.Val.Value  = 0xA0,  // bin 3
    .P1_THR_4.Val.Value  = 0x80,  // bin 4
    .P1_THR_5.Val.Value  = 0x60,  // bin 5
    .P1_THR_6.Val.Value  = 0x50,  // bin 6 — usable echo edge starts here
    .P1_THR_7.Val.Value  = 0x40,  // bin 7
    .P1_THR_8.Val.Value  = 0x30,  // bin 8
    .P1_THR_9.Val.Value  = 0x28,
    .P1_THR_10.Val.Value = 0x20,
    .P1_THR_11           = 0x18,
    .P1_THR_12           = 0x18,
    .P1_THR_13           = 0x10,
    .P1_THR_14           = 0x08,
    .P1_THR_15.Val.Value = 0x08,

    .P2_THR_0.Val.Value  = 0xFF,
    .P2_THR_1.Val.Value  = 0xFF,
    .P2_THR_2.Val.Value  = 0xF0,
    .P2_THR_3.Val.Value  = 0xD0,
    .P2_THR_4.Val.Value  = 0xA0,
    .P2_THR_5.Val.Value  = 0x70,
    .P2_THR_6.Val.Value  = 0x50,
    .P2_THR_7.Val.Value  = 0x30,
    .P2_THR_8.Val.Value  = 0x20,
    .P2_THR_9.Val.Value  = 0x18,
    .P2_THR_10.Val.Value = 0x10,
    .P2_THR_11           = 0x10,
    .P2_THR_12           = 0x10,
    .P2_THR_13           = 0x08,
    .P2_THR_14           = 0x08,
    .P2_THR_15.Val.Value = 0x00,

    .THR_CRC = 0x00  // to be recalculated
};

const PGA460_TVG_t TGV_25_STRUCT = {
    .TVGAIN0.Val.Value = 0x88,
    .TVGAIN1.Val.Value = 0x88,
    .TVGAIN2.Val.Value = 0x88,
    .TVGAIN3.Val.Value = 0x41,
    .TVGAIN4.Val.Value = 0x04,
    .TVGAIN5.Val.Value = 0x10,
    .TVGAIN6.Val.Value = 0x40
};

const PGA460_TVG_t TGV_50_STRUCT = {
    .TVGAIN0.Val.Value = 0x88,
    .TVGAIN1.Val.Value = 0x88,
    .TVGAIN2.Val.Value = 0x88,
    .TVGAIN3.Val.Value = 0x82,
    .TVGAIN4.Val.Value = 0x08,
    .TVGAIN5.Val.Value = 0x20,
    .TVGAIN6.Val.Value = 0x80
};

const PGA460_TVG_t TGV_75_STRUCT = {
    .TVGAIN0.Val.Value = 0x88,
    .TVGAIN1.Val.Value = 0x88,
    .TVGAIN2.Val.Value = 0x88,
    .TVGAIN3.Val.Value = 0xC3,
    .TVGAIN4.Val.Value = 0x0C,
    .TVGAIN5.Val.Value = 0x30,
    .TVGAIN6.Val.Value = 0xC0
};

PGA460_TVG_t TGV_CUSTOM_STRUCT = {
    .TVGAIN0.Val.Value = 0x12,  // Start: 200 µs, Duration: 200 µs
    .TVGAIN1.Val.Value = 0x22,  // Start: 400 µs, Duration: 200 µs
    .TVGAIN2.Val.Value = 0x43,  // Start: 600 µs, Duration: 400 µs
    .TVGAIN3.Val.Value = 0x03,  // G1=0, G2=3 ? 24.5–26 dB
    .TVGAIN4.Val.Value = 0x73,  // G2=7, G3=3 ? 28–26 dB
    .TVGAIN5.Val.Value = 0x77,  // G3=7, G4=7 ? flat at 28 dB
    .TVGAIN6.Val.Value = 0x60   // G5=6 ? 27.5 dB, fixed gain, no freq shift
};
#endif /* __Transducers_H */

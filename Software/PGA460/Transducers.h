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
        .INIT_GAIN.Val = 0x18,  // ~17 dB
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
        .CURR_LIM_P1.Val = 0x28,
        .CURR_LIM_P2.Val = 0x32,  // ~400 mA if needed for farther range
        // Record Length — maximize resolution
        // 4096 µs total ? 32 µs per bin
        .REC_LENGTH.Val = 0x00,
        // Diagnostics and scaling
        .FREQ_DIAG.Val     = 0x33,
        .SAT_FDIAG_TH.Val  = 0xEE,
        .FVOLT_DEC.Val     = 0x02,
        .DECPL_TEMP.Val    = 0x8F,
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
    // P1
    .P1_THR_0.Val.Value  = 0x00, .P1_THR_1.Val.Value  = 0x00, .P1_THR_2.Val.Value  = 0x00, .P1_THR_3.Val.Value  = 0x00,
    .P1_THR_4.Val.Value  = 0x00, .P1_THR_5.Val.Value  = 0x00, .P1_THR_6.Val.Value  = 0x21, .P1_THR_7.Val.Value  = 0x04,
    .P1_THR_8.Val.Value  = 0x22, .P1_THR_9.Val.Value  = 0x10, .P1_THR_10.Val.Value = 0x42, .P1_THR_11           = 0x04,
    .P1_THR_12           = 0x04, .P1_THR_13           = 0x02, .P1_THR_14           = 0x02, .P1_THR_15.Val.Value = 0x07,

    // P2
    .P2_THR_0.Val.Value  = 0x00, .P2_THR_1.Val.Value  = 0x00, .P2_THR_2.Val.Value  = 0x00, .P2_THR_3.Val.Value  = 0x00,
    .P2_THR_4.Val.Value  = 0x00, .P2_THR_5.Val.Value  = 0x00, .P2_THR_6.Val.Value  = 0x20, .P2_THR_7.Val.Value  = 0x84,
    .P2_THR_8.Val.Value  = 0x22, .P2_THR_9.Val.Value  = 0x08, .P2_THR_10.Val.Value = 0x42, .P2_THR_11           = 0x04,
    .P2_THR_12           = 0x02, .P2_THR_13           = 0x02, .P2_THR_14           = 0x02, .P2_THR_15.Val.Value = 0x07,

    .THR_CRC = 0xBE // for Sensor 1; recalc with your CRC function if needed
};

const PGA460_TGV_t TGV_25_STRUCT = {
    .TVGAIN0.Val.Value = 0x88,
    .TVGAIN1.Val.Value = 0x88,
    .TVGAIN2.Val.Value = 0x88,
    .TVGAIN3.Val.Value = 0x41,
    .TVGAIN4.Val.Value = 0x04,
    .TVGAIN5.Val.Value = 0x10,
    .TVGAIN6.Val.Value = 0x40
};

const PGA460_TGV_t TGV_50_STRUCT = {
    .TVGAIN0.Val.Value = 0x88,
    .TVGAIN1.Val.Value = 0x88,
    .TVGAIN2.Val.Value = 0x88,
    .TVGAIN3.Val.Value = 0x82,
    .TVGAIN4.Val.Value = 0x08,
    .TVGAIN5.Val.Value = 0x20,
    .TVGAIN6.Val.Value = 0x80
};

const PGA460_TGV_t TGV_75_STRUCT = {
    .TVGAIN0.Val.Value = 0x88,
    .TVGAIN1.Val.Value = 0x88,
    .TVGAIN2.Val.Value = 0x88,
    .TVGAIN3.Val.Value = 0xC3,
    .TVGAIN4.Val.Value = 0x0C,
    .TVGAIN5.Val.Value = 0x30,
    .TVGAIN6.Val.Value = 0xC0
};

PGA460_TGV_t TGV_CUSTOM_STRUCT = {
    .TVGAIN0.Val.Value = 0x78,  // Early: avoid saturation
    .TVGAIN1.Val.Value = 0x88,  // Slight ramp up
    .TVGAIN2.Val.Value = 0xA0,
    .TVGAIN3.Val.Value = 0xC8,  // Mid-echo strong gain
    .TVGAIN4.Val.Value = 0xE0,
    .TVGAIN5.Val.Value = 0xF4,  // Tail amplification
    .TVGAIN6.Val.Value = 0xF8   // Max gain for final echoes
};
#endif /* __Transducers_H */

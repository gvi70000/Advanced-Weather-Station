#ifndef __Transducers_H
#define __Transducers_H

#include "PGA460_REG.h"

#ifdef USE_MA4S4S_R // Murata MA4S4S/R Configuration
const PGA460_t transducer = {
    // User data registers (optional user-defined settings)
    .USER_DATA1 = 0x00,
    .USER_DATA2 = 0x00,
    .USER_DATA3 = 0x00,
    .USER_DATA4 = 0x00,
    .USER_DATA5 = 0x00,
    .USER_DATA6 = 0x00,
    .USER_DATA7 = 0x00,
    .USER_DATA8 = 0x00,
    .USER_DATA9 = 0x00,
    .USER_DATA10 = 0x00,
    .USER_DATA11 = 0x00,
    .USER_DATA12 = 0x00,
    .USER_DATA13 = 0x00,
    .USER_DATA14 = 0x00,
    .USER_DATA15 = 0x00,
    .USER_DATA16 = 0x00,
    .USER_DATA17 = 0x00,
    .USER_DATA18 = 0x00,
    .USER_DATA19 = 0x00,
    .USER_DATA20 = 0x00,

    // Time-varying gain settings
    .TVGAIN0.Val = 0xAA, // Start gain timing for TVG
    .TVGAIN1.Val = 0xAA, // Timing point 1
    .TVGAIN2.Val = 0xAA, // Timing point 2
    .TVGAIN3.Val = 0x51, // Timing point 3
    .TVGAIN4.Val = 0x45, // Timing point 4
    .TVGAIN5.Val = 0x14, // Timing point 5
    .TVGAIN6.Val = 0x50, // Timing point 6 and frequency shift control

    // Gain control
    .INIT_GAIN.Val = 0x54, // GAIN = 0.5 × (GAIN_INIT + 1) + value(AFE_GAIN_RNG) [dB]

    // Frequency settings
    .FREQ = 0x32, // Frequency = 0.2 × FREQ + 30 [kHz]

    // Dead time and comparator deglitch period
    .DEADTIME.Val = 0xA0, // Dead time and deglitch settings

    .PULSE_P1.Val = 0x02, // 2 pulses for OUTA burst (Preset 1)
    .PULSE_P2.Val = 0x10, // 16 pulses for OUTA burst (Preset 2)

    // Current limit settings
    .CURR_LIM_P1.Val = 0x40, // 330mA (Preset 1) [Current limit = 7×CURR_LIM + 50 mA]
    .CURR_LIM_P2.Val = 0x40, // 330mA (Preset 2)

    // Recording length
    .REC_LENGTH.Val = 0x19, // Recording time length = 4096 × (P_REC + 1) [µs]

    // Diagnostic frequency
    .FREQ_DIAG.Val = 0x33, // Frequency diagnostic settings

    // Saturation diagnostic threshold
    .SAT_FDIAG_TH.Val = 0xFE, // Threshold level

    // Voltage diagnostic and scaling
    .FVOLT_DEC.Val = 0x7C, // VPWR_OV_TH = 12.3V, other diagnostic scaling settings

    // Decoupling time and AFE gain range
    .DECPL_TEMP.Val = 0x4F, // Decoupling time, temperature, and AFE gain range

    // DSP scaling
    .DSP_SCALE.Val = 0x00, // Non-linear scaling noise level

    // Temperature trim
    .TEMP_TRIM.Val = 0x00, // Temperature scale offset and gain

    // Gain control for Preset 1 and 2
    .P1_GAIN_CTRL.Val = 0x00, // Gain control settings for Preset 1
    .P2_GAIN_CTRL.Val = 0x00  // Gain control settings for Preset 2
};
#elif defined(USE_MA40H1SR) // Murata MA40H1SR Configuration
const PGA460_t transducer = {
    .USER_DATA1 = 0x00,
    .USER_DATA2 = 0x00,
    .USER_DATA3 = 0x00,
    .USER_DATA4 = 0x00,
    .USER_DATA5 = 0x00,
    .USER_DATA6 = 0x00,
    .USER_DATA7 = 0x00,
    .USER_DATA8 = 0x00,
    .USER_DATA9 = 0x00,
    .USER_DATA10 = 0x00,
    .USER_DATA11 = 0x00,
    .USER_DATA12 = 0x00,
    .USER_DATA13 = 0x00,
    .USER_DATA14 = 0x00,
    .USER_DATA15 = 0x00,
    .USER_DATA16 = 0x00,
    .USER_DATA17 = 0x00,
    .USER_DATA18 = 0x00,
    .USER_DATA19 = 0x00,
    .USER_DATA20 = 0x00,

    // Time-varying gain settings
    .TVGAIN0.Val = 0xAA, // Start gain timing for TVG
    .TVGAIN1.Val = 0xAA, // Timing point 1
    .TVGAIN2.Val = 0xAA, // Timing point 2
    .TVGAIN3.Val = 0x51, // Timing point 3
    .TVGAIN4.Val = 0x45, // Timing point 4
    .TVGAIN5.Val = 0x14, // Timing point 5
    .TVGAIN6.Val = 0x50, // Timing point 6 and frequency shift control

    // Gain control
    .INIT_GAIN.Val = 0x54, // GAIN = 0.5 × (GAIN_INIT + 1) + value(AFE_GAIN_RNG) [dB]

    // Frequency settings
    .FREQ = 0x32, // Frequency = 0.2 × FREQ + 30 [kHz]

    // Dead time and comparator deglitch period
    .DEADTIME.Val = 0xA0, // Dead time and deglitch settings

    .PULSE_P1.Val = 0x08, // 8 pulses for OUTA burst (Preset 1)
    .PULSE_P2.Val = 0x10, // 16 pulses for OUTA burst (Preset 2)

    // Current limit settings
    .CURR_LIM_P1.Val = 0x40, // 330mA (Preset 1) [Current limit = 7×CURR_LIM + 50 mA]
    .CURR_LIM_P2.Val = 0x40, // 330mA (Preset 2)

    // Recording length
    .REC_LENGTH.Val = 0x19, // Recording time length = 4096 × (P_REC + 1) [µs]

    // Diagnostic frequency
    .FREQ_DIAG.Val = 0x33, // Frequency diagnostic settings

    // Saturation diagnostic threshold
    .SAT_FDIAG_TH.Val = 0xFE, // Threshold level

    // Voltage diagnostic and scaling
    .FVOLT_DEC.Val = 0x7C, // VPWR_OV_TH = 12.3V, other diagnostic scaling settings

    // Decoupling time and AFE gain range
    .DECPL_TEMP.Val = 0x4F, // Decoupling time, temperature, and AFE gain range

    // DSP scaling
    .DSP_SCALE.Val = 0x00, // Non-linear scaling noise level

    // Temperature trim
    .TEMP_TRIM.Val = 0x00, // Temperature scale offset and gain

    // Gain control for Preset 1 and 2
    .P1_GAIN_CTRL.Val = 0x09, // Gain control settings for Preset 1
    .P2_GAIN_CTRL.Val = 0x09  // Gain control settings for Preset 2
};
#elif defined(USE_MA58MF14_7N) // Murata MA58MF14-7N Configuration
const PGA460_t transducer = {
    .USER_DATA1 = 0x00,
    .USER_DATA2 = 0x00,
    .USER_DATA3 = 0x00,
    .USER_DATA4 = 0x00,
    .USER_DATA5 = 0x00,
    .USER_DATA6 = 0x00,
    .USER_DATA7 = 0x00,
    .USER_DATA8 = 0x00,
    .USER_DATA9 = 0x00,
    .USER_DATA10 = 0x00,
    .USER_DATA11 = 0x00,
    .USER_DATA12 = 0x00,
    .USER_DATA13 = 0x00,
    .USER_DATA14 = 0x00,
    .USER_DATA15 = 0x00,
    .USER_DATA16 = 0x00,
    .USER_DATA17 = 0x00,
    .USER_DATA18 = 0x00,
    .USER_DATA19 = 0x00,
    .USER_DATA20 = 0x00,

    // Time-varying gain settings
    .TVGAIN0.Val = 0xAA, // Start gain timing for TVG
    .TVGAIN1.Val = 0xAA, // Timing point 1
    .TVGAIN2.Val = 0xAA, // Timing point 2
    .TVGAIN3.Val = 0x82, // Timing point 3
    .TVGAIN4.Val = 0x08, // Timing point 4
    .TVGAIN5.Val = 0x20, // Timing point 5
    .TVGAIN6.Val = 0x80, // Timing point 6 and frequency shift control

    // Gain control
    .INIT_GAIN.Val = 0x60, // GAIN = 0.5 × (GAIN_INIT + 1) + value(AFE_GAIN_RNG) [dB]

    // Frequency settings
    .FREQ = 0x8F, // Frequency = 0.2 × FREQ + 30 [kHz]

    // Dead time and comparator deglitch period
    .DEADTIME.Val = 0xA0, // Dead time and deglitch settings

    .PULSE_P1.Val = 0x04, // 4 pulses for OUTA burst (Preset 1)
    .PULSE_P2.Val = 0x10, // 16 pulses for OUTA burst (Preset 2)

    // Current limit settings
    .CURR_LIM_P1.Val = 0x55, // 435mA (Preset 1) [Current limit = 7×CURR_LIM + 50 mA]
    .CURR_LIM_P2.Val = 0x55, // 435mA (Preset 2)

    // Recording length
    .REC_LENGTH.Val = 0x19, // Recording time length = 4096 × (P_REC + 1) [µs]

    // Diagnostic frequency
    .FREQ_DIAG.Val = 0x33, // Frequency diagnostic settings

    // Saturation diagnostic threshold
    .SAT_FDIAG_TH.Val = 0xEE, // Threshold level

    // Voltage diagnostic and scaling
    .FVOLT_DEC.Val = 0x7C, // VPWR_OV_TH = 12.3V, other diagnostic scaling settings

    // Decoupling time and AFE gain range
    .DECPL_TEMP.Val = 0x4F, // Decoupling time, temperature, and AFE gain range

    // DSP scaling
    .DSP_SCALE.Val = 0x00, // Non-linear scaling noise level

    // Temperature trim
    .TEMP_TRIM.Val = 0x00, // Temperature scale offset and gain

    // Gain control for Preset 1 and 2
    .P1_GAIN_CTRL.Val = 0x09, // Gain control settings for Preset 1
    .P2_GAIN_CTRL.Val = 0x09  // Gain control settings for Preset 2
};
#elif defined(USE_UTR_1440K_TT_R) // PUI Audio UTR-1440K-TT-R configuration
const PGA460_t transducer = {
    .USER_DATA1 = 0x00, // User data 1
    .USER_DATA2 = 0x00, // User data 2
    .USER_DATA3 = 0x00, // User data 3
    .USER_DATA4 = 0x00, // User data 4
    .USER_DATA5 = 0x00, // User data 5
    .USER_DATA6 = 0x00, // User data 6
    .USER_DATA7 = 0x00, // User data 7
    .USER_DATA8 = 0x00, // User data 8
    .USER_DATA9 = 0x00, // User data 9
    .USER_DATA10 = 0x00, // User data 10
    .USER_DATA11 = 0x00, // User data 11
    .USER_DATA12 = 0x00, // User data 12
    .USER_DATA13 = 0x00, // User data 13
    .USER_DATA14 = 0x00, // User data 14
    .USER_DATA15 = 0x00, // User data 15
    .USER_DATA16 = 0x00, // User data 16
    .USER_DATA17 = 0x00, // User data 17
    .USER_DATA18 = 0x00, // User data 18
    .USER_DATA19 = 0x00, // User data 19
    .USER_DATA20 = 0x00, // User data 20

    // Time-varying gain settings
    .TVGAIN0.Val = 0x9D, // Start gain timing for TVG
    .TVGAIN1.Val = 0xEE, // Timing point 1
    .TVGAIN2.Val = 0xEF, // Timing point 2
    .TVGAIN3.Val = 0x2D, // Timing point 3
    .TVGAIN4.Val = 0xB9, // Timing point 4
    .TVGAIN5.Val = 0xEF, // Timing point 5
    .TVGAIN6.Val = 0xDC, // Timing point 6 and frequency shift control

    // Gain control
    .INIT_GAIN.Val = 0x20, //0x03, // GAIN = 0.5 × (GAIN_INIT+1) + value(AFE_GAIN_RNG) [dB]

    // Frequency settings
    .FREQ = 0x32, // Frequency = 0.2 × FREQ + 30 [kHz]

    // Dead time and comparator deglitch period
    .DEADTIME.Val = 0x80, // Dead time and deglitch settings

		.PULSE_P1.Val = 0x08, // 8 pulses for OUTA burst (Preset 1)
    .PULSE_P2.Val = 0x12, // 10 pulses for OUTA burst (Preset 2)

    // Current limit settings
    .CURR_LIM_P1.Val = 0x32, // 400mA (Preset 1) [Current limit = 7×CURR_LIM+50 mA]
    .CURR_LIM_P2.Val = 0x32, // 400mA (Preset 2)

    // Recording length
    .REC_LENGTH.Val = 0x00, // Recording time length = 4096 × (P_REC+1) [µs]

    // Diagnostic frequency
    .FREQ_DIAG.Val = 0x33, // Frequency diagnostic settings

    // Saturation diagnostic threshold
    .SAT_FDIAG_TH.Val = 0xEE, // Threshold level

    // Voltage diagnostic and scaling
    .FVOLT_DEC.Val = 0x02, //0x3C, // VPWR_OV_TH = 17.7V, other diagnostic scaling settings

    // Decoupling time and AFE gain range
    .DECPL_TEMP.Val = 0x8F, // Decoupling time, temperature, and AFE gain range

    // DSP scaling
    .DSP_SCALE.Val = 0x00, // Non-linear scaling noise level

    // Temperature trim
    .TEMP_TRIM.Val = 0x00, // Temperature scale offset and gain

    // Gain control for Preset 1 and 2
    .P1_GAIN_CTRL.Val = 0x05, // Gain control settings for Preset 1
    .P2_GAIN_CTRL.Val = 0x05, // Gain control settings for Preset 2
};
#else // User Custom XDCR Configuration
const PGA460_t transducer = {
    .USER_DATA1 = 0x00,
    .USER_DATA2 = 0x00,
    .USER_DATA3 = 0x00,
    .USER_DATA4 = 0x00,
    .USER_DATA5 = 0x00,
    .USER_DATA6 = 0x00,
    .USER_DATA7 = 0x00,
    .USER_DATA8 = 0x00,
    .USER_DATA9 = 0x00,
    .USER_DATA10 = 0x00,
    .USER_DATA11 = 0x00,
    .USER_DATA12 = 0x00,
    .USER_DATA13 = 0x00,
    .USER_DATA14 = 0x00,
    .USER_DATA15 = 0x00,
    .USER_DATA16 = 0x00,
    .USER_DATA17 = 0x00,
    .USER_DATA18 = 0x00,
    .USER_DATA19 = 0x00,
    .USER_DATA20 = 0x00,

    // Time-varying gain settings
    .TVGAIN0.Val = 0xAA, // Start gain timing for TVG
    .TVGAIN1.Val = 0xAA, // Timing point 1
    .TVGAIN2.Val = 0xAA, // Timing point 2
    .TVGAIN3.Val = 0x82, // Timing point 3
    .TVGAIN4.Val = 0x08, // Timing point 4
    .TVGAIN5.Val = 0x20, // Timing point 5
    .TVGAIN6.Val = 0x80, // Timing point 6 and frequency shift control

    // Gain control
    .INIT_GAIN.Val = 0x60, // GAIN = 0.5 × (GAIN_INIT + 1) + value(AFE_GAIN_RNG) [dB]

    // Frequency settings
    .FREQ = 0x8F, // Frequency = 0.2 × FREQ + 30 [kHz]

    // Dead time and comparator deglitch period
    .DEADTIME.Val = 0xA0, // Dead time and deglitch settings

    .PULSE_P1.Val = 0x04, // 4 pulses for OUTA burst (Preset 1)
    .PULSE_P2.Val = 0x10, // 16 pulses for OUTA burst (Preset 2)

    // Current limit settings
    .CURR_LIM_P1.Val = 0x55, // 440mA (Preset 1) [Current limit = 7×CURR_LIM + 50 mA]
    .CURR_LIM_P2.Val = 0x55, // 440mA (Preset 2)

    // Recording length
    .REC_LENGTH.Val = 0x19, // Recording time length = 4096 × (P_REC + 1) [µs]

    // Diagnostic frequency
    .FREQ_DIAG.Val = 0x33, // Frequency diagnostic settings

    // Saturation diagnostic threshold
    .SAT_FDIAG_TH.Val = 0xFE, // Threshold level

    // Voltage diagnostic and scaling
    .FVOLT_DEC.Val = 0x7C, // VPWR_OV_TH = 12.3V, other diagnostic scaling settings

    // Decoupling time and AFE gain range
    .DECPL_TEMP.Val = 0x4F, // Decoupling time, temperature, and AFE gain range

    // DSP scaling
    .DSP_SCALE.Val = 0x00, // Non-linear scaling noise level

    // Temperature trim
    .TEMP_TRIM.Val = 0x00, // Temperature scale offset and gain

    // Gain control for Preset 1 and 2
    .P1_GAIN_CTRL.Val = 0x09, // Gain control settings for Preset 1
    .P2_GAIN_CTRL.Val = 0x09  // Gain control settings for Preset 2
};
#endif

const uint8_t THRESHOLD_25[32] = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x42, 0x10, 0x84, 0x21, 0x08, 0x40, 0x40, 0x40, 0x40, 0x00,
																	0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x42, 0x10, 0x84, 0x21, 0x08, 0x40, 0x40, 0x40, 0x40, 0x00};

const uint8_t THRESHOLD_50[32] = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x84, 0x21, 0x08, 0x42, 0x10, 0x80, 0x80, 0x80, 0x80, 0x00,
																	0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x84, 0x21, 0x08, 0x42, 0x10, 0x80, 0x80, 0x80, 0x80, 0x00};

const uint8_t THRESHOLD_75[32] = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0xC6, 0x31, 0x8C, 0x63, 0x18, 0xC0, 0xC0, 0xC0, 0xC0, 0x00,
																	0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0xC6, 0x31, 0x8C, 0x63, 0x18, 0xC0, 0xC0, 0xC0, 0xC0, 0x00};

const uint8_t THRESHOLD_CUSTOM[32] = {0x41, 0x44, 0x10, 0x06, 0x69, 0x99, 0xDD, 0x4C, 0x31, 0x08, 0x42, 0x18, 0x20, 0x24, 0x2A, 0x00,
																			0x41, 0x44, 0x10, 0x06, 0x09, 0x99, 0xDD, 0x4C, 0x31, 0x08, 0x42, 0x24, 0x30, 0x36, 0x3C, 0x00};

const uint8_t TGV_25[7] = {0x88, 0x88, 0x88, 0x41, 0x04, 0x10, 0x40};
const uint8_t TGV_50[7] = {0x88, 0x88, 0x88, 0x82, 0x08, 0x20, 0x80};
const uint8_t TGV_75[7] = {0x88, 0x88, 0x88, 0xC3, 0x0C, 0x30, 0xC0};

#endif /* __Transducers_H */

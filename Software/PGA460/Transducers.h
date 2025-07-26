#ifndef __Transducers_H
#define __Transducers_H

#include "PGA460_REG.h"

#ifdef USE_MA4S4S_R // Murata MA4S4S/R Configuration
const PGA460_t transducer = {
    // User data registers (optional user-defined settings)
    .USER_DATA.USER_DATA1 = 0x00,
    .USER_DATA.USER_DATA2 = 0x00,
    .USER_DATA.USER_DATA3 = 0x00,
    .USER_DATA.USER_DATA4 = 0x00,
    .USER_DATA.USER_DATA5 = 0x00,
    .USER_DATA.USER_DATA6 = 0x00,
    .USER_DATA.USER_DATA7 = 0x00,
    .USER_DATA.USER_DATA8 = 0x00,
    .USER_DATA.USER_DATA9 = 0x00,
    .USER_DATA.USER_DATA10 = 0x00,
    .USER_DATA.USER_DATA11 = 0x00,
    .USER_DATA.USER_DATA12 = 0x00,
    .USER_DATA.USER_DATA13 = 0x00,
    .USER_DATA.USER_DATA14 = 0x00,
    .USER_DATA.USER_DATA15 = 0x00,
    .USER_DATA.USER_DATA16 = 0x00,
    .USER_DATA.USER_DATA17 = 0x00,
    .USER_DATA.USER_DATA18 = 0x00,
    .USER_DATA.USER_DATA19 = 0x00,
    .USER_DATA.USER_DATA20 = 0x00,

    // Time-varying gain settings
    .EEPROM.TGV.TVGAIN0.Val.Value = 0xAA, // Start gain timing for TVG
    .EEPROM.TGV.TVGAIN1.Val.Value = 0xAA, // Timing point 1
    .EEPROM.TGV.TVGAIN2.Val.Value = 0xAA, // Timing point 2
    .EEPROM.TGV.TVGAIN3.Val.Value = 0x51, // Timing point 3
    .EEPROM.TGV.TVGAIN4.Val.Value = 0x45, // Timing point 4
    .EEPROM.TGV.TVGAIN5.Val.Value = 0x14, // Timing point 5
    .EEPROM.TGV.TVGAIN6.Val.Value = 0x50, // Timing point 6 and frequency shift control

    // Gain control
    .EEPROM.INIT_GAIN.Val = 0x54, // GAIN = 0.5 × (GAIN_INIT + 1) + value(AFE_GAIN_RNG) [dB]

    // Frequency settings
    .EEPROM.FREQ = 0x32, // Frequency = 0.2 × FREQ + 30 [kHz]

    // Dead time and comparator deglitch period
    .EEPROM.DEADTIME.Val = 0xA0, // Dead time and deglitch settings

    .EEPROM.PULSE_P1.Val = 0x02, // 2 pulses for OUTA burst (Preset 1)
    .EEPROM.PULSE_P2.Val = 0x10, // 16 pulses for OUTA burst (Preset 2)

    // Current limit settings
    .EEPROM.CURR_LIM_P1.Val = 0x40, // 330mA (Preset 1) [Current limit = 7×CURR_LIM + 50 mA]
    .EEPROM.CURR_LIM_P2.Val = 0x40, // 330mA (Preset 2)

    // Recording length
    .EEPROM.REC_LENGTH.Val = 0x19, // Recording time length = 4096 × (P_REC + 1) [µs]

    // Diagnostic frequency
    .EEPROM.FREQ_DIAG.Val = 0x33, // Frequency diagnostic settings

    // Saturation diagnostic threshold
    .EEPROM.SAT_FDIAG_TH.Val = 0xFE, // Threshold level

    // Voltage diagnostic and scaling
    .EEPROM.FVOLT_DEC.Val = 0x7C, // VPWR_OV_TH = 12.3V, other diagnostic scaling settings

    // Decoupling time and AFE gain range
    .EEPROM.DECPL_TEMP.Val = 0x4F, // Decoupling time, temperature, and AFE gain range

    // DSP scaling
    .EEPROM.DSP_SCALE.Val = 0x00, // Non-linear scaling noise level

    // Temperature trim
    .EEPROM.TEMP_TRIM.Val = 0x00, // Temperature scale offset and gain

    // Gain control for Preset 1 and 2
    .EEPROM.P1_GAIN_CTRL.Val = 0x00, // Gain control settings for Preset 1
    .EEPROM.P2_GAIN_CTRL.Val = 0x00  // Gain control settings for Preset 2
};
#elif defined(USE_MA40H1SR) // Murata MA40H1SR Configuration
const PGA460_t transducer = {
    .USER_DATA.USER_DATA1 = 0x00,
    .USER_DATA.USER_DATA2 = 0x00,
    .USER_DATA.USER_DATA3 = 0x00,
    .USER_DATA.USER_DATA4 = 0x00,
    .USER_DATA.USER_DATA5 = 0x00,
    .USER_DATA.USER_DATA6 = 0x00,
    .USER_DATA.USER_DATA7 = 0x00,
    .USER_DATA.USER_DATA8 = 0x00,
    .USER_DATA.USER_DATA9 = 0x00,
    .USER_DATA.USER_DATA10 = 0x00,
    .USER_DATA.USER_DATA11 = 0x00,
    .USER_DATA.USER_DATA12 = 0x00,
    .USER_DATA.USER_DATA13 = 0x00,
    .USER_DATA.USER_DATA14 = 0x00,
    .USER_DATA.USER_DATA15 = 0x00,
    .USER_DATA.USER_DATA16 = 0x00,
    .USER_DATA.USER_DATA17 = 0x00,
    .USER_DATA.USER_DATA18 = 0x00,
    .USER_DATA.USER_DATA19 = 0x00,
    .USER_DATA.USER_DATA20 = 0x00,

    // Time-varying gain settings
    .EEPROM.TGV.TVGAIN0.Val.Value = 0xAA, // Start gain timing for TVG
    .EEPROM.TGV.TVGAIN1.Val.Value = 0xAA, // Timing point 1
    .EEPROM.TGV.TVGAIN2.Val.Value = 0xAA, // Timing point 2
    .EEPROM.TGV.TVGAIN3.Val.Value = 0x51, // Timing point 3
    .EEPROM.TGV.TVGAIN4.Val.Value = 0x45, // Timing point 4
    .EEPROM.TGV.TVGAIN5.Val.Value = 0x14, // Timing point 5
    .EEPROM.TGV.TVGAIN6.Val.Value = 0x50, // Timing point 6 and frequency shift control

    // Gain control
    .EEPROM.INIT_GAIN.Val = 0x54, // GAIN = 0.5 × (GAIN_INIT + 1) + value(AFE_GAIN_RNG) [dB]

    // Frequency settings
    .EEPROM.FREQ = 0x32, // Frequency = 0.2 × FREQ + 30 [kHz]

    // Dead time and comparator deglitch period
    .EEPROM.DEADTIME.Val = 0xA0, // Dead time and deglitch settings

    .EEPROM.PULSE_P1.Val = 0x08, // 8 pulses for OUTA burst (Preset 1)
    .EEPROM.PULSE_P2.Val = 0x10, // 16 pulses for OUTA burst (Preset 2)

    // Current limit settings
    .EEPROM.CURR_LIM_P1.Val = 0x40, // 330mA (Preset 1) [Current limit = 7×CURR_LIM + 50 mA]
    .EEPROM.CURR_LIM_P2.Val = 0x40, // 330mA (Preset 2)

    // Recording length
    .EEPROM.REC_LENGTH.Val = 0x19, // Recording time length = 4096 × (P_REC + 1) [µs]

    // Diagnostic frequency
    .EEPROM.FREQ_DIAG.Val = 0x33, // Frequency diagnostic settings

    // Saturation diagnostic threshold
    .EEPROM.SAT_FDIAG_TH.Val = 0xFE, // Threshold level

    // Voltage diagnostic and scaling
    .EEPROM.FVOLT_DEC.Val = 0x7C, // VPWR_OV_TH = 12.3V, other diagnostic scaling settings

    // Decoupling time and AFE gain range
    .EEPROM.DECPL_TEMP.Val = 0x4F, // Decoupling time, temperature, and AFE gain range

    // DSP scaling
    .EEPROM.DSP_SCALE.Val = 0x00, // Non-linear scaling noise level

    // Temperature trim
    .EEPROM.TEMP_TRIM.Val = 0x00, // Temperature scale offset and gain

    // Gain control for Preset 1 and 2
    .EEPROM.P1_GAIN_CTRL.Val = 0x09, // Gain control settings for Preset 1
    .EEPROM.P2_GAIN_CTRL.Val = 0x09  // Gain control settings for Preset 2
};
#elif defined(USE_MA58MF14_7N) // Murata MA58MF14-7N Configuration
const PGA460_t transducer = {
    .USER_DATA.USER_DATA1 = 0x00,
    .USER_DATA.USER_DATA2 = 0x00,
    .USER_DATA.USER_DATA3 = 0x00,
    .USER_DATA.USER_DATA4 = 0x00,
    .USER_DATA.USER_DATA5 = 0x00,
    .USER_DATA.USER_DATA6 = 0x00,
    .USER_DATA.USER_DATA7 = 0x00,
    .USER_DATA.USER_DATA8 = 0x00,
    .USER_DATA.USER_DATA9 = 0x00,
    .USER_DATA.USER_DATA10 = 0x00,
    .USER_DATA.USER_DATA11 = 0x00,
    .USER_DATA.USER_DATA12 = 0x00,
    .USER_DATA.USER_DATA13 = 0x00,
    .USER_DATA.USER_DATA14 = 0x00,
    .USER_DATA.USER_DATA15 = 0x00,
    .USER_DATA.USER_DATA16 = 0x00,
    .USER_DATA.USER_DATA17 = 0x00,
    .USER_DATA.USER_DATA18 = 0x00,
    .USER_DATA.USER_DATA19 = 0x00,
    .USER_DATA.USER_DATA20 = 0x00,

    // Time-varying gain settings
    .EEPROM.TGV.TVGAIN0.Val.Value = 0xAA, // Start gain timing for TVG
    .EEPROM.TGV.TVGAIN1.Val.Value = 0xAA, // Timing point 1
    .EEPROM.TGV.TVGAIN2.Val.Value = 0xAA, // Timing point 2
    .EEPROM.TGV.TVGAIN3.Val.Value = 0x82, // Timing point 3
    .EEPROM.TGV.TVGAIN4.Val.Value = 0x08, // Timing point 4
    .EEPROM.TGV.TVGAIN5.Val.Value = 0x20, // Timing point 5
    .EEPROM.TGV.TVGAIN6.Val.Value = 0x80, // Timing point 6 and frequency shift control

    // Gain control
    .EEPROM.INIT_GAIN.Val = 0x60, // GAIN = 0.5 × (GAIN_INIT + 1) + value(AFE_GAIN_RNG) [dB]

    // Frequency settings
    .EEPROM.FREQ = 0x8F, // Frequency = 0.2 × FREQ + 30 [kHz]

    // Dead time and comparator deglitch period
    .EEPROM.DEADTIME.Val = 0xA0, // Dead time and deglitch settings

    .EEPROM.PULSE_P1.Val = 0x04, // 4 pulses for OUTA burst (Preset 1)
    .EEPROM.PULSE_P2.Val = 0x10, // 16 pulses for OUTA burst (Preset 2)

    // Current limit settings
    .EEPROM.CURR_LIM_P1.Val = 0x55, // 435mA (Preset 1) [Current limit = 7×CURR_LIM + 50 mA]
    .EEPROM.CURR_LIM_P2.Val = 0x55, // 435mA (Preset 2)

    // Recording length
    .EEPROM.REC_LENGTH.Val = 0x19, // Recording time length = 4096 × (P_REC + 1) [µs]

    // Diagnostic frequency
    .EEPROM.FREQ_DIAG.Val = 0x33, // Frequency diagnostic settings

    // Saturation diagnostic threshold
    .EEPROM.SAT_FDIAG_TH.Val = 0xEE, // Threshold level

    // Voltage diagnostic and scaling
    .EEPROM.FVOLT_DEC.Val = 0x7C, // VPWR_OV_TH = 12.3V, other diagnostic scaling settings

    // Decoupling time and AFE gain range
    .EEPROM.DECPL_TEMP.Val = 0x4F, // Decoupling time, temperature, and AFE gain range

    // DSP scaling
    .EEPROM.DSP_SCALE.Val = 0x00, // Non-linear scaling noise level

    // Temperature trim
    .EEPROM.TEMP_TRIM.Val = 0x00, // Temperature scale offset and gain

    // Gain control for Preset 1 and 2
    .EEPROM.P1_GAIN_CTRL.Val = 0x09, // Gain control settings for Preset 1
    .EEPROM.P2_GAIN_CTRL.Val = 0x09  // Gain control settings for Preset 2
};
#elif defined(USE_UTR_1440K_TT_R) // PUI Audio UTR-1440K-TT-R configuration
const PGA460_t transducer = {
    .USER_DATA.USER_DATA1 = 0x00, // User data 1
    .USER_DATA.USER_DATA2 = 0x00, // User data 2
    .USER_DATA.USER_DATA3 = 0x00, // User data 3
    .USER_DATA.USER_DATA4 = 0x00, // User data 4
    .USER_DATA.USER_DATA5 = 0x00, // User data 5
    .USER_DATA.USER_DATA6 = 0x00, // User data 6
    .USER_DATA.USER_DATA7 = 0x00, // User data 7
    .USER_DATA.USER_DATA8 = 0x00, // User data 8
    .USER_DATA.USER_DATA9 = 0x00, // User data 9
    .USER_DATA.USER_DATA10 = 0x00, // User data 10
    .USER_DATA.USER_DATA11 = 0x00, // User data 11
    .USER_DATA.USER_DATA12 = 0x00, // User data 12
    .USER_DATA.USER_DATA13 = 0x00, // User data 13
    .USER_DATA.USER_DATA14 = 0x00, // User data 14
    .USER_DATA.USER_DATA15 = 0x00, // User data 15
    .USER_DATA.USER_DATA16 = 0x00, // User data 16
    .USER_DATA.USER_DATA17 = 0x00, // User data 17
    .USER_DATA.USER_DATA18 = 0x00, // User data 18
    .USER_DATA.USER_DATA19 = 0x00, // User data 19
    .USER_DATA.USER_DATA20 = 0x00, // User data 20

    // Time-varying gain settings
    .EEPROM.TGV.TVGAIN0.Val.Value = 0x9D, // Start gain timing for TVG
    .EEPROM.TGV.TVGAIN1.Val.Value = 0xEE, // Timing point 1
    .EEPROM.TGV.TVGAIN2.Val.Value = 0xEF, // Timing point 2
    .EEPROM.TGV.TVGAIN3.Val.Value = 0x2D, // Timing point 3
    .EEPROM.TGV.TVGAIN4.Val.Value = 0xB9, // Timing point 4
    .EEPROM.TGV.TVGAIN5.Val.Value = 0xEF, // Timing point 5
    .EEPROM.TGV.TVGAIN6.Val.Value = 0xDC, // Timing point 6 and frequency shift control

    // Gain control
    .EEPROM.INIT_GAIN.Val = 0x18, //0x03, // GAIN = 0.5 × (GAIN_INIT+1) + value(AFE_GAIN_RNG) [dB]

    // Frequency settings
    .EEPROM.FREQ = 0x32, // Frequency = 0.2 × FREQ + 30 [kHz]

    // Dead time and comparator deglitch period
    .EEPROM.DEADTIME.Val = 0x80, // Dead time and deglitch settings

		.EEPROM.PULSE_P1.Val = 0x0C, // 8 pulses for OUTA burst (Preset 1)
    .EEPROM.PULSE_P2.Val = 0x10, // 10 pulses for OUTA burst (Preset 2)

    // Current limit settings
    .EEPROM.CURR_LIM_P1.Val = 0x32, // 400mA (Preset 1) [Current limit = 7×CURR_LIM+50 mA]
    .EEPROM.CURR_LIM_P2.Val = 0x32, // 400mA (Preset 2)

    // Recording length
    .EEPROM.REC_LENGTH.Val = 0x00, // Recording time length = 4096 × (P_REC+1) [µs]

    // Diagnostic frequency
    .EEPROM.FREQ_DIAG.Val = 0x33, // Frequency diagnostic settings

    // Saturation diagnostic threshold
    .EEPROM.SAT_FDIAG_TH.Val = 0xEE, // Threshold level

    // Voltage diagnostic and scaling
    .EEPROM.FVOLT_DEC.Val = 0x02, //0x3C, // VPWR_OV_TH = 17.7V, other diagnostic scaling settings

    // Decoupling time and AFE gain range
    .EEPROM.DECPL_TEMP.Val = 0x8F, // Decoupling time, temperature, and AFE gain range

    // DSP scaling
    .EEPROM.DSP_SCALE.Val = 0x00, // Non-linear scaling noise level

    // Temperature trim
    .EEPROM.TEMP_TRIM.Val = 0x00, // Temperature scale offset and gain

    // Gain control for Preset 1 and 2
    .EEPROM.P1_GAIN_CTRL.Val = 0x05, // Gain control settings for Preset 1
    .EEPROM.P2_GAIN_CTRL.Val = 0x05, // Gain control settings for Preset 2
};
#else // User Custom XDCR Configuration
const PGA460_t transducer = {
    .USER_DATA.USER_DATA1 = 0x00,
    .USER_DATA.USER_DATA2 = 0x00,
    .USER_DATA.USER_DATA3 = 0x00,
    .USER_DATA.USER_DATA4 = 0x00,
    .USER_DATA.USER_DATA5 = 0x00,
    .USER_DATA.USER_DATA6 = 0x00,
    .USER_DATA.USER_DATA7 = 0x00,
    .USER_DATA.USER_DATA8 = 0x00,
    .USER_DATA.USER_DATA9 = 0x00,
    .USER_DATA.USER_DATA10 = 0x00,
    .USER_DATA.USER_DATA11 = 0x00,
    .USER_DATA.USER_DATA12 = 0x00,
    .USER_DATA.USER_DATA13 = 0x00,
    .USER_DATA.USER_DATA14 = 0x00,
    .USER_DATA.USER_DATA15 = 0x00,
    .USER_DATA.USER_DATA16 = 0x00,
    .USER_DATA.USER_DATA17 = 0x00,
    .USER_DATA.USER_DATA18 = 0x00,
    .USER_DATA.USER_DATA19 = 0x00,
    .USER_DATA.USER_DATA20 = 0x00,

    // Time-varying gain settings
    .EEPROM.TGV.TVGAIN0.Val.Value = 0xAA, // Start gain timing for TVG
    .EEPROM.TGV.TVGAIN1.Val.Value = 0xAA, // Timing point 1
    .EEPROM.TGV.TVGAIN2.Val.Value = 0xAA, // Timing point 2
    .EEPROM.TGV.TVGAIN3.Val.Value = 0x82, // Timing point 3
    .EEPROM.TGV.TVGAIN4.Val.Value = 0x08, // Timing point 4
    .EEPROM.TGV.TVGAIN5.Val.Value = 0x20, // Timing point 5
    .EEPROM.TGV.TVGAIN6.Val.Value = 0x80, // Timing point 6 and frequency shift control

    // Gain control
    .EEPROM.INIT_GAIN.Val = 0x60, // GAIN = 0.5 × (GAIN_INIT + 1) + value(AFE_GAIN_RNG) [dB]

    // Frequency settings
    .EEPROM.FREQ = 0x8F, // Frequency = 0.2 × FREQ + 30 [kHz]

    // Dead time and comparator deglitch period
    .EEPROM.DEADTIME.Val = 0xA0, // Dead time and deglitch settings

    .EEPROM.PULSE_P1.Val = 0x04, // 4 pulses for OUTA burst (Preset 1)
    .EEPROM.PULSE_P2.Val = 0x10, // 16 pulses for OUTA burst (Preset 2)

    // Current limit settings
    .EEPROM.CURR_LIM_P1.Val = 0x55, // 440mA (Preset 1) [Current limit = 7×CURR_LIM + 50 mA]
    .EEPROM.CURR_LIM_P2.Val = 0x55, // 440mA (Preset 2)

    // Recording length
    .EEPROM.REC_LENGTH.Val = 0x19, // Recording time length = 4096 × (P_REC + 1) [µs]

    // Diagnostic frequency
    .EEPROM.FREQ_DIAG.Val = 0x33, // Frequency diagnostic settings

    // Saturation diagnostic threshold
    .EEPROM.SAT_FDIAG_TH.Val = 0xFE, // Threshold level

    // Voltage diagnostic and scaling
    .EEPROM.FVOLT_DEC.Val = 0x7C, // VPWR_OV_TH = 12.3V, other diagnostic scaling settings

    // Decoupling time and AFE gain range
    .EEPROM.DECPL_TEMP.Val = 0x4F, // Decoupling time, temperature, and AFE gain range

    // DSP scaling
    .EEPROM.DSP_SCALE.Val = 0x00, // Non-linear scaling noise level

    // Temperature trim
    .EEPROM.TEMP_TRIM.Val = 0x00, // Temperature scale offset and gain

    // Gain control for Preset 1 and 2
    .EEPROM.P1_GAIN_CTRL.Val = 0x09, // Gain control settings for Preset 1
    .EEPROM.P2_GAIN_CTRL.Val = 0x09  // Gain control settings for Preset 2
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
    .P1_THR_0.Val.Value  = 0xFF, .P1_THR_1.Val.Value  = 0xFF, .P1_THR_2.Val.Value  = 0xFF, .P1_THR_3.Val.Value  = 0xFF,
    .P1_THR_4.Val.Value  = 0xA0, .P1_THR_5.Val.Value  = 0x80, .P1_THR_6.Val.Value  = 0x60, .P1_THR_7.Val.Value  = 0x40,
    .P1_THR_8.Val.Value  = 0x30, .P1_THR_9.Val.Value  = 0x20, .P1_THR_10.Val.Value = 0x10, .P1_THR_11           = 0x0A,
    .P1_THR_12           = 0x08, .P1_THR_13           = 0x08, .P1_THR_14           = 0x06, .P1_THR_15.Val.Value = 0x06,

    // P2
    .P2_THR_0.Val.Value  = 0xFF, .P2_THR_1.Val.Value  = 0xFF, .P2_THR_2.Val.Value  = 0xFF, .P2_THR_3.Val.Value  = 0xFF,
    .P2_THR_4.Val.Value  = 0xA0, .P2_THR_5.Val.Value  = 0x80, .P2_THR_6.Val.Value  = 0x60, .P2_THR_7.Val.Value  = 0x40,
    .P2_THR_8.Val.Value  = 0x30, .P2_THR_9.Val.Value  = 0x20, .P2_THR_10.Val.Value = 0x10, .P2_THR_11           = 0x0A,
    .P2_THR_12           = 0x08, .P2_THR_13           = 0x08, .P2_THR_14           = 0x06, .P2_THR_15.Val.Value = 0x06,

    .THR_CRC = 0x00 // You'll calculate this before EEPROM burn
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
    .TVGAIN0.Val.Value = 0x0A,  // 10%
    .TVGAIN1.Val.Value = 0x14,  // 20%
    .TVGAIN2.Val.Value = 0x1E,  // 30%
    .TVGAIN3.Val.Value = 0x28,  // 45%
    .TVGAIN4.Val.Value = 0x32,  // 60%
    .TVGAIN5.Val.Value = 0x3C,  // 75%
    .TVGAIN6.Val.Value = 0x3F   // 90%
};
#endif /* __Transducers_H */

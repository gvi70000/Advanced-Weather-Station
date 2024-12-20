#ifndef AS3935_H
#define AS3935_H

#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/// @brief I2C Address of the AS3935 sensor.
#define AS3935_I2C_ADDRESS 0x06 //(0x03 << 1)
//#define AS3935_I2C_R_ADDRESS 0x07

/// @brief Masks for extracting specific bits from registers.
#define AS3935_DISTANCE_MASK					0x3F ///< Mask for distance estimation bits.
#define AS3935_LIGHTNING_ENERGY_MASK	0x1F ///< Mask for lightning energy bits.
#define AS3935_OSC_MASK								0xE0 ///< Mask for lightning energy bits.
#define AS3935_DIRECT_COMMAND					0x96 ///< Command to reset the sensor to default values.
#define AS3935_CMD_DELAY					4 
#define AS3935_CAL_DELAY					250
/// @brief Register addresses for the AS3935 sensor.
typedef enum {
    AFE_GAIN             = 0x00, ///< Power management and AFE gain boost.
    THRESHOLD            = 0x01, ///< Noise floor and watchdog threshold.
    LIGHTNING_REG        = 0x02, ///< Spike rejection and lightning statistics.
    INT_MASK_ANT         = 0x03, ///< Interrupts and antenna tuning.
    ENERGY_LIGHT_LSB     = 0x04, ///< Lightning energy LSB.
    ENERGY_LIGHT_MSB     = 0x05, ///< Lightning energy MSB.
    ENERGY_LIGHT_MMSB    = 0x06, ///< Lightning energy MMSB (5 bits only).
    DISTANCE             = 0x07, ///< Distance estimation.
    FREQ_DISP_IRQ        = 0x08, ///< Oscillator tuning and IRQ configuration.
    CALIB_TRCO           = 0x3A, ///< Calibration status for TRCO.
    CALIB_SRCO           = 0x3B, ///< Calibration status for SRCO.
    PRESET_DEFAULT       = 0x3C, ///< Restore default settings.
    CALIB_RCO            = 0x3D  ///< Recalibrate internal oscillators.
} AS3935_Registers_t;

//Register 0x00 Power management and AFE gain boost.
/**
 * @brief Enumeration for powering up or down the AS3935 sensor.
 */
typedef enum {
    AS3935_POWER_DOWN = 0x01, ///< Power down the sensor.
    AS3935_POWER_UP   = 0x00  ///< Power up the sensor.
} AS3935_PowerState_t;

/**
 * @brief Enumeration for AFE Gain Settings.
 * 
 * Represents the possible Analog Front End (AFE) gain configurations
 * for the AS3935 sensor, as defined in REG0x00[5:1].
 */
typedef enum {
    AFE_GAIN_OUTDOOR = 0x0E, ///< Outdoor mode (01110 in binary).
    AFE_GAIN_INDOOR  = 0x12  ///< Indoor mode (10010 in binary).
} AS3935_AFE_Gain_t;

/**
 * @brief Power and AFE gain configuration register (0x00).
 * 
 * This structure represents the configuration of the AS3935 sensor's power 
 * state and Analog Front-End (AFE) gain settings. The register includes:
 * - PWD: Controls the power state of the sensor (1 = power down, 0 = power up).
 * - GAIN: Sets the AFE gain boost level (0-31). Higher values increase the sensitivity.
 * - Reserved: Reserved bits that must always be set to 0.
 * 
 * The union allows access to either the complete 8-bit register value or 
 * individual fields through the `BitField` structure.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            uint8_t PWD         : 1; ///< Power-down control (1 = power down, 0 = power up).
            uint8_t GAIN        : 5; ///< AFE gain boost (0-31).
            uint8_t Reserved    : 2; ///< Reserved bits (default 0).
        } BitField; ///< Individual bit fields.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_PWR_t;

//Register 0x01 Noise floor and watchdog threshold.
/**
 * @brief Enumeration for WDTH (Detection Efficiency Width Settings).
 * 
 * Specifies the width setting for detection efficiency with descriptive names.
 */
typedef enum {
    EFFICIENCY_100 = 0x00, ///< WDTH 0000: Highest detection efficiency (~100%)
    EFFICIENCY_90  = 0x01, ///< WDTH 0001: ~90% detection efficiency
    EFFICIENCY_80  = 0x02, ///< WDTH 0010: ~80% detection efficiency
    EFFICIENCY_70  = 0x03, ///< WDTH 0011: ~70% detection efficiency
    EFFICIENCY_60  = 0x04, ///< WDTH 0100: ~60% detection efficiency
    EFFICIENCY_50  = 0x05, ///< WDTH 0101: ~50% detection efficiency
    EFFICIENCY_40  = 0x06, ///< WDTH 0110: ~40% detection efficiency
    EFFICIENCY_30  = 0x07, ///< WDTH 0111: ~30% detection efficiency
    EFFICIENCY_20  = 0x08, ///< WDTH 1000: ~20% detection efficiency
    EFFICIENCY_10  = 0x09, ///< WDTH 1001: ~10% detection efficiency
    EFFICIENCY_0   = 0x0A  ///< WDTH 1010: Lowest detection efficiency (~0%)
} AS3935_WDTH_t;

/**
 * @brief Enumeration for noise floor threshold settings.
 * 
 * Maps the noise floor threshold levels to the corresponding register values.
 */
typedef enum {
    NOISE_FLOOR_390uVrms  = 0x00, ///< Outdoor: 390 µVrms, Indoor: 28 µVrms
    NOISE_FLOOR_630uVrms  = 0x01, ///< Outdoor: 630 µVrms, Indoor: 45 µVrms
    NOISE_FLOOR_860uVrms  = 0x02, ///< Outdoor: 860 µVrms, Indoor: 62 µVrms
    NOISE_FLOOR_1100uVrms = 0x03, ///< Outdoor: 1100 µVrms, Indoor: 78 µVrms
    NOISE_FLOOR_1140uVrms = 0x04, ///< Outdoor: 1140 µVrms, Indoor: 95 µVrms
    NOISE_FLOOR_1570uVrms = 0x05, ///< Outdoor: 1570 µVrms, Indoor: 112 µVrms
    NOISE_FLOOR_1800uVrms = 0x06, ///< Outdoor: 1800 µVrms, Indoor: 130 µVrms
    NOISE_FLOOR_2000uVrms = 0x07  ///< Outdoor: 2000 µVrms, Indoor: 146 µVrms
} AS3935_NoiseFloorLevel_t;

/**
 * @brief Noise floor and watchdog threshold configuration register (0x01).
 * 
 * This structure represents the configuration of the AS3935 sensor's noise 
 * floor level and watchdog threshold. The register includes:
 * - WDTH: Configures the watchdog threshold to adjust detection sensitivity.
 * - NF_LEV: Sets the noise floor level to filter out environmental noise.
 * - Reserved: A reserved bit that must always be set to 0.
 * 
 * The union allows access to either the complete 8-bit register value or 
 * individual fields through the `BitField` structure.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            uint8_t WDTH      : 4; ///< Watchdog threshold (0-15), adjusts detection sensitivity.
            uint8_t NF_LEV    : 3; ///< Noise floor level (0-7), filters out environmental noise.
            uint8_t Reserved  : 1; ///< Reserved bit (always 0).
        } BitField; ///< Individual bit fields.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_NOISE_t;

//Register 0x02 Spike rejection and lightning statistics.

/**
 * @brief Enumeration for SREJ (Spike Rejection Settings).
 * 
 * Specifies the spike rejection threshold, affecting detection efficiency and distance.
 */
typedef enum {
    SREJ_LEVEL_0  = 0x00, ///< Spike rejection level 0000: Highest detection efficiency
    SREJ_LEVEL_1  = 0x01, ///< Spike rejection level 0001
    SREJ_LEVEL_2  = 0x02, ///< Spike rejection level 0010
    SREJ_LEVEL_3  = 0x03, ///< Spike rejection level 0011
    SREJ_LEVEL_4  = 0x04, ///< Spike rejection level 0100
    SREJ_LEVEL_5  = 0x05, ///< Spike rejection level 0101
    SREJ_LEVEL_6  = 0x06, ///< Spike rejection level 0110
    SREJ_LEVEL_7  = 0x07, ///< Spike rejection level 0111
    SREJ_LEVEL_8  = 0x08, ///< Spike rejection level 1000
    SREJ_LEVEL_9  = 0x09, ///< Spike rejection level 1001
    SREJ_LEVEL_10 = 0x0A, ///< Spike rejection level 1010
    SREJ_LEVEL_11 = 0x0B  ///< Spike rejection level 1011: Lowest detection efficiency
} AS3935_SREJ_t;

/**
 * @brief Enumeration for minimum number of lightning events for detection.
 * 
 * Maps the minimum number of lightning events to the corresponding register bits.
 */
typedef enum {
    MIN_NUM_LIGHTNING_1  = 0x00, ///< Minimum of 1 lightning event (00 in REG0x02[5:4])
    MIN_NUM_LIGHTNING_5  = 0x01, ///< Minimum of 5 lightning events (01 in REG0x02[5:4])
    MIN_NUM_LIGHTNING_9  = 0x02, ///< Minimum of 9 lightning events (10 in REG0x02[5:4])
    MIN_NUM_LIGHTNING_16 = 0x03  ///< Minimum of 16 lightning events (11 in REG0x02[5:4])
} AS3935_MinLightning_t;

/**
 * @brief Enumeration for clearing lightning detection statistics (CL_STAT).
 * 
 * The `CL_STAT` bit in register 0x02 is used to clear the lightning detection statistics.
 * - Setting `CL_STAT` to `CLEAR_STAT_ENABLED` clears the statistics.
 * - Setting `CL_STAT` to `CLEAR_STAT_DISABLED` leaves the statistics unchanged.
 */
typedef enum {
    CLEAR_STAT_DISABLED = 0, ///< Do not clear lightning detection statistics.
    CLEAR_STAT_ENABLED  = 1  ///< Clear lightning detection statistics.
} AS3935_ClearStat_t;

/// @brief Lightning and spike rejection configuration register (0x02).
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            uint8_t SREJ        : 4; ///< Spike rejection level (0-15).
            uint8_t MIN_NUM_LIGH: 2; ///< Minimum lightning events for validation.
            uint8_t CL_STAT     : 1; ///< Clear statistics (1 = clear).
            uint8_t Reserved    : 1; ///< Reserved bit (default 1).
        } BitField; ///< Individual bit fields.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_STATISTICS_t;

//Register 0x03 Interrupts and antenna tuning.

/**
 * @brief Interrupt types (REG0x03[3:0]) that the sensor can generate.
 * 
 * The AS3935 sensor can generate the following types of interrupts:
 * - INT_NH: Triggered when the noise level is too high.
 * - INT_D: Triggered when a disturber is detected.
 * - INT_L: Triggered when a lightning event is detected.
 */
typedef enum {
    INT_NH = 0x01,  ///< Noise level too high.
    INT_D  = 0x04,  ///< Disturber detected.
    INT_L  = 0x08   ///< Lightning detected.
} AS3935_INT_t;

/**
 * @brief Enumeration for configuring the MASK_DIST bit.
 * 
 * This enum determines whether disturber events are masked or allowed to trigger 
 * an interrupt (INT_D). It maps directly to the MASK_DIST bit in REG0x03[5].
 */
typedef enum {
    MASK_DIST_DISABLED = 0, ///< Do not mask disturber events (INT_D enabled for disturbers).
    MASK_DIST_ENABLED  = 1  ///< Mask disturber events (INT_D disabled for disturbers).
} AS3935_MaskDist_t;

/**
 * @brief Enumeration for frequency division ratio (LCO_FDIV).
 * 
 * Determines the division ratio for the antenna frequency.
 */
typedef enum {
    LCO_FDIV_RATIO_16		= 0x00, ///< Frequency divided by 16.
    LCO_FDIV_RATIO_32		= 0x01, ///< Frequency divided by 32.
    LCO_FDIV_RATIO_64		= 0x02, ///< Frequency divided by 64.
    LCO_FDIV_RATIO_128	= 0x03  ///< Frequency divided by 128.
} AS3935_LCO_FDiv_t;

/**
 * @brief Interrupt and frequency tuning configuration register (0x03).
 * 
 * This structure represents the configuration for the interrupt types and 
 * frequency division ratio in the AS3935 sensor. It includes:
 * - INT: Interrupt type (REG0x03[3:0]).
 * - MASK_DIST: Mask disturber flag (REG0x03[5], 1 = mask disturbers).
 * - LCO_FDIV: Frequency division ratio (REG0x03[7:6], values 0-3).
 * - Reserved: Reserved bit (always 0, REG0x03[4]).
 * 
 * The union allows access to either the full register value or individual fields.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            uint8_t INT         : 4; ///< Interrupt type.
            uint8_t Reserved    : 1; ///< Reserved bit (default 0).
            uint8_t MASK_DIST   : 1; ///< Mask disturber (1 = mask).
            uint8_t LCO_FDIV    : 2; ///< Frequency division ratio (0-3).
        } BitField; ///< Individual bit fields.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_INT_FREQ_t;

//Register 0x04-0x07 Energy of the Single Lightning LSBYTE, MSBYTE, MMSBYTE,  Distance estimation

/**
 * @brief Distance estimation results from the sensor.
 * 
 * This enumeration represents the estimated distance to a storm detected by the AS3935 sensor.
 * The values correspond to specific distance ranges or states:
 * - DIST_OUT_OF_RANGE: The distance is out of the measurable range.
 * - DIST_STORM_OVERHEAD: The storm is directly overhead (closest possible).
 * - DIST_6_KM to DIST_40_KM: Distance in kilometers to the storm.
 */
typedef enum {
    DIST_OUT_OF_RANGE    = 0x3F, ///< Out of range.
    DIST_STORM_OVERHEAD  = 0x01, ///< Storm overhead (closest).
    DIST_6_KM            = 0x02, ///< 6 kilometers.
    DIST_8_KM            = 0x03, ///< 8 kilometers.
    DIST_10_KM           = 0x04, ///< 10 kilometers.
    DIST_12_KM           = 0x05, ///< 12 kilometers.
    DIST_14_KM           = 0x06, ///< 14 kilometers.
    DIST_17_KM           = 0x07, ///< 17 kilometers.
    DIST_20_KM           = 0x08, ///< 20 kilometers.
    DIST_24_KM           = 0x09, ///< 24 kilometers.
    DIST_27_KM           = 0x0A, ///< 27 kilometers.
    DIST_31_KM           = 0x0B, ///< 31 kilometers.
    DIST_34_KM           = 0x0C, ///< 34 kilometers.
    DIST_37_KM           = 0x0D, ///< 37 kilometers.
    DIST_40_KM           = 0x0E  ///< 40 kilometers.
} AS3935_Distance_t;

/**
 * @brief Lightning energy and distance estimation registers (0x04-0x07).
 * 
 * This structure combines lightning energy (20 bits) and distance estimation
 * into a single representation, as read from registers 0x04 to 0x07.
 */
typedef struct __attribute__((packed)) {
    union {
        struct {
            uint8_t LSB;        ///< Least significant byte of lightning energy (0x04).
            uint8_t MSB;        ///< Most significant byte of lightning energy (0x05).
            struct {
                uint8_t MMSB			: 5; ///< Most most significant byte (5 bits valid, 0x06[4:0]).
                uint8_t Reserved	: 3; ///< Reserved bits (0x06[7:5], always 0).
            };
            struct {
                uint8_t Distance	: 6; ///< Distance estimation (0x07[5:0]).
                uint8_t Reserved2	: 2; ///< Reserved bits (0x07[7:6], always 0).
            };
        };
        uint8_t ByteArray[4];   ///< Registers 0x04 to 0x07 as a byte array.
        struct {
            uint32_t LightningEnergy : 20; ///< Combined 20-bit lightning energy value.
            uint8_t DistanceEstimation;    ///< 6-bit distance estimation.
        };
    };
} AS3935_Energy_t;

//Register 0x08 Oscillator tuning and IRQ configuration.

/**
 * @brief Enumeration for internal tuning capacitor settings.
 * 
 * TUN_CAP allows setting capacitance from 0 to 120pF in steps of 8pF.
 */
typedef enum {
    TUN_CAP_0PF    = 0x00, ///< 0 pF
    TUN_CAP_8PF    = 0x01, ///< 8 pF
    TUN_CAP_16PF   = 0x02, ///< 16 pF
    TUN_CAP_24PF   = 0x03, ///< 24 pF
    TUN_CAP_32PF   = 0x04, ///< 32 pF
    TUN_CAP_40PF   = 0x05, ///< 40 pF
    TUN_CAP_48PF   = 0x06, ///< 48 pF
    TUN_CAP_56PF   = 0x07, ///< 56 pF
    TUN_CAP_64PF   = 0x08, ///< 64 pF
    TUN_CAP_72PF   = 0x09, ///< 72 pF
    TUN_CAP_80PF   = 0x0A, ///< 80 pF
    TUN_CAP_88PF   = 0x0B, ///< 88 pF
    TUN_CAP_96PF   = 0x0C, ///< 96 pF
    TUN_CAP_104PF  = 0x0D, ///< 104 pF
    TUN_CAP_112PF  = 0x0E, ///< 112 pF
    TUN_CAP_120PF  = 0x0F  ///< 120 pF
} AS3935_TUNE_CAP_t;

/**
 * @brief Enumeration for configuring oscillator display on IRQ pin.
 * 
 * This enum represents all possible combinations of TRCO, SRCO, and LCO oscillator
 * display settings on the IRQ pin. These settings allow specific oscillators to be displayed 
 * for diagnostic purposes.
 */
typedef enum {
    OSC_DISPLAY_DISABLED				= 0, ///< All oscillators disabled (000 in binary).
    OSC_DISPLAY_TRCO_ONLY				= 1, ///< TRCO oscillator only enabled (001 in binary). System RCO at 32.768kHz
    OSC_DISPLAY_SRCO_ONLY				= 2, ///< SRCO oscillator only enabled (010 in binary). Timer RCO Oscillators 1.1MHz
    OSC_DISPLAY_LCO_ONLY				= 4  ///< LCO oscillator only enabled (100 in binary). Frequency of the Antenna 500kHz
} AS3935_OSCDisplay_t;

/**
 * @brief IRQ and tuning configuration register (0x08).
 * 
 * This structure represents the configuration of the AS3935 sensor's internal tuning capacitors 
 * and oscillator display settings on the IRQ pin. It allows users to adjust the antenna's tuning 
 * capacitance and configure which oscillator(s) (TRCO, SRCO, LCO) are displayed on the IRQ pin 
 * for diagnostic purposes.
 * 
 * The register includes:
 * - TUN_CAP: Internal tuning capacitors (bits 3:0).
 * - Reserved: Reserved bit that must always be set to 0 (bit 4).
 * - OSC_DISP: Configures which oscillator(s) are displayed on the IRQ pin (bits 7:5).
 * 
 * The union provides access to the full 8-bit register value or individual fields via the `BitField` structure.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            uint8_t TUN_CAP   : 4; ///< Internal tuning capacitors (0-15) for antenna optimization.
            uint8_t Reserved  : 1; ///< Reserved bit (always 0, unused).
            uint8_t OSC_DISP  : 3; ///< Oscillator display configuration (TRCO, SRCO, LCO; bits 7:5).
        } BitField; ///< Individual bit fields for fine-grained control.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_IRQ_t;

/**
 * @brief Calibration status register for TRCO (Timer RC Oscillator) (0x3A).
 * 
 * This structure represents the calibration status of the Timer RC Oscillator (TRCO) in the AS3935 sensor.
 * It indicates whether the calibration was successful or not. 
 * The register includes:
 * - Reserved: Reserved bits (always 0, bits 5:0).
 * - TRCO_CALIB_NOK: Indicates if TRCO calibration failed (bit 6).
 * - TRCO_CALIB_DONE: Indicates if TRCO calibration was completed successfully (bit 7).
 * 
 * The union allows access to the complete register value or individual fields for detailed status.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            uint8_t Reserved        : 6; ///< Reserved bits (always 0).
            uint8_t TRCO_CALIB_NOK  : 1; ///< TRCO calibration unsuccessful (1 = failed).
            uint8_t TRCO_CALIB_DONE : 1; ///< TRCO calibration completed successfully (1 = success).
        } BitField; ///< Individual bit fields.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_TRCO_t;


/**
 * @brief Calibration status register for SRCO (System RC Oscillator) (0x3B).
 * 
 * This structure represents the calibration status of the System RC Oscillator (SRCO) in the AS3935 sensor.
 * It indicates whether the calibration was successful or not.
 * The register includes:
 * - Reserved: Reserved bits (always 0, bits 5:0).
 * - SRCO_CALIB_NOK: Indicates if SRCO calibration failed (bit 6).
 * - SRCO_CALIB_DONE: Indicates if SRCO calibration was completed successfully (bit 7).
 * 
 * The union allows access to the complete register value or individual fields for detailed status.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            uint8_t Reserved        : 6; ///< Reserved bits (always 0).
            uint8_t SRCO_CALIB_NOK  : 1; ///< SRCO calibration unsuccessful (1 = failed).
            uint8_t SRCO_CALIB_DONE : 1; ///< SRCO calibration completed successfully (1 = success).
        } BitField; ///< Individual bit fields.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_SRCO_t;

/// @brief Complete register map of the AS3935 sensor.
typedef struct __attribute__((packed)) {
    AS3935_PWR_t POWER;             ///< Power management and AFE Gain Boost (0x00).
    AS3935_NOISE_t NOISE;           ///< Noise floor and watchdog threshold (0x01).
    AS3935_STATISTICS_t STATISTICS; ///< Lightning event and spike rejection (0x02).
    AS3935_INT_FREQ_t INT_FREQ;     ///< Interrupt and antenna tuning (0x03).
    AS3935_Energy_t ENERGY;         ///< Lightning energy and distance registers (0x04-0x07).
    AS3935_IRQ_t IRQ;               ///< IRQ configuration and tuning (0x08).
    AS3935_TRCO_t TRCO;             ///< TRCO calibration status (0x3A).
    AS3935_SRCO_t SRCO;             ///< SRCO calibration status (0x3B).
} AS3935_REGS_t;

// Function prototypes
HAL_StatusTypeDef AS3935_Init(void);
HAL_StatusTypeDef AS3935_ReadAllRegisters(void);
HAL_StatusTypeDef AS3935_WakeUp(void);
HAL_StatusTypeDef AS3935_SetPower(AS3935_PowerState_t powerState);
HAL_StatusTypeDef AS3935_SetGain(AS3935_AFE_Gain_t gain);
HAL_StatusTypeDef AS3935_SetWatchdog(AS3935_WDTH_t watchdogThreshold);
HAL_StatusTypeDef AS3935_SetNoise(AS3935_NoiseFloorLevel_t noiseLevel);
HAL_StatusTypeDef AS3935_SetSpikeRejectionLevel(AS3935_SREJ_t spikeRejectionLevel);
HAL_StatusTypeDef AS3935_SetLightningNo(AS3935_MinLightning_t noLightningEvents);
HAL_StatusTypeDef AS3935_ClearStatistics(AS3935_ClearStat_t clearStat);
HAL_StatusTypeDef AS3935_SetInterruptType(AS3935_INT_t interrupt);
HAL_StatusTypeDef AS3935_SetdDisturberMask(AS3935_MaskDist_t maskDist);
HAL_StatusTypeDef AS3935_setFrequencyDivisionRatio(AS3935_LCO_FDiv_t fdivRatio);
HAL_StatusTypeDef AS3935_ReadLightningEnergyAndDistance(AS3935_Energy_t *energy);
HAL_StatusTypeDef AS3935_setTuningCapacitor(AS3935_TUNE_CAP_t tuningCap);
HAL_StatusTypeDef AS3935_SetOscillatorDisplay(AS3935_OSCDisplay_t oscDisplay);
HAL_StatusTypeDef AS3935_ReadTRCOCalibrationStatus(void);
HAL_StatusTypeDef AS3935_ReadSRCOCalibrationStatus(void);
HAL_StatusTypeDef AS3935_CalibrateOscillators(void);
HAL_StatusTypeDef AS3935_RecalibrateAfterPowerDown(void);
uint8_t AS3935_TuneAntenna(void);
#endif // AS3935_H
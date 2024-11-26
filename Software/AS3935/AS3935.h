#ifndef AS3935_H
#define AS3935_H

#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/// @brief I2C Address of the AS3935 sensor.
#define AS3935_I2C_ADDRESS (0x03 << 1)


/// @brief Masks for extracting specific bits from registers.
#define AS3935_DISTANCE_MASK          0x3F   ///< Mask for distance estimation bits.
#define AS3935_LIGHTNING_ENERGY_MASK  0x1F   ///< Mask for lightning energy bits.
#define AS3935_RESET_COMMAND          0x96   ///< Command to reset the sensor to default values.

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
} AS3935_Register_t;

/// @brief Interrupt types (REG0x03[3:0]) that the sensor can generate.
typedef enum {
    INT_NH = 0x01,  ///< Noise level too high.
    INT_D  = 0x04,  ///< Disturber detected.
    INT_L  = 0x08   ///< Lightning detected.
} AS3935_INT_t;

/// @brief Distance estimation results from the sensor.
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

/// @brief Power and AFE gain configuration register (0x00).
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            uint8_t POWER    : 1; ///< Power-down control (1 = power down).
            uint8_t GAIN     : 5; ///< AFE gain boost (0-31).
            uint8_t Reserved : 2; ///< Reserved bits (always 0).
        } BitField; ///< Individual bit fields.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_PWR_t;

/// @brief Noise floor and watchdog threshold configuration register (0x01).
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            uint8_t WDTH     : 4; ///< Watchdog threshold (0-15).
            uint8_t NF_LEV   : 3; ///< Noise floor level (0-7).
            uint8_t Reserved : 1; ///< Reserved bit (always 0).
        } BitField; ///< Individual bit fields.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_NOISE_t;

/// @brief Lightning and spike rejection configuration register (0x02).
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            uint8_t SREJ        : 4; ///< Spike rejection level (0-15).
            uint8_t MIN_NUM_LIGH: 2; ///< Minimum lightning events for validation.
            uint8_t CL_STAT     : 1; ///< Clear statistics (1 = clear).
            uint8_t Reserved    : 1; ///< Reserved bit (always 0).
        } BitField; ///< Individual bit fields.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_STATISTICS_t;

/// @brief Interrupt and frequency tuning configuration register (0x03).
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            AS3935_INT_t INT    : 4; ///< Interrupt type.
            uint8_t Reserved    : 1; ///< Reserved bit (always 0).
            uint8_t MASK_DIST   : 1; ///< Mask disturber (1 = mask).
            uint8_t LCO_FDIV    : 2; ///< Frequency division ratio (0-3).
        } BitField; ///< Individual bit fields.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_INT_FREQ_t;

/// @brief Lightning energy registers (0x04-0x06).
typedef struct __attribute__((packed)) {
    union {
        struct {
            uint8_t LSB;        ///< Least significant byte of lightning energy.
            uint8_t MSB;        ///< Most significant byte of lightning energy.
            uint8_t MMSB;       ///< Most most significant byte (5 bits valid).
            uint8_t Reserved;   ///< Reserved for alignment.
        };
        uint8_t ByteArray[4];   ///< Lightning energy as a byte array.
        uint32_t Value;         ///< Combined 20-bit lightning energy value.
    };
} AS3935_Energy_t;

/// @brief IRQ and tuning configuration register (0x08).
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            uint8_t TUN_CAP  : 4; ///< Internal tuning capacitors (0-15).
            uint8_t Reserved : 1; ///< Reserved bit (always 0).
            uint8_t DISP_TRCO: 1; ///< Display TRCO on IRQ pin.
            uint8_t DISP_SRCO: 1; ///< Display SRCO on IRQ pin.
            uint8_t DISP_LCO : 1; ///< Display LCO on IRQ pin.
        } BitField; ///< Individual bit fields.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_IRQ_t;

/// @brief Calibration status register for TRCO (0x3A).
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            uint8_t Reserved        : 6; ///< Reserved bits (always 0).
            uint8_t TRCO_CALIB_NOK  : 1; ///< TRCO calibration unsuccessful.
            uint8_t TRCO_CALIB_DONE : 1; ///< TRCO calibration completed successfully.
        } BitField; ///< Individual bit fields.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_TRCO_t;

/// @brief Calibration status register for SRCO (0x3B).
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Complete register value.
        struct __attribute__((packed)) {
            uint8_t Reserved        : 6; ///< Reserved bits (always 0).
            uint8_t SRCO_CALIB_NOK  : 1; ///< SRCO calibration unsuccessful.
            uint8_t SRCO_CALIB_DONE : 1; ///< SRCO calibration completed successfully.
        } BitField; ///< Individual bit fields.
    } Val; ///< Union for accessing full value or bit fields.
} AS3935_SRCO_t;

/// @brief Complete register map of the AS3935 sensor.
typedef struct __attribute__((packed)) {
    AS3935_PWR_t POWER;             ///< Power management and AFE Gain Boost (0x00).
    AS3935_NOISE_t NOISE;           ///< Noise floor and watchdog threshold (0x01).
    AS3935_STATISTICS_t STATISTICS; ///< Lightning event and spike rejection (0x02).
    AS3935_INT_FREQ_t INT_FREQ;     ///< Interrupt and antenna tuning (0x03).
    AS3935_Energy_t ENERGY;         ///< Lightning energy registers (0x04-0x06).
    uint8_t DISTANCE;               ///< Distance estimation (0x07).
    AS3935_IRQ_t IRQ;               ///< IRQ configuration and tuning (0x08).
    AS3935_TRCO_t TRCO;             ///< TRCO calibration status (0x3A).
    AS3935_SRCO_t SRCO;             ///< SRCO calibration status (0x3B).
} AS3935_REGS_t;

// Function prototypes
HAL_StatusTypeDef AS3935_Init(void);
void AS3935_PowerDown(void);
bool AS3935_WakeUp(void);
void AS3935_RecalibrateAfterPowerDown(void);
bool AS3935_CalibrateOscillators(void);
void AS3935_DisplayOscillator(bool enable, uint8_t oscillator);
void AS3935_SetTuningCapacitance(uint8_t capacitance);
void AS3935_SetFrequencyDivision(uint8_t divisionRatio);
void AS3935_EnableAntennaFrequencyDisplay(bool enable);
AS3935_Distance_t AS3935_GetDistanceToStorm(void);
uint32_t AS3935_GetLightningEnergy(void);
void AS3935_SetNoiseFloorLevel(uint8_t level);
uint8_t AS3935_ReadNoiseFloorLevel(void);
void AS3935_MaskDisturber(bool mask);
uint8_t AS3935_ReadInterruptStatus(void);
void AS3935_SetLightningEventThreshold(uint8_t threshold);
void AS3935_ClearStatistics(void);
void AS3935_SendDirectCommand(uint8_t command);
void AS3935_SetAFEGain(uint8_t gain);

#endif // AS3935_H

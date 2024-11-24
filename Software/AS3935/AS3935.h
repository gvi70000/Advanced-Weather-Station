#ifndef AS3935_H
#define AS3935_H

#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// I2C Address
#define AS3935_I2C_ADDRESS (0x03 << 1)

// Common macros
#define AS3935_DISTANCE_MASK					0x3F
#define AS3935_LIGHTNING_ENERGY_MASK	0x1F
#define AS3935_RESET_COMMAND					0x96

// Enum for Interrupt Name REG0x03[3:0]
typedef enum {
    INT_NH = 0x01,  // Noise level too high
    INT_D  = 0x04,  // Disturber detected
    INT_L  = 0x08   // Lightning interrupt
} AS3935_INT_t;

// Enum for Distance Interpretation
typedef enum {
    DIST_OUT_OF_RANGE = 0x3F,
    DIST_STORM_OVERHEAD = 0x01,
    DIST_6_KM = 0x02,
    DIST_8_KM = 0x03,
    DIST_10_KM = 0x04,
    DIST_12_KM = 0x05,
    DIST_14_KM = 0x06,
    DIST_17_KM = 0x07,
    DIST_20_KM = 0x08,
    DIST_24_KM = 0x09,
    DIST_27_KM = 0x0A,
    DIST_31_KM = 0x0B,
    DIST_34_KM = 0x0C,
    DIST_37_KM = 0x0D,
    DIST_40_KM = 0x0E
} AS3935_Distance_t;

// Enum for Register Addresses
typedef enum {
    AFE_GAIN = 0x00,
    THRESHOLD = 0x01,
    LIGHTNING_REG = 0x02,
    INT_MASK_ANT = 0x03,
    ENERGY_LIGHT_LSB = 0x04,
    ENERGY_LIGHT_MSB = 0x05,
    ENERGY_LIGHT_MMSB = 0x06,
    DISTANCE = 0x07,
    FREQ_DISP_IRQ = 0x08,
    CALIB_TRCO = 0x3A,
    CALIB_SRCO = 0x3B,
    PRESET_DEFAULT = 0x3C,
    CALIB_RCO = 0x3D
} AS3935_Register_t;

// 0x00 R/W Structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t POWER : 1;    // 0 Power-down
            uint8_t GAIN : 5;     // 1-5 AFE Gain Boost
            uint8_t Reserved : 2; // 6-7 Reserved
        } BitField;
    } Val;
} AS3935_PWR_t;

// 0x01 R/W Structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t WDTH : 4;     // 0-3 Watchdog threshold
            uint8_t NF_LEV : 3;   // 4-6 Noise Floor Level
            uint8_t Reserved : 1; // 7 Reserved
        } BitField;
    } Val;
} AS3935_NOISE_t;

// 0x02 R/W Structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t SREJ : 4;         // 0-3 Spike rejection
            uint8_t MIN_NUM_LIGH : 2; // 4-5 Minimum number of lightning
            uint8_t CL_STAT : 1;      // 6 Clear statistics
            uint8_t Reserved : 1;     // 7 Reserved
        } BitField;
    } Val;
} AS3935_STATISTICS_t;

// 0x03 R/W Structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            AS3935_INT_t INT : 4; // 0-3 Interrupt
            uint8_t Reserved : 1; // 4 Reserved
            uint8_t MASK_DIST : 1; // 5 Mask Disturber
            uint8_t LCO_FDIV : 2;  // 6-7 Frequency division ratio for antenna tuning
        } BitField;
    } Val;
} AS3935_INT_FREQ_t;

// 0x08 R/W Structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t TUN_CAP : 4;   // 0-3 Internal Tuning Capacitors (0-120pF in 8pF steps)
            uint8_t Reserved : 1;  // 4 Reserved
            uint8_t DISP_TRCO : 1; // 5 Display TRCO on IRQ pin
            uint8_t DISP_SRCO : 1; // 6 Display SRCO on IRQ pin
            uint8_t DISP_LCO : 1;  // 7 Display LCO on IRQ pin
        } BitField;
    } Val;
} AS3935_IRQ_t;

// 0x3A R/W Structure for TRCO Calibration
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t Reserved : 6;        // 0-5 Reserved
            uint8_t TRCO_CALIB_NOK : 1;  // 6 Calibration of TRCO unsuccessful (1 = not successful)
            uint8_t TRCO_CALIB_DONE : 1; // 7 Calibration of TRCO done (1 = successful)
        } BitField;
    } Val;
} AS3935_TRCO_t;

// 0x3B R/W Structure for SRCO Calibration
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t Reserved : 6;        // 0-5 Reserved
            uint8_t SRCO_CALIB_NOK : 1;  // 6 Calibration of SRCO unsuccessful (1 = not successful)
            uint8_t SRCO_CALIB_DONE : 1; // 7 Calibration of SRCO done (1 = successful)
        } BitField;
    } Val;
} AS3935_SRCO_t;

// Complete Register Structure
typedef struct __attribute__((packed)) {
    AS3935_PWR_t POWER;             // 0x00 Power management and AFE Gain Boost
    AS3935_NOISE_t NOISE;         // 0x01 Noise and Watchdog threshold
    AS3935_STATISTICS_t STATISTICS; // 0x02 Spike rejection and lightning statistics
    AS3935_INT_FREQ_t INT_FREQ;   // 0x03 Interrupts and frequency tuning
    uint8_t S_LIG_L;              // 0x04 Energy of the Single Lightning LSBYTE
    uint8_t S_LIG_M;              // 0x05 Energy of the Single Lightning MSBYTE
    uint8_t S_LIG_MM;             // 0x06 Energy of the Single Lightning MMSBYTE (only bits 0-4)
    uint8_t DISTANCE;             // 0x07 Distance estimation (only bits 0-5)
    AS3935_IRQ_t IRQ;             // 0x08 IRQ configuration and tuning capacitors
    AS3935_TRCO_t TRCO;           // 0x3A Calibration of TRCO
    AS3935_SRCO_t SRCO;           // 0x3B Calibration of SRCO
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

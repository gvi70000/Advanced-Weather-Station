#ifndef AS7331_H
#define AS7331_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

// AS7331 I2C Address
#define AS7331_I2C_ADDR				(0x74 << 1) // ADDR 1, 2 LOW

#define AS7331_ID							0x21 // 

#define AS7331_TEMP1					5
#define AS7331_TEMP2					6690

// Register Addresses
#define AS7331_REG_OSR        0x00
#define AS7331_REG_TEMP				0x01
#define AS7331_REG_AGEN       0x02
#define AS7331_REG_MRES2      0x03
#define AS7331_REG_MRES3			0x04
#define AS7331_REG_OUTCONVL		0x05
#define AS7331_REG_CREG1      0x06
#define AS7331_REG_CREG2      0x07
#define AS7331_REG_CREG3      0x08
#define AS7331_REG_BREAK      0x09
#define AS7331_REG_EDGES      0x0A
#define AS7331_REG_OPTREG     0x0B

// --------------------------- Enums and Structures ---------------------------
// OSR Register Enums and Structure
typedef enum {
    STOP_MEASUREMENT	= 0,
		START_MEASUREMENT	= 1
} AS7331_SS_t;

typedef enum {
    POWER_OFF	= 0,
    POWER_ON	= 1
} AS7331_PWR_t;

typedef enum {
    SW_RESET_OFF	= 0,
    SW_RESET_ON		= 1
} AS7331_SW_RST_t;

typedef enum {
    MODE_NOP						= 0b000,
    MODE_CONFIGURATION	= 0b010,
    MODE_MEASUREMENT		= 0b011,
} AS7331_DOS_t;
// OSR and STATUS Register Structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; /**< Complete register value as a byte */
        struct {
            uint8_t DOS        : 3; /**< Device operational state (bits 0–2, OSR) */
            uint8_t SW_RES     : 1; /**< Software reset (bit 3, OSR) */
            uint8_t RESERVED   : 2; /**< Reserved (bits 4–5, OSR) */
            uint8_t PD         : 1; /**< Power down (bit 6, OSR) */
            uint8_t SS         : 1; /**< Start/Stop measurement (bit 7, OSR) */
        } BitField;
    } Val;
} AS7331_OSR_t;

typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; /**< Complete register value as a byte */
        struct {
            uint8_t POWERSTATE   : 1; /**< Power down state (bit 0, STATUS) */
            uint8_t STANDBYSTATE : 1; /**< Standby state (bit 1, STATUS) */
            uint8_t NOTREADY     : 1; /**< Measurement not ready (bit 2, STATUS) */
            uint8_t NDATA        : 1; /**< New data available (bit 3, STATUS) */
            uint8_t LDATA        : 1; /**< Buffer overrun (bit 4, STATUS) */
            uint8_t ADCOF        : 1; /**< ADC overflow (bit 5, STATUS) */
            uint8_t MRESOF       : 1; /**< Measurement result overflow (bit 6, STATUS) */
            uint8_t OUTCONVOF    : 1; /**< Conversion overflow (bit 7, STATUS) */
        } BitField;
    } Val;
} AS7331_STATUS_t;

typedef struct __attribute__((packed)) {
    union {
        struct {
            AS7331_OSR_t OSR;       /**< Operational State Register */
            AS7331_STATUS_t STATUS; /**< Status Register */
        } BitField; /**< Individual register fields */

        uint16_t Value; /**< Complete 2-byte value (LSB first) */
    } Val;
} AS7331_OSR_STATUS_t;

// AGEN Register Structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct {
            uint8_t MUT     : 4; /**< Mutation number of control register bank (bits 0–3) */
            uint8_t DEVID   : 4; /**< Device ID number (bits 4–7) */
        } BitField;
    } Val;
} AS7331_AGEN_t;

// CREG1 Register Enums and Structure
typedef enum {
    GAIN_2048X = 0b0000,
    GAIN_1024X = 0b0001,
    GAIN_512X  = 0b0010,
    GAIN_256X  = 0b0011,
    GAIN_128X  = 0b0100,
    GAIN_64X   = 0b0101,
    GAIN_32X   = 0b0110,
    GAIN_16X   = 0b0111,
    GAIN_8X    = 0b1000,
    GAIN_4X    = 0b1001,
    GAIN_2X    = 0b1010,
    GAIN_1X    = 0b1011
} AS7331_GAIN_t;

typedef enum {
    TIME_1MS    = 0b0000,
    TIME_2MS    = 0b0001,
    TIME_4MS    = 0b0010,
    TIME_8MS    = 0b0011,
    TIME_16MS   = 0b0100,
    TIME_32MS   = 0b0101,
    TIME_64MS   = 0b0110,
    TIME_128MS  = 0b0111,
    TIME_256MS  = 0b1000,
    TIME_512MS  = 0b1001,
    TIME_1024MS = 0b1010,
    TIME_2048MS = 0b1011,
    TIME_4096MS = 0b1100,
    TIME_8192MS = 0b1101,
    TIME_16384MS = 0b1110,
    TIME_1MS_ALT = 0b1111
} AS7331_TIME_t;

// CREG1 Register Structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct {
            uint8_t TIME    : 4; /**< Integration time (bits 0–3) */
            uint8_t GAIN    : 4; /**< Gain settings (bits 4–7) */
        } BitField;
    } Val;
} AS7331_CREG1_t;

// CREG2 Register Enums and Structure
typedef enum { // Only in SYND mode
    TIMER_DISABLED	= 0,
    TIMER_ENABLED		= 1
} AS7331_EN_TM_t;

typedef enum {
    DIVIDER_DISABLED	= 0,
    DIVIDER_ENABLED		= 1
} AS7331_EN_DIV_t;

typedef enum {
    DIVIDER_2   = 0b000,
    DIVIDER_4   = 0b001,
    DIVIDER_8   = 0b010,
    DIVIDER_16  = 0b011,
    DIVIDER_32  = 0b100,
    DIVIDER_64  = 0b101,
    DIVIDER_128 = 0b110,
    DIVIDER_256 = 0b111
} AS7331_DIV_t;

// CREG2 Register Structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct {
            uint8_t DIV        : 3; /**< Digital divider value (bits 0–2) */
            uint8_t EN_DIV     : 1; /**< Enable digital divider (bit 3) */
            uint8_t RESERVED   : 2; /**< Reserved (bits 4–5) */
            uint8_t EN_TM      : 1; /**< Enable internal measurement (bit 6) */
            uint8_t RESERVED1  : 1; /**< Reserved (bit 7) */
        } BitField;
    } Val;
} AS7331_CREG2_t;

// CREG3 Register Enums and Structure
typedef enum {
    MEASURE_MODE_CONT = 0b00,
    MEASURE_MODE_CMD	= 0b01,
    MEASURE_MODE_SYNS	= 0b10,
    MEASURE_MODE_SYND	= 0b11
} AS7331_MMODE_t;

typedef enum {
    STANDBY_OFF = 0,
    STANDBY_ON = 1
} AS7331_SB_t;

typedef enum {
    RDY_PIN_PUSH_PULL = 0,
    RDY_PIN_OPEN_DRAIN = 1
} AS7331_CREG3_RDYOD_t;

typedef enum {
    CLK_1MHZ = 0b00,
    CLK_2MHZ = 0b01,
    CLK_4MHZ = 0b10,
    CLK_8MHZ = 0b11
} AS7331_CLK_t;

// CREG3 Register Structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct {
            uint8_t CCLK       : 2; /**< Internal clock frequency (bits 0–1) */
            uint8_t RESERVED   : 1; /**< Reserved (bit 2) */
            uint8_t RDYOD      : 1; /**< READY pin output mode (bit 3) */
            uint8_t SB         : 1; /**< Standby mode (bit 4) */
            uint8_t RESERVED1  : 1; /**< Reserved (bit 5) */
            uint8_t MMODE      : 2; /**< Measurement mode (bits 6–7) */
        } BitField;
    } Val;
} AS7331_CREG3_t;

// OPTREG Register Structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct {
            uint8_t INIT_IDX : 1; /**< INIT_IDX field (bit 0) */
            uint8_t RESERVED : 7; /**< Reserved (bits 1–7) */
        } BitField;
    } Val;
} AS7331_OPTREG_t;

// Input Data Structure
typedef struct __attribute__((packed)) {
		AS7331_OSR_t OSR;
    AS7331_AGEN_t AGEN;
    AS7331_CREG1_t CREG1;
    AS7331_CREG2_t CREG2;
    AS7331_CREG3_t CREG3;
    uint8_t BREAK; /**< Break time register */
    uint8_t EDGES; /**< Edges configuration register */
    uint8_t OPTREG; /**< Optimization register */
} AS7331_DataIn_t;

// Output Data Structure
typedef struct __attribute__((packed)) {
    int16_t TEMP_C100; /**< Temperature in °C * 100 (16 bits) */
    uint16_t UVA;      /**< UVA (16 bits) */
    uint16_t UVB;      /**< UVB (16 bits) */
    uint16_t UVC;      /**< UVC (16 bits) */
} AS7331_DataOut_t;

// Full Output Result Structure
typedef struct __attribute__((packed)) {
    AS7331_OSR_STATUS_t OSR_STAT; /**< OSR and STATUS register (8 bits) */
    AS7331_DataOut_t DATA;        /**< UV data results */
    uint16_t OUTCONVL;            /**< Time reference, conversion time (LSB) */
    uint16_t OUTCONVH;            /**< Time reference, conversion time (MSB) */
} AS7331_OUT_t;

// --------------------------- Function Prototypes ---------------------------

HAL_StatusTypeDef AS7331_Init(void);
HAL_StatusTypeDef AS7331_GetChipID(uint8_t *chipID);	
// Write 1 byte of data to a register of the AS7331 sensor
static HAL_StatusTypeDef AS7331_WriteRegister(uint8_t reg, uint8_t data);

// Read 2 bytes of data from a register of the AS7331 sensor
static HAL_StatusTypeDef AS7331_ReadRegister(uint8_t reg, uint8_t* data);

// Set the operational state in the OSR register
HAL_StatusTypeDef AS7331_SetOperationalState(AS7331_DOS_t operationalState);

// Perform a software reset by modifying the SW_RES field in the OSR register
HAL_StatusTypeDef AS7331_SoftReset(AS7331_SW_RST_t swReset);

// Set the power state in the OSR register
HAL_StatusTypeDef AS7331_SetPower(AS7331_PWR_t setPower);

// Start or stop measurement by modifying the SS field in the OSR register
HAL_StatusTypeDef AS7331_StartStopMeasurement(AS7331_SS_t startStopMeasure);

// Set the integration time in the CREG1 register
HAL_StatusTypeDef AS7331_SetIntegrationTime(AS7331_TIME_t time);

// Set the gain in the CREG1 register
HAL_StatusTypeDef AS7331_SetGain(AS7331_GAIN_t gain);

// Set the digital divider value in the CREG2 register
HAL_StatusTypeDef AS7331_SetDivider(AS7331_DIV_t divider);

// Enable or disable the digital divider in the CREG2 register
HAL_StatusTypeDef AS7331_EnableDivider(AS7331_EN_DIV_t enableDivider);

// Enable or disable internal measurement of the conversion time in the CREG2 register
HAL_StatusTypeDef AS7331_EnableInternalMeasurement(AS7331_EN_TM_t enableTimer);

// Set the measurement mode in the CREG3 register
HAL_StatusTypeDef AS7331_SetMeasurementMode(AS7331_MMODE_t mode);

// Set the standby mode in the CREG3 register
HAL_StatusTypeDef AS7331_SetStandbyMode(AS7331_SB_t standby);

// Set the READY pin output mode in the CREG3 register
HAL_StatusTypeDef AS7331_SetReadyOutputMode(AS7331_CREG3_RDYOD_t rdyod);

// Set the internal clock frequency in the CREG3 register
HAL_StatusTypeDef AS7331_SetClockFrequency(AS7331_CLK_t cclk);

// Set the break time (TBREAK) between two measurements in the BREAK register
HAL_StatusTypeDef AS7331_SetBreakTime(uint8_t breakTime);

// Set the number of SYN falling edges in the EDGES register
HAL_StatusTypeDef AS7331_SetEdges(uint8_t edges);

// Set the OPTREG:INIT_IDX field in the OPTREG register
HAL_StatusTypeDef AS7331_SetOptReg(uint8_t initIdx);

// Read the Operational State Register (OSR) and Status Register values
HAL_StatusTypeDef AS7331_GetOSR_Status(void);

// Get the temperature measurement result
HAL_StatusTypeDef AS7331_GetTemperature(uint16_t *result);

// Read the measurement result for Channel A
HAL_StatusTypeDef AS7331_getChannelA(uint16_t *result);

// Read the measurement result for Channel B
HAL_StatusTypeDef AS7331_getChannelB(uint16_t *result);

// Read the measurement result for Channel C
HAL_StatusTypeDef AS7331_getChannelC(uint16_t *result);

// Read the conversion time measurement (OUTCONVL and OUTCONVH)
HAL_StatusTypeDef AS7331_ReadConversionTime(uint32_t *conversionTime);

HAL_StatusTypeDef AS7331_ReadUVData(AS7331_DataOut_t *uvData);

#endif // AS7331_H

#ifndef AS7331_H
#define AS7331_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

// AS7331 I2C Address
#define AS7331_I2C_ADDR 0x74 << 1 // ADDR 1, 2 LOW

#define AS7331_REG_OSR			0x00
//#define AS7331_REG_TEMP			0x01
#define AS7331_REG_AGEN			0x02
//#define AS7331_REG_MRES2		0x03
//#define AS7331_REG_MRES3		0x04
//#define AS7331_REG_OUTCONV_L	0x05 // ONLY in SYND
#define AS7331_REG_CREG1		0x06
#define AS7331_REG_CREG2		0x07
#define AS7331_REG_CREG3		0x08
#define AS7331_REG_BREAK		0x09
#define AS7331_REG_EDGES		0x0A
#define AS7331_REG_OPTREG		0x0B

// OSR Register Enums and Structure
typedef enum {
    OSR_SS_STOP		= 0,
    OSR_SS_START	= 1
} AS7331_OSR_SS_t;

typedef enum {
    OSR_PD_OFF	= 0,
    OSR_PD_ON	= 1
} AS7331_OSR_PD_t;

typedef enum {
    OSR_SW_RES_INACTIVE = 0,
    OSR_SW_RES_ACTIVE	= 1
} AS7331_OSR_SW_RES_t;

typedef enum {
    OSR_DOS_NOP				= 0b000,
    OSR_DOS_CONFIGURATION	= 0b010,
    OSR_DOS_MEASUREMENT		= 0b011,
    OSR_DOS_NOP_ALT			= 0b100
} AS7331_OSR_DOS_t;

typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct {
            AS7331_OSR_DOS_t DOS		: 3;
            AS7331_OSR_SW_RES_t SW_RES	: 1;
            uint8_t RESERVED			: 2;
            AS7331_OSR_PD_t PD			: 1;
            AS7331_OSR_SS_t SS			: 1;
        } BitField;
    } Val;
} AS7331_OSR_t;

// AGEN Register Structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct {
            uint8_t MUT : 4;
            uint8_t DEVID : 4;
        } BitField;
    } Val;
} AS7331_AGEN_t;

// CREG1 Register Enums and Structure
typedef enum {
    CREG1_GAIN_2048X = 0b0000,
    CREG1_GAIN_1024X = 0b0001,
    CREG1_GAIN_512X  = 0b0010,
    CREG1_GAIN_256X  = 0b0011,
    CREG1_GAIN_128X  = 0b0100,
    CREG1_GAIN_64X   = 0b0101,
    CREG1_GAIN_32X   = 0b0110,
    CREG1_GAIN_16X   = 0b0111,
    CREG1_GAIN_8X    = 0b1000,
    CREG1_GAIN_4X    = 0b1001,
    CREG1_GAIN_2X    = 0b1010,
    CREG1_GAIN_1X    = 0b1011
} AS7331_CREG1_GAIN_t;

typedef enum {
    CREG1_TIME_1MS    = 0b0000,
    CREG1_TIME_2MS    = 0b0001,
    CREG1_TIME_4MS    = 0b0010,
    CREG1_TIME_8MS    = 0b0011,
    CREG1_TIME_16MS   = 0b0100,
    CREG1_TIME_32MS   = 0b0101,
    CREG1_TIME_64MS   = 0b0110,
    CREG1_TIME_128MS  = 0b0111,
    CREG1_TIME_256MS  = 0b1000,
    CREG1_TIME_512MS  = 0b1001,
    CREG1_TIME_1024MS = 0b1010,
    CREG1_TIME_2048MS = 0b1011,
    CREG1_TIME_4096MS = 0b1100,
    CREG1_TIME_8192MS = 0b1101,
    CREG1_TIME_16384MS = 0b1110,
    CREG1_TIME_1MS_ALT = 0b1111
} AS7331_CREG1_TIME_t;

typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct {
            AS7331_CREG1_TIME_t TIME : 4;
            AS7331_CREG1_GAIN_t GAIN : 4;
        } BitField;
    } Val;
} AS7331_CREG1_t;

// CREG2 Register Enums and Structure
typedef enum {
    CREG2_EN_TM_DISABLED = 0,
    CREG2_EN_TM_ENABLED = 1
} AS7331_CREG2_EN_TM_t;

typedef enum {
    CREG2_EN_DIV_DISABLED = 0,
    CREG2_EN_DIV_ENABLED = 1
} AS7331_CREG2_EN_DIV_t;

typedef enum {
    CREG2_DIV_2   = 0b000,
    CREG2_DIV_4   = 0b001,
    CREG2_DIV_8   = 0b010,
    CREG2_DIV_16  = 0b011,
    CREG2_DIV_32  = 0b100,
    CREG2_DIV_64  = 0b101,
    CREG2_DIV_128 = 0b110,
    CREG2_DIV_256 = 0b111
} AS7331_CREG2_DIV_t;

typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct {
            AS7331_CREG2_DIV_t DIV			: 3;
            AS7331_CREG2_EN_DIV_t EN_DIV	: 1;
            uint8_t RESERVED				: 2;
            AS7331_CREG2_EN_TM_t EN_TM		: 1;
            uint8_t RESERVED1				: 1;
        } BitField;
    } Val;
} AS7331_CREG2_t;

// CREG3 Register Enums and Structure
typedef enum {
    CREG3_MMODE_CONT = 0b00,
    CREG3_MMODE_CMD = 0b01,
    CREG3_MMODE_SYNS = 0b10,
    CREG3_MMODE_SYND = 0b11
} AS7331_CREG3_MMODE_t;

typedef enum {
    CREG3_SB_OFF = 0,
    CREG3_SB_ON = 1
} AS7331_CREG3_SB_t;

typedef enum {
    CREG3_RDYOD_PUSH_PULL = 0,
    CREG3_RDYOD_OPEN_DRAIN = 1
} AS7331_CREG3_RDYOD_t;

typedef enum {
    CREG3_CCLK_1MHZ = 0b00,
    CREG3_CCLK_2MHZ = 0b01,
    CREG3_CCLK_4MHZ = 0b10,
    CREG3_CCLK_8MHZ = 0b11
} AS7331_CREG3_CCLK_t;

typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct {
            AS7331_CREG3_CCLK_t CCLK	: 2;
            uint8_t RESERVED			: 1;
            AS7331_CREG3_RDYOD_t RDYOD	: 1;
            AS7331_CREG3_SB_t SB		: 1;
            uint8_t RESERVED1			: 1;
            AS7331_CREG3_MMODE_t MMODE	: 2;
        } BitField;
    } Val;
} AS7331_CREG3_t;

// OPTREG Register Enums and Structure
typedef enum {
    OPTREG_INIT_IDX_SET_ON_STOP = 0,
    OPTREG_INIT_IDX_RESET_ON_STOP = 1
} AS7331_OPTREG_INIT_IDX_t;

typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct {
            AS7331_OPTREG_INIT_IDX_t INIT_IDX : 1;
            uint8_t RESERVED : 7;
        } BitField;
    } Val;
} AS7331_OPTREG_t;

// STATUS Register Structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct {
            uint8_t POWERSTATE		: 1;
            uint8_t STANDBYSTATE	: 1;
            uint8_t NOTREADY		: 1;
            uint8_t NDATA			: 1;
            uint8_t LDATA			: 1;
            uint8_t ADCOF			: 1;
            uint8_t MRESOF			: 1;
            uint8_t OUTCONVOF 		: 1;
        } BitField;
    } Val;
} AS7331_STATUS_t;

// Structure for Output Result Register Bank
typedef struct __attribute__((packed)) {
    AS7331_OSR_t OSR;             // Operational State Register (8 bits)
    AS7331_STATUS_t STATUS;       // Status Register (8 bits)
    uint16_t TEMP;                // Temperature Measurement Result (16 bits, 12 bits for the value)
    uint16_t MRES1;               // Measurement Result A-Channel (16 bits)
    uint16_t MRES2;               // Measurement Result B-Channel (16 bits)
    uint16_t MRES3;               // Measurement Result C-Channel (16 bits)
    uint16_t OUTCONVL;            // Time reference, conversion time measurement (16 bits, LSB + middle byte)
    uint16_t OUTCONVH;            // Time reference, conversion time measurement (16 bits, MSB + empty byte)
} AS7331_OutputResultRegisterBank_t;

// Structure for all registers
typedef struct __attribute__((packed)) {
    AS7331_OSR_t OSR;
    uint8_t STATUS;
    uint16_t TEMP;
    uint16_t MRES1;
    uint16_t MRES2;
    uint16_t MRES3;
    uint16_t OUTCONVL;
    uint16_t OUTCONVH;
    AS7331_AGEN_t AGEN;
    AS7331_CREG1_t CREG1;
    AS7331_CREG2_t CREG2;
    AS7331_CREG3_t CREG3;
    uint8_t BREAK;
    uint8_t EDGES;
    AS7331_OPTREG_t OPTREG;
} AS7331_Registers_t;

// Structure to hold UV data
typedef struct __attribute__((packed)) {
    uint16_t UVA;
    uint16_t UVB;
    uint16_t UVC;
} AS7331_UVData_t;

HAL_StatusTypeDef AS7331_Init(void);
HAL_StatusTypeDef AS7331_ReadUVData(AS7331_UVData_t *uvData);
HAL_StatusTypeDef AS7331_WriteRegister(uint8_t regAddr, uint8_t value);
HAL_StatusTypeDef AS7331_ReadRegister(uint8_t regAddr, uint8_t *value);
HAL_StatusTypeDef AS7331_ReadRegister16(uint8_t regAddr, uint16_t *value);
HAL_StatusTypeDef AS7331_UpdateOutputResultRegisters(AS7331_OutputResultRegisterBank_t *outputRegisters);
HAL_StatusTypeDef AS7331_SetIntegrationTime(AS7331_CREG1_TIME_t time);
HAL_StatusTypeDef AS7331_SetGain(AS7331_CREG1_GAIN_t gain);
HAL_StatusTypeDef AS7331_ReadOSRStatus(uint8_t *osr, uint8_t *status);
HAL_StatusTypeDef AS7331_WriteRegisters(void);

#endif // AS7331_H

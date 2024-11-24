#ifndef TCS34717_H
#define TCS34717_H

#include "stm32f3xx_hal.h"

// TCS34717 register addresses
#define TCS347171_ENABLE_REG  0x00
#define TCS347171_ATIME_REG   0x01
#define TCS347171_WTIME_REG   0x03
#define TCS347171_AILTL_REG   0x04
#define TCS347171_AILTH_REG   0x05
#define TCS347171_AIHTL_REG   0x06
#define TCS347171_AIHTH_REG   0x07
#define TCS347171_PERS_REG    0x0C
#define TCS347171_CONFIG_REG  0x0D
#define TCS347171_CONTROL_REG 0x0F
#define TCS347171_ID_REG      0x12
#define TCS347171_STATUS_REG  0x13
#define TCS347171_CDATA_REG   0x14
#define TCS347171_CDATAH_REG  0x15
#define TCS347171_RDATA_REG   0x16
#define TCS347171_RDATAH_REG  0x17
#define TCS347171_GDATA_REG   0x18
#define TCS347171_GDATAH_REG  0x19
#define TCS347171_BDATA_REG   0x1A
#define TCS347171_BDATAH_REG  0x1B

#define TCS34717_I2C_ADDRESS	0x29 << 1  // Replace 0x29 with the actual 7-bit address of your sensor
#define TCS3471_COMMAND_BIT		0x80  // Bit 7 set to 1 to indicate a command

// COMMAND register structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t ADD   : 5; // Bits 4:0: Register address or special function
            uint8_t TYPE  : 2; // Bits 6:5: Transaction type (00: byte, 01: auto-increment, 11: special function)
            uint8_t CMD   : 1; // Bit 7: Must be set to 1 when addressing the COMMAND register
        } BitField;
    } Val;
} TCS34717_COMMAND_t;

// ENABLE register structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t POWER_ON : 1; // Bit 0: Power ON
            uint8_t RGBC_EN  : 1; // Bit 1: RGBC enable
            uint8_t RESERVED0 : 1; // Bit 2: Reserved
            uint8_t WAIT_EN  : 1; // Bit 3: Wait enable
            uint8_t INT_EN   : 1; // Bit 4: RGBC interrupt enable
            uint8_t RESERVED1 : 3; // Bits 5-7: Reserved
        } BitField;
    } Val;
} TCS34717_ENABLE_t;

// Enum for RGBC integration time (ATIME)
typedef enum {
    TCS34717_ATIME_2_4ms   = 0xFF,
    TCS34717_ATIME_24ms    = 0xF6,
    TCS34717_ATIME_101ms   = 0xD5,
    TCS34717_ATIME_154ms   = 0xC0,
    TCS34717_ATIME_700ms   = 0x00
} TCS34717_ATIME_t;

// Enum for wait time (WTIME)
typedef enum {
    TCS34717_WTIME_2_4ms   = 0xFF,
    TCS34717_WTIME_204ms   = 0xAB,
    TCS34717_WTIME_614ms   = 0x00
} TCS34717_WTIME_t;

// RGBC interrupt threshold structure
typedef struct __attribute__((packed)) {
    union {
        struct {
            uint16_t AIL; // 0x04-0x05: RGBC clear channel low threshold
            uint16_t AIH; // 0x06-0x07: RGBC clear channel high threshold
        } Combined;
        struct {
            uint8_t AILTL; // 0x04: Low threshold low byte
            uint8_t AILTH; // 0x05: Low threshold high byte
            uint8_t AIHTL; // 0x06: High threshold low byte
            uint8_t AIHTH; // 0x07: High threshold high byte
        } ByteField;
    } Val;
} TCS34717_RGBC_Interrupt_Threshold_t;

// Enum for persistence register (PERS)
typedef enum {
    APERS_EVERY_RGBC_CYCLE = 0x0,
    APERS_1_CLEAR_OUT_OF_RANGE = 0x1,
    APERS_2_CONSECUTIVE_OUT_OF_RANGE = 0x2,
    APERS_3_CONSECUTIVE_OUT_OF_RANGE = 0x3,
    APERS_5_CONSECUTIVE_OUT_OF_RANGE = 0x4,
    APERS_10_CONSECUTIVE_OUT_OF_RANGE = 0x5,
    APERS_15_CONSECUTIVE_OUT_OF_RANGE = 0x6,
    APERS_20_CONSECUTIVE_OUT_OF_RANGE = 0x7,
    APERS_25_CONSECUTIVE_OUT_OF_RANGE = 0x8,
    APERS_30_CONSECUTIVE_OUT_OF_RANGE = 0x9,
    APERS_35_CONSECUTIVE_OUT_OF_RANGE = 0xA,
    APERS_40_CONSECUTIVE_OUT_OF_RANGE = 0xB,
    APERS_45_CONSECUTIVE_OUT_OF_RANGE = 0xC,
    APERS_50_CONSECUTIVE_OUT_OF_RANGE = 0xD,
    APERS_55_CONSECUTIVE_OUT_OF_RANGE = 0xE,
    APERS_60_CONSECUTIVE_OUT_OF_RANGE = 0xF
} TCS34717_APERS_t;

// Enum for configuration register (CONFIG)
typedef enum {
    TCS34717_WLONG_DISABLE = 0x00,
    TCS34717_WLONG_ENABLE = 0x02
} TCS34717_WLONG_t;

// Enum for control register (CONTROL)
typedef enum {
    TCS34717_AGAIN_1X = 0x00,
    TCS34717_AGAIN_4X = 0x01,
    TCS34717_AGAIN_16X = 0x02,
    TCS34717_AGAIN_60X = 0x03
} TCS34717_AGAIN_t;

// STATUS register structure
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t AVALID   : 1; // Bit 0: RGBC Valid
            uint8_t RESERVED0 : 1; // Bit 1: Reserved
            uint8_t AINT     : 1; // Bit 4: RGBC clear channel Interrupt
            uint8_t RESERVED1 : 5; // Bits 3, 5-7: Reserved
        } BitField;
    } Val;
} TCS34717_STATUS_t;

// RGBC channel data structure
typedef struct __attribute__((packed)) {
	uint16_t CDATA; // 0x14-0x15: Clear data
	uint16_t RDATA; // 0x16-0x17: Red data
	uint16_t GDATA; // 0x18-0x19: Green data
	uint16_t BDATA; // 0x1A-0x1B: Blue data
} TCS34717_CRGB_t;

// Function prototypes
uint8_t TCS34717_Read(uint8_t addr, uint8_t *buffer, uint16_t num_bytes);
uint8_t TCS34717_Write(uint8_t addr, uint8_t *buffer, uint16_t num_bytes);
uint8_t TCS34717_SetBits(uint8_t addr, uint8_t bits);
uint8_t TCS34717_ClearBits(uint8_t addr, uint8_t bits);
HAL_StatusTypeDef TCS34717_ReadRGBCData(TCS34717_CRGB_t *rgbcData);
void TCS34717_Enable(void);
void TCS34717_Disable(void);
void TCS34717_SetGain(TCS34717_AGAIN_t gain);
void TCS34717_SetIntegrationTime(uint8_t time);
void TCS34717_SetWaitTime(uint8_t wait_time);
uint8_t TCS34717_IsDataValid(void);
void TCS34717_SetPersistence(TCS34717_APERS_t persistence);

#endif // TCS34717_H

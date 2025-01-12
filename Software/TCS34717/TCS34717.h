#ifndef TCS34717_H
#define TCS34717_H

#include "stm32f3xx_hal.h"

/** @brief I2C Address */
#define TCS34717_I2C_ADDRESS   (0x29 << 1) ///< 7-bit I2C address shifted for 8-bit

/** @brief Command and Protocol Macros */
#define TCS3471_AUTOINCR_BIT   (0x01 << 5) ///< TYPE = 01: Auto-increment protocol
#define TCS3471_SPECIAL_BIT    (0x03 << 5) ///< TYPE = 11: Special function
#define TCS3471_INTCLEAR_BIT   0x06        ///< ADD = 0x06: RGBC Interrupt Clear

/** @brief Macro to construct the GET_ALL command for reading multiple registers */
#define TCS3471_GET_ALL (0x14 | TCS3471_AUTOINCR_BIT) ///< Command to read CRGB registers with auto-increment

/** @brief Macro to construct the INT_CLR command for clearing interrupts */
#define TCS3471_INT_CLR (TCS3471_SPECIAL_BIT | TCS3471_INTCLEAR_BIT) ///< Command to clear RGBC interrupt

/** @brief TCS34717 Register Addresses */
#define TCS34717_ENABLE_REG   (0x00 | 0x80)
#define TCS34717_ATIME_REG    (0x01 | 0x80)
#define TCS34717_WTIME_REG    (0x03 | 0x80)
#define TCS34717_AILTL_REG    (0x04 | 0x80)
#define TCS34717_AILTH_REG    0x05
#define TCS34717_AIHTL_REG    (0x06 | 0x80)
#define TCS34717_AIHTH_REG    0x07
#define TCS34717_PERS_REG     (0x0C | 0x80)
#define TCS34717_CONFIG_REG   0x0D
#define TCS34717_CONTROL_REG  (0x0F | 0x80)
#define TCS34717_ID_REG       (0x12 | 0x80)
#define TCS34717_STATUS_REG   0x13
#define TCS34717_CDATA_REG    0x14
#define TCS34717_RDATA_REG    0x16
#define TCS34717_GDATA_REG    0x18
#define TCS34717_BDATA_REG    0x1A

#define TCS34717_MID_VAL		0x7FFF
/** @brief COMMAND register structure */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t ADD   : 5; // Bits 4:0: Register address or special function
            uint8_t TYPE  : 2; // Bits 6:5: Transaction type
            uint8_t CMD   : 1; // Bit 7: Must be set to 1 when addressing the COMMAND register
        } BitField;
    } Val;
} TCS34717_COMMAND_t;

/** @brief ENABLE register structure */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t POWER_ON	: 1; // Bit 0: Power ON
            uint8_t RGBC_EN		: 1; // Bit 1: RGBC enable
            uint8_t RESERVED0	: 1; // Bit 2: Reserved
            uint8_t WAIT_EN		: 1; // Bit 3: Wait enable
            uint8_t INT_EN		: 1; // Bit 4: Interrupt enable
            uint8_t RESERVED1	: 3; // Bits 5-7: Reserved
        } BitField;
    } Val;
} TCS34717_ENABLE_t;

/** @brief Enum for RGBC integration time (ATIME) */
typedef enum {
    TCS34717_ATIME_2_4ms   = 0xFF,
    TCS34717_ATIME_24ms    = 0xF6,
    TCS34717_ATIME_101ms   = 0xD5,
    TCS34717_ATIME_154ms   = 0xC0,
    TCS34717_ATIME_700ms   = 0x00
} TCS34717_ATIME_t;

/** @brief Enum for wait time (WTIME) */
typedef enum {
    TCS34717_WTIME_2_4ms   = 0xFF,
    TCS34717_WTIME_204ms   = 0xAB,
    TCS34717_WTIME_614ms   = 0x00
} TCS34717_WTIME_t;

/** @brief RGBC interrupt threshold structure */
typedef struct __attribute__((packed)) {
    union {
        struct {
            uint16_t AIL; ///< Low threshold (16-bit)
            uint16_t AIH; ///< High threshold (16-bit)
        };
        uint8_t ByteArray[4]; ///< Thresholds as byte array
    };
} TCS34717_Int_Val_t;

/** @brief Enum for persistence register (PERS) */
typedef enum {
    APERS_EVERY_RGBC_CYCLE				= 0x0,
    APERS_1_CLEAR_OUT_OF_RANGE			= 0x1,
    APERS_2_CONSECUTIVE_OUT_OF_RANGE	= 0x2,
    APERS_3_CONSECUTIVE_OUT_OF_RANGE	= 0x3,
    APERS_5_CONSECUTIVE_OUT_OF_RANGE	= 0x4,
    APERS_10_CONSECUTIVE_OUT_OF_RANGE	= 0x5,
    APERS_15_CONSECUTIVE_OUT_OF_RANGE	= 0x6,
    APERS_20_CONSECUTIVE_OUT_OF_RANGE	= 0x7,
    APERS_25_CONSECUTIVE_OUT_OF_RANGE	= 0x8,
    APERS_30_CONSECUTIVE_OUT_OF_RANGE	= 0x9,
    APERS_35_CONSECUTIVE_OUT_OF_RANGE	= 0xA,
    APERS_40_CONSECUTIVE_OUT_OF_RANGE	= 0xB,
    APERS_45_CONSECUTIVE_OUT_OF_RANGE	= 0xC,
    APERS_50_CONSECUTIVE_OUT_OF_RANGE	= 0xD,
    APERS_55_CONSECUTIVE_OUT_OF_RANGE	= 0xE,
    APERS_60_CONSECUTIVE_OUT_OF_RANGE	= 0xF
} TCS34717_PERS_t;

// Enum for configuration register (CONFIG)
typedef enum {
    TCS34717_WLONG_DISABLE	= 0x00,
    TCS34717_WLONG_ENABLE	= 0x02
} TCS34717_WLONG_t;

// Enum for control register (CONTROL)
typedef enum {
    TCS34717_AGAIN_1X	= 0x00,
    TCS34717_AGAIN_4X	= 0x01,
    TCS34717_AGAIN_16X	= 0x02,
    TCS34717_AGAIN_60X	= 0x03
} TCS34717_AGAIN_t;

/** @brief STATUS register structure */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t AVALID		: 1; // Bit 0: RGBC Valid
            uint8_t RESERVED0	: 1; // Bit 1: Reserved
            uint8_t AINT		: 1; // Bit 4: RGBC interrupt
            uint8_t RESERVED1	: 5; // Bits 3, 5-7: Reserved
        } BitField;
    } Val;
} TCS34717_STATUS_t;

/** @brief CRGB data structure */
typedef struct __attribute__((packed)) {
    union {
        struct {
            uint16_t CDATA; ///< Clear channel data (16-bit)
            uint16_t RDATA; ///< Red channel data (16-bit)
            uint16_t GDATA; ///< Green channel data (16-bit)
            uint16_t BDATA; ///< Blue channel data (16-bit)
        };
        uint8_t ByteArray[8]; ///< RGBC data as a byte array for I2C transactions
    };
} TCS34717_CRGB_t;

/**
 * @brief Structure to represent the TCS34717 registers.
 */
typedef struct __attribute__((packed)) {
    TCS34717_ENABLE_t ENABLE;	///< Enable register (0x00)
    TCS34717_ATIME_t ATIME;		///< Integration time register (0x01)
    TCS34717_WTIME_t WTIME;		///< Wait time register (0x03)
    TCS34717_Int_Val_t INT_VAL;	///< Low/High interrupt threshold (0x04-0x07)
    TCS34717_PERS_t PERS;		///< Persistence register (0x0C)
    TCS34717_WLONG_t CONFIG;	///< Configuration register (0x0D)
    TCS34717_AGAIN_t CONTROL;	///< Control register (0x0F)
    uint8_t ID;					///< ID register (0x12)
    TCS34717_STATUS_t STATUS;	///< Status register (0x13)
    //TCS34717_CRGB_t RGBC;		///< RGBC data registers (0x14-0x1B)
} TCS34717_Registers_t;

HAL_StatusTypeDef TCS34717_Init(void);
HAL_StatusTypeDef TCS34717_GetID(void);
HAL_StatusTypeDef TCS34717_PowerOn(void);
HAL_StatusTypeDef TCS34717_PowerOff(void);
HAL_StatusTypeDef TCS34717_Enable(void);
HAL_StatusTypeDef TCS34717_Disable(void);
HAL_StatusTypeDef TCS34717_SetIntegrationTime(TCS34717_ATIME_t integration_time_ms);
HAL_StatusTypeDef TCS34717_SetWaitTime(TCS34717_WTIME_t wait_time_ms);
HAL_StatusTypeDef TCS34717_SetGain(TCS34717_AGAIN_t gain);
HAL_StatusTypeDef TCS34717_EnableInterrupt(void);
HAL_StatusTypeDef TCS34717_DisableInterrupt(void);
HAL_StatusTypeDef TCS34717_ClearInt(void);
HAL_StatusTypeDef TCS34717_SetInterruptLow(uint16_t low_threshold);
HAL_StatusTypeDef TCS34717_SetInterruptHigh(uint16_t high_threshold);
HAL_StatusTypeDef TCS34717_Persistance(TCS34717_PERS_t persistence);
HAL_StatusTypeDef TCS34717_GetCRGB(TCS34717_CRGB_t *crgb);
HAL_StatusTypeDef TCS34717_ReadRegisters(void);
#endif // TCS34717_H

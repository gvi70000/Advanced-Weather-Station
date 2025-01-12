#ifndef TCS3471_H
#define TCS3471_H

#include "stm32f3xx_hal.h"

/** @brief I2C Address */
#define TCS3471_I2C_ADDRESS   (0x29 << 1) ///< 7-bit I2C address shifted for 8-bit

/** @brief TCS3471 Register Addresses */
#define TCS3471_ENABLE_REG   (0x00 | 0x80)
#define TCS3471_ATIME_REG    (0x01 | 0x80)
#define TCS3471_WTIME_REG    (0x03 | 0x80)
#define TCS3471_AILTL_REG    (0x04 | 0x80)
#define TCS3471_AILTH_REG    0x05
#define TCS3471_AIHTL_REG    (0x06 | 0x80)
#define TCS3471_AIHTH_REG    0x07
#define TCS3471_PERS_REG     (0x0C | 0x80)
#define TCS3471_CONFIG_REG   (0x0D | 0x80)
#define TCS3471_CONTROL_REG  (0x0F | 0x80)
#define TCS3471_ID_REG       (0x12 | 0x80)
#define TCS3471_STATUS_REG   0x13
#define TCS3471_CDATA_REG    0x14
#define TCS3471_RDATA_REG    0x16
#define TCS3471_GDATA_REG    0x18
#define TCS3471_BDATA_REG    0x1A

#define TCS3471_MID_VAL		0x7FFF

typedef enum {
    COMMAND_NORMAL						= 0x80, // Normal — no action
    COMMAND_AUTO_INCREMENT		= 0xA0, // Auto-increment protocol transaction
    COMMAND_GET_ALL						= 0xB4, // Command to read CRGB registers with auto-increment
    COMMAND_SPECIAL_FUNCTION	= 0xE0, // Special function command
    COMMAND_INTERRUPT_CLEAR		= 0xE6  // RGBC Interrupt Clear
} TCS3471_Command_t;

/** @brief COMMAND register structure 
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t ADD   : 5; // Bits 4:0: Register address or special function
            uint8_t TYPE  : 2; // Bits 6:5: Transaction type
            uint8_t CMD   : 1; // Bit 7: Must be set to 1 when addressing the COMMAND register
        } BitField;
    } Val;
} TCS3471_COMMAND_t; */

typedef enum {
    ENABLE_NONE          		= 0x00, ///< All bits disabled
    ENABLE_POWER_ON      		= 0x01, ///< POWER_ON enabled
    ENABLE_RGBC_EN       		= 0x02, ///< RGBC_EN enabled
    ENABLE_WAIT_EN       		= 0x08, ///< WAIT_EN enabled
    ENABLE_INT_EN        		= 0x10, ///< INT_EN enabled
    ENABLE_POWER_RGBC    		= 0x03, ///< POWER_ON and RGBC_EN enabled
    ENABLE_POWER_WAIT    		= 0x09, ///< POWER_ON and WAIT_EN enabled
    ENABLE_POWER_INT     		= 0x11, ///< POWER_ON and INT_EN enabled
    ENABLE_RGBC_WAIT     		= 0x0A, ///< RGBC_EN and WAIT_EN enabled
    ENABLE_RGBC_INT      		= 0x12, ///< RGBC_EN and INT_EN enabled
    ENABLE_WAIT_INT      		= 0x18, ///< WAIT_EN and INT_EN enabled
    ENABLE_POWER_RGBC_WAIT	= 0x0B, ///< POWER_ON, RGBC_EN, and WAIT_EN enabled
    ENABLE_POWER_RGBC_INT		= 0x13, ///< POWER_ON, RGBC_EN, and INT_EN enabled
    ENABLE_POWER_WAIT_INT		= 0x19, ///< POWER_ON, WAIT_EN, and INT_EN enabled
    ENABLE_RGBC_WAIT_INT		= 0x1A, ///< RGBC_EN, WAIT_EN, and INT_EN enabled
    ENABLE_ALL				= 0x1B  ///< POWER_ON, RGBC_EN, WAIT_EN, and INT_EN enabled
} TCS3471_EnableValues_t;

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
} TCS3471_ENABLE_t;

/** @brief Enum for RGBC integration time (ATIME) */
typedef enum {
    TCS3471_ATIME_2_4ms   = 0xFF,
    TCS3471_ATIME_24ms    = 0xF6,
    TCS3471_ATIME_101ms   = 0xD5,
    TCS3471_ATIME_154ms   = 0xC0,
    TCS3471_ATIME_700ms   = 0x00
} TCS3471_ATIME_t;

/** @brief Enum for wait time (WTIME) */
typedef enum {
    WTIME_2_4MS = 0xFF, // Wait time = 2.4 ms (WLONG = 0), 0.029 s (WLONG = 1)
    WTIME_204MS = 0xAB, // Wait time = 204 ms (WLONG = 0), 2.45 s (WLONG = 1)
    WTIME_614MS = 0x00  // Wait time = 614 ms (WLONG = 0), 7.4 s (WLONG = 1)
} TCS3471_WTIME_t;

/** @brief RGBC interrupt threshold structure */
typedef struct __attribute__((packed)) {
    union {
        struct {
            uint16_t AIL; ///< Low threshold (16-bit)
            uint16_t AIH; ///< High threshold (16-bit)
        };
        uint8_t ByteArray[4]; ///< Thresholds as byte array
    };
} TCS3471_THR_t;

/** @brief Enum for persistence register (PERS) */
typedef enum {
    APERS_EVERY_RGBC_CYCLE						= 0x0,
    APERS_1_CLEAR_OUT_OF_RANGE				= 0x1,
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
} TCS3471_PERS_t;

// Enum for configuration register (CONFIG)
typedef enum {
    TCS3471_WLONG_DISABLE	= 0x00,
    TCS3471_WLONG_ENABLE	= 0x02
} TCS3471_WLONG_t;

// Enum for control register (CONTROL)
typedef enum {
    TCS3471_AGAIN_1X	= 0x00,
    TCS3471_AGAIN_4X	= 0x01,
    TCS3471_AGAIN_16X	= 0x02,
    TCS3471_AGAIN_60X	= 0x03
} TCS3471_AGAIN_t;

/** @brief STATUS register structure */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t AVALID		: 1; // Bit 0: RGBC Valid
            uint8_t RESERVED0	: 3; // Bit 3:1: Reserved
            uint8_t AINT			: 1; // Bit 4: RGBC interrupt
            uint8_t RESERVED1	: 3; // Bit 7:5: Reserved
        } BitField;
    } Val;
} TCS3471_STATUS_t;

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
    float Irradiance; ///< Irradiance value in W/m², not linked to ByteArray
} TCS3471_CRGB_t;

/**
 * @brief Structure to represent the TCS3471 registers.
 */
typedef struct __attribute__((packed)) {
    TCS3471_ENABLE_t ENABLE;	///< Enable register (0x00)
    TCS3471_ATIME_t ATIME;		///< Integration time register (0x01)
    TCS3471_WTIME_t WTIME;		///< Wait time register (0x03)
    TCS3471_THR_t INT_VAL;		///< Low/High interrupt threshold (0x04-0x07)
    TCS3471_PERS_t PERS;			///< Persistence register (0x0C)
    TCS3471_WLONG_t WLONG;		///< Configuration register (0x0D)
    TCS3471_AGAIN_t GAIN;			///< Control register (0x0F)
    //uint8_t ID;							///< ID register (0x12)
    TCS3471_STATUS_t STATUS;	///< Status register (0x13)
    //TCS3471_CRGB_t RGBC;		///< RGBC data registers (0x14-0x1B)
} TCS3471_Registers_t;

HAL_StatusTypeDef TCS3471_Init(void);
HAL_StatusTypeDef TCS3471_GetID(uint8_t *id);
HAL_StatusTypeDef TCS3471_Enable(TCS3471_EnableValues_t regVal);
HAL_StatusTypeDef TCS3471_SetIntegrationTime(TCS3471_ATIME_t integration_time_ms);
HAL_StatusTypeDef TCS3471_SetWaitTime(TCS3471_WTIME_t wait_time_ms, TCS3471_WLONG_t multiply_12x);
HAL_StatusTypeDef TCS3471_ClearInt(void);
HAL_StatusTypeDef TCS3471_SetInterruptThreshold(uint16_t low_threshold, uint16_t high_threshold);
HAL_StatusTypeDef TCS3471_Persistance(TCS3471_PERS_t persistence);
HAL_StatusTypeDef TCS3471_SetGain(TCS3471_AGAIN_t gain);
HAL_StatusTypeDef TCS3471_GetStatus(void);
HAL_StatusTypeDef TCS3471_GetCRGB(TCS3471_CRGB_t *crgb);
#endif // TCS3471_H

#ifndef TSL25911_H
#define TSL25911_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

/** @brief TSL2591 I2C Address (default). */
#define TSL25911_I2C_ADDR (0x29 << 1) // Adjust based on hardware configuration

/** @brief Device ID for TSL2591 sensor. */
#define TSL25911_ID 0x50

/** @brief Midpoint value for ALS thresholds. */
#define TSL25911_SET_MID 0x7FFF

/** @brief Command bit for transactions. */
#define TSL25911_COMMAND_BIT 0xA0 // CMD (bit 7) + TRANSACTION (bits 6:5) set to 11 for special function

/** @brief TSL2591 Register Addresses. */
#define TSL25911_REG_ENABLE     (0x00 | TSL25911_COMMAND_BIT) ///< Enable register
#define TSL25911_REG_CONTROL    (0x01 | TSL25911_COMMAND_BIT) ///< Control register
#define TSL25911_REG_AILTL      (0x04 | TSL25911_COMMAND_BIT) ///< ALS interrupt low threshold (low byte)
#define TSL25911_REG_AILTH      (0x05 | TSL25911_COMMAND_BIT) ///< ALS interrupt low threshold (high byte)
#define TSL25911_REG_AIHTL      (0x06 | TSL25911_COMMAND_BIT) ///< ALS interrupt high threshold (low byte)
#define TSL25911_REG_AIHTH      (0x07 | TSL25911_COMMAND_BIT) ///< ALS interrupt high threshold (high byte)
#define TSL25911_REG_NPAILTL    (0x08 | TSL25911_COMMAND_BIT) ///< No-persist ALS low threshold (low byte)
#define TSL25911_REG_NPAILTH    0x09                         ///< No-persist ALS low threshold (high byte)
#define TSL25911_REG_NPAIHTL    0x0A                         ///< No-persist ALS high threshold (low byte)
#define TSL25911_REG_NPAIHTH    0x0B                         ///< No-persist ALS high threshold (high byte)
#define TSL25911_REG_PERSIST    (0x0C | TSL25911_COMMAND_BIT) ///< Interrupt persistence filter register
#define TSL25911_REG_PID        (0x11 | TSL25911_COMMAND_BIT) ///< Package ID register
#define TSL25911_REG_ID         (0x12 | TSL25911_COMMAND_BIT) ///< Device ID register
#define TSL25911_REG_STATUS     (0x13 | TSL25911_COMMAND_BIT) ///< Status register
#define TSL25911_REG_C0DATA     (0x14 | TSL25911_COMMAND_BIT) ///< Channel 0 data (low byte)
#define TSL25911_REG_C1DATA     0x16                         ///< Channel 1 data (low byte)

/**
 * @brief Special functions for TSL2591 commands.
 */
typedef enum {
    TSL25911_SPECIAL_FUNCTION_INT_SET				= 0xE4, ///< Interrupt set - forces an interrupt
    TSL25911_SPECIAL_FUNCTION_CLEAR_ALS_INT	= 0xE6, ///< Clears ALS interrupt
    TSL25911_SPECIAL_FUNCTION_CLEAR_ALL_INT	= 0xE7, ///< Clears ALS and no persist ALS interrupt
    TSL25911_SPECIAL_FUNCTION_CLEAR_NP_INT		= 0xEA  ///< Clears no persist ALS interrupt
} TSL25911_SpecialFunction_t;

/** @brief Lux calculation coefficient. */
#define TSL25911_LUX_DF 408.0F

/**
 * @brief Structure representing the COMMAND register of TSL2591.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Full register value.
        struct __attribute__((packed)) {
            uint8_t ADDR_SF			: 5; ///< Address/Special function field.
            uint8_t TRANSACTION : 2; ///< Transaction type.
            uint8_t CMD					: 1; ///< Command bit (must be set to 1).
        } BitField; ///< Bit field representation.
    } Val;
} TSL25911_COMMAND_t;

/**
 * @brief Enumeration for the ENABLE register states.
 */
typedef enum {
    TSL25911_STATE_DISABLED         = 0x00, ///< Disabled state.
    TSL25911_STATE_PON              = 0x01, ///< Power ON only.
    TSL25911_STATE_AEN              = 0x02, ///< ALS Enable only.
    TSL25911_STATE_PON_AEN          = 0x03, ///< Power ON and ALS Enable.
    TSL25911_STATE_AIEN             = 0x10, ///< ALS Interrupt Enable only.
    TSL25911_STATE_PON_AEN_AIEN     = 0x13, ///< Power ON, ALS Enable, and ALS Interrupt Enable.
    TSL25911_STATE_SAI              = 0x40, ///< Sleep After Interrupt.
    TSL25911_STATE_NPIEN            = 0x80, ///< No Persist Interrupt Enable.
    TSL25911_STATE_ENABLE           = 0x93  ///< Fully enabled state.
} TSL25911_STATE_t;

/**
 * @brief Structure representing the ENABLE register.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Full register value.
        struct __attribute__((packed)) {
            uint8_t PON				: 1; ///< Power ON.
            uint8_t AEN				: 1; ///< ALS Enable.
            uint8_t Reserved	: 2; ///< Reserved bits.
            uint8_t AIEN			: 1; ///< ALS Interrupt Enable.
            uint8_t Reserved2	: 1; ///< Reserved bit.
            uint8_t SAI				: 1; ///< Sleep after interrupt.
            uint8_t NPIEN			: 1; ///< No Persist Interrupt Enable.
        } BitField; ///< Bit field representation.
    } Val;
} TSL25911_ENABLE_t;

/**
 * @brief Enumeration for gain settings.
 */
typedef enum {
    TSL25911_GAIN_LOW	= 0x00, ///< 1x Gain.
    TSL25911_GAIN_MED	= 0x10, ///< 25x Gain.
    TSL25911_GAIN_HIGH	= 0x20, ///< 428x Gain.
    TSL25911_GAIN_MAX	= 0x30  ///< 9876x Gain.
} TSL25911_GAIN_t;

/**
 * @brief Enumeration for integration times.
 */
typedef enum {
    TSL25911_INTEGRATION_100MS = 0x00, ///< 100ms.
    TSL25911_INTEGRATION_200MS = 0x01, ///< 200ms.
    TSL25911_INTEGRATION_300MS = 0x02, ///< 300ms.
    TSL25911_INTEGRATION_400MS = 0x03, ///< 400ms.
    TSL25911_INTEGRATION_500MS = 0x04, ///< 500ms.
    TSL25911_INTEGRATION_600MS = 0x05  ///< 600ms.
} TSL25911_INTEGRATION_t;

/**
 * @brief Structure representing the CONTROL register.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Full 8-bit CONTROL register value.
        struct __attribute__((packed)) {
            uint8_t ATIME			: 3; ///< Integration time setting (bits [2:0]).
            uint8_t Reserved	: 1; ///< Reserved bit (bit 3).
            uint8_t AGAIN			: 2; ///< Analog gain setting (bits [5:4]).
            uint8_t Reserved1 : 1; ///< Reserved bit (bit 6).
            uint8_t SRESET		: 1; ///< System reset (bit 7).
        } BitField; ///< Bit field representation of the CONTROL register.
    } Val;
} TSL25911_CONTROL_t;

/**
 * @brief Structure representing the ALS register.
 */
/**
 * @brief Structure representing ALS Interrupt Thresholds.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t FullArray[4]; ///< Combined low and high thresholds as an array for I2C transfer.
        struct {
            union {
                struct {
                    uint8_t LSB; ///< Low byte of the threshold.
                    uint8_t MSB; ///< High byte of the threshold.
                } Bytes; ///< Byte-wise representation of the threshold.
                uint16_t Value; ///< Combined 16-bit threshold value.
            } LowThreshold; ///< ALS Interrupt Low Threshold.

            union {
                struct {
                    uint8_t LSB; ///< Low byte of the threshold.
                    uint8_t MSB; ///< High byte of the threshold.
                } Bytes; ///< Byte-wise representation of the threshold.
                uint16_t Value; ///< Combined 16-bit threshold value.
            } HighThreshold; ///< ALS Interrupt High Threshold.
        };
    };
} TSL25911_ALS_Interrupt_Threshold_t;

/**
 * @brief Structure representing the NPAI register.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t FullArray[4]; ///< Combined low and high thresholds as an array.
        struct __attribute__((packed)) {
            uint8_t LowLSB;  ///< Low Threshold LSB.
            uint8_t LowMSB;  ///< Low Threshold MSB.
            uint8_t HighLSB; ///< High Threshold LSB.
            uint8_t HighMSB; ///< High Threshold MSB.
        } FullBytes; ///< Byte-wise representation for the full array.
    } Thresholds; ///< No Persist ALS Interrupt Thresholds.
} TSL25911_NPAI_Interrupt_Threshold_t;

/**
 * @brief Enumeration for PERSIST.
 */
typedef enum {
    TSL25911_PERSIST_EVERY_CYCLE = 0x00, ///< Interrupt every ALS cycle.
    TSL25911_PERSIST_ANY_OUT_OF_RANGE = 0x01, ///< Interrupt on any out-of-range value.
    TSL25911_PERSIST_2_CYCLES	= 0x02, ///< Interrupt after 2 consecutive out-of-range values.
    TSL25911_PERSIST_3_CYCLES	= 0x03, ///< Interrupt after 3 consecutive out-of-range values.
    TSL25911_PERSIST_5_CYCLES	= 0x04, ///< Interrupt after 5 consecutive out-of-range values.
    TSL25911_PERSIST_10_CYCLES = 0x05, ///< Interrupt after 10 consecutive out-of-range values.
    TSL25911_PERSIST_15_CYCLES = 0x06, ///< Interrupt after 15 consecutive out-of-range values.
    TSL25911_PERSIST_20_CYCLES = 0x07, ///< Interrupt after 20 consecutive out-of-range values.
    TSL25911_PERSIST_25_CYCLES = 0x08, ///< Interrupt after 25 consecutive out-of-range values.
    TSL25911_PERSIST_30_CYCLES = 0x09, ///< Interrupt after 30 consecutive out-of-range values.
    TSL25911_PERSIST_35_CYCLES = 0x0A, ///< Interrupt after 35 consecutive out-of-range values.
    TSL25911_PERSIST_40_CYCLES = 0x0B, ///< Interrupt after 40 consecutive out-of-range values.
    TSL25911_PERSIST_45_CYCLES = 0x0C, ///< Interrupt after 45 consecutive out-of-range values.
    TSL25911_PERSIST_50_CYCLES = 0x0D, ///< Interrupt after 50 consecutive out-of-range values.
    TSL25911_PERSIST_55_CYCLES = 0x0E, ///< Interrupt after 55 consecutive out-of-range values.
    TSL25911_PERSIST_60_CYCLES	= 0x0F  ///< Interrupt after 60 consecutive out-of-range values.
} TSL25911_PERSIST_t;

/**
 * @brief Structure representing the NPAI register.
 */
typedef struct __attribute__((packed)) {
    union {
        struct {
            union {
                struct {
                    uint8_t LSB; ///< Low byte of Channel 0 data.
                    uint8_t MSB; ///< High byte of Channel 0 data.
                } Bytes; ///< Byte-wise representation of Channel 0 data.
                uint16_t Value; ///< 16-bit representation of Channel 0 data.
            } CH0; ///< Channel 0 (Full Spectrum) data.

            union {
                struct {
                    uint8_t LSB; ///< Low byte of Channel 1 data.
                    uint8_t MSB; ///< High byte of Channel 1 data.
                } Bytes; ///< Byte-wise representation of Channel 1 data.
                uint16_t Value; ///< 16-bit representation of Channel 1 data.
            } CH1; ///< Channel 1 (Infrared) data.
        } Channels; ///< Channel data as structured fields.

        uint8_t Array[4]; ///< Array representation for direct I2C mapping.
    };
} TSL25911_ALS_Data_t;

/**
 * @brief Structure representing the STATUS register.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Full 8-bit STATUS register value.
        struct __attribute__((packed)) {
            uint8_t AVALID		: 1; ///< ALS Valid bit (bit 0). Set when new data is available.
            uint8_t Reserved	: 3; ///< Reserved bits (bits [3:1]).
            uint8_t AINT			: 1; ///< ALS Interrupt bit (bit 4). Set when ALS interrupt occurs.
            uint8_t NPINTR		: 1; ///< No-persist Interrupt bit (bit 5). Set when a no-persist interrupt occurs.
            uint8_t Reserved2	: 2; ///< Reserved bits (bits [7:6]).
        } BitField; ///< Bit field representation of the STATUS register.
    } Val;
} TSL25911_STATUS_t;

/**
 * @brief Combined structure for all TSL2591 registers.
 */
typedef struct __attribute__((packed)) {
    TSL25911_ENABLE_t ENABLE; ///< ENABLE register.
    TSL25911_CONTROL_t CTRL; ///< CONTROL register.
    TSL25911_ALS_Interrupt_Threshold_t ALS; ///< ALS Interrupt Threshold Registers.
		//TSL25911_NPAI_Interrupt_Threshold_t NPAI; ///< No Persist ALS Interrupt Threshold (optional, same as ALS).
    TSL25911_PERSIST_t PERSIST; ///< PERSIST register.
    //uint8_t PID; ///< Device Identification (0x50).
    uint8_t ID; ///< Device Identification (0x50).
    TSL25911_STATUS_t STATUS; ///< STATUS register.
		TSL25911_ALS_Data_t DATA;
} TSL25911_REGISTERS_t;

/**
 * @brief Structure to hold light data.
 */
typedef struct __attribute__((packed)) {
    uint16_t FullSpectrum;	///< Full-spectrum (Channel 0) light data.
    uint16_t Infrared;			///< Infrared (Channel 1) light data.
    uint16_t Visible;				///< Visible light data, calculated as FullSpectrum - Infrared.
    float Lux;							///< Calculated lux value based on sensor data.
} TSL25911_LightData_t;

// Function prototypes
HAL_StatusTypeDef TSL25911_Init(void);
HAL_StatusTypeDef TSL25911_Enable(TSL25911_STATE_t state);
HAL_StatusTypeDef TSL25911_Reset(void);
HAL_StatusTypeDef TSL25911_SetGainAndIntegrationTime(TSL25911_GAIN_t gain, TSL25911_INTEGRATION_t integrationTime);
HAL_StatusTypeDef TSL25911_SetInterruptThresholds(uint16_t low, uint16_t high);
HAL_StatusTypeDef TSL25911_SetPersistence(TSL25911_PERSIST_t persistenceValue);
HAL_StatusTypeDef TSL25911_ReadID(void);
HAL_StatusTypeDef TSL25911_GetStatus(void);
HAL_StatusTypeDef TSL25911_ClearInterrupt(TSL25911_SpecialFunction_t interrupt);
HAL_StatusTypeDef TSL25911_ReadLightData(TSL25911_LightData_t *lightData);

#endif // TSL25911_H

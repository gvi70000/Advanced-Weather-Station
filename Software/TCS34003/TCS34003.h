#ifndef TCS34003_H
#define TCS34003_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

/** @brief TCS34003 I2C Address (default). */
#define TCS34003_I2C_ADDR (0x29 << 1) // Adjust based on hardware configuration

// Register Addresses for TCS3400
#define TCS34003_REG_ENABLE		0x80  // Enables states and interrupts, Reset Value: 0x00
#define TCS34003_REG_ATIME		0x81  // RGBC integration time, Reset Value: 0xFF
#define TCS34003_REG_WTIME		0x83  // Wait time, Reset Value: 0xFF

// Clear Interrupt Threshold Registers
#define TCS34003_REG_AILTL		0x84  // Clear interrupt low threshold low byte, Reset Value: 0x00
#define TCS34003_REG_AILTH		0x85  // Clear interrupt low threshold high byte, Reset Value: 0x00
#define TCS34003_REG_AIHTL		0x86  // Clear interrupt high threshold low byte, Reset Value: 0x00
#define TCS34003_REG_AIHTH		0x87  // Clear interrupt high threshold high byte, Reset Value: 0x00

// Interrupt and Configuration Registers
#define TCS34003_REG_PERS			0x8C  // Interrupt persistence filter, Reset Value: 0x00
#define TCS34003_REG_CONFIG		0x8D  // Configuration register, Reset Value: 0x40
#define TCS34003_REG_CONTROL	0x8F  // Gain control register, Reset Value: 0x00
#define TCS34003_REG_AUX			0x90  // Auxiliary control register, Reset Value: 0x00

// Device Identification Registers
#define TCS34003_REG_REVID		0x91  // Revision ID register, Read-only
#define TCS34003_REG_ID				0x92  // Device ID register, Read-only

// Status Register
#define TCS34003_REG_STATUS		0x93  // Device status, Reset Value: 0x00

// RGBC Data Registers
#define TCS34003_REG_CDATAL		0x94  // Clear/IR channel low data register, Reset Value: 0x00
#define TCS34003_REG_CDATAH		0x95  // Clear/IR channel high data register, Reset Value: 0x00
#define TCS34003_REG_RDATAL		0x96  // Red ADC low data register, Reset Value: 0x00
#define TCS34003_REG_RDATAH		0x97  // Red ADC high data register, Reset Value: 0x00
#define TCS34003_REG_GDATAL		0x98  // Green ADC low data register, Reset Value: 0x00
#define TCS34003_REG_GDATAH		0x99  // Green ADC high data register, Reset Value: 0x00
#define TCS34003_REG_BDATAL		0x9A  // Blue ADC low data register, Reset Value: 0x00
#define TCS34003_REG_BDATAH		0x9B  // Blue ADC high data register, Reset Value: 0x00

// IR Channel and Interrupt Control
#define TCS34003_REG_IR				0xC0  // Access IR Channel, Reset Value: 0x00
#define TCS34003_REG_IFORCE		0xE4  // Force interrupt, Write-only
#define TCS34003_REG_CICLEAR	0xE6  // Clear channel interrupt clear, Write-only
#define TCS34003_REG_AICLEAR	0xE7  // Clear all interrupts, Write-only

// Constants for Correlated Color Temperature (CCT) calculation
#define TCS34003_CCT_COEFF_X_RED    -0.14282f ///< Coefficient for Red channel in X calculation
#define TCS34003_CCT_COEFF_X_GREEN   1.54924f ///< Coefficient for Green channel in X calculation
#define TCS34003_CCT_COEFF_X_BLUE   -0.95641f ///< Coefficient for Blue channel in X calculation

#define TCS34003_CCT_COEFF_Y_RED    -0.32466f ///< Coefficient for Red channel in Y calculation
#define TCS34003_CCT_COEFF_Y_GREEN   1.57837f ///< Coefficient for Green channel in Y calculation
#define TCS34003_CCT_COEFF_Y_BLUE   -0.73191f ///< Coefficient for Blue channel in Y calculation

#define TCS34003_CCT_COEFF_Z_RED    -0.68202f ///< Coefficient for Red channel in Z calculation
#define TCS34003_CCT_COEFF_Z_GREEN   0.77073f ///< Coefficient for Green channel in Z calculation
#define TCS34003_CCT_COEFF_Z_BLUE    0.56332f ///< Coefficient for Blue channel in Z calculation

#define TCS34003_CCT_COEFF_C1       449.0f    ///< CCT calculation coefficient (n^3)
#define TCS34003_CCT_COEFF_C2      3525.0f    ///< CCT calculation coefficient (n^2)
#define TCS34003_CCT_COEFF_C3      6823.3f    ///< CCT calculation coefficient (n)
#define TCS34003_CCT_COEFF_C4      5520.33f   ///< CCT calculation constant

// Constants for Luminance (lux) calculation
#define TCS34003_LUX_COEFF_CLEAR    0.136f    ///< Coefficient for Clear channel in Lux calculation
#define TCS34003_LUX_COEFF_GREEN    0.119f    ///< Coefficient for Green channel in Lux calculation

// Constants for Irradiance calculation
#define TCS34003_RESPONSIVITY_CLEAR 14.0f     ///< Responsivity for Clear channel (counts/µW/cm²)
#define TCS34003_MICRO_TO_W_CONV    0.0001f   ///< Conversion factor from µW/cm² to W/m²

#define TCS34003_MID_VAL    0x7FFF   ///< 32767 as mid value of 65535
/**
 * @brief Structure ENABLE Register (ATIME 0x80)
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Full register value.
        struct __attribute__((packed)) {
            uint8_t PON		: 1; ///< Bit 0 - Power ON bit. Activates internal oscillator (0 = Disable, 1 = Enable).
            uint8_t AEN		: 1; ///< Bit 1 - ADC Enable. Activates the four-channel (RGBC) ADC (0 = Disable, 1 = Enable).
            uint8_t RES2	: 1; ///< Bit 2 - Reserved. Write as 0.
            uint8_t WEN		: 1; ///< Bit 3 - Wait Enable. Activates the wait timer (0 = Disable, 1 = Enable).
            uint8_t AIEN	: 1; ///< Bit 4 - ALS Interrupt Enable. Allows ALS interrupts (0 = Disable, 1 = Enable).
            uint8_t RES5	: 1; ///< Bit 5 - Reserved. Write as 0.
            uint8_t SAI		: 1; ///< Bit 6 - Sleep After Interrupt. Powers down at end of RGBC cycle if interrupt is generated (0 = Disable, 1 = Enable).
            uint8_t RES7	: 1; ///< Bit 7 - Reserved. Write as 0.
        } BitField; ///< Bit field representation.
    } Val;
} TCS34003_REG_ENABLE_t;

/**
 * @brief Enum for RGBC Integration Time Register (ATIME 0x81)
 * Controls the internal integration time of the RGBC channel ADCs.
 */
typedef enum {
    TCS34003_ATIME_1_CYCLE		= 0xFF, ///< 1 cycle (2.78 ms), Max Count: 1024
    TCS34003_ATIME_10_CYCLES	= 0xF6, ///< 10 cycles (27.8 ms), Max Count: 10240
    TCS34003_ATIME_37_CYCLES	= 0xDB, ///< 37 cycles (103 ms), Max Count: 37888
    TCS34003_ATIME_64_CYCLES	= 0xC0, ///< 64 cycles (178 ms), Max Count: 65535
    TCS34003_ATIME_256_CYCLES	= 0x00  ///< 256 cycles (712 ms), Max Count: 65535
} TCS34003_ATIME_t;

/**
 * @brief Enum for Wait Time Register (WTIME 0x83)
 * Controls the amount of time the device spends in a low-power mode.
 * Wait time is in 2.78 ms increments unless the WLONG bit is asserted, in which case
 * the wait times are 12x longer.
 */
typedef enum {
    TCS34003_WTIME_1_CYCLE			= 0xFF, ///< 1 cycle: Wait Time = 2.78 ms (WLONG=0) or 0.03 s (WLONG=1)
    TCS34003_WTIME_85_CYCLES		= 0xAB, ///< 85 cycles: Wait Time = 236 ms (WLONG=0) or 2.84 s (WLONG=1)
    TCS34003_WTIME_256_CYCLES	= 0x00 ///< 256 cycles: Wait Time = 712 ms (WLONG=0) or 8.54 s (WLONG=1)
} TCS34003_WTIME_t;

/**
 * @brief Clear Channel Interrupt Threshold Register (WTIME 0x84-0x87)
 */
 
typedef struct __attribute__((packed)) {
    union {
        uint8_t FullArray[4]; ///< Combined low and high thresholds as an array for I2C transfer.
        struct {
            union {
                struct {
                    uint8_t LSB; ///< Low byte of the low threshold.
                    uint8_t MSB; ///< High byte of the low threshold.
                } Bytes; ///< Byte-wise representation of the low threshold.
                uint16_t Value; ///< Combined 16-bit low threshold value.
            } LowThreshold; ///< Clear Channel Low Threshold.

            union {
                struct {
                    uint8_t LSB; ///< Low byte of the high threshold.
                    uint8_t MSB; ///< High byte of the high threshold.
                } Bytes; ///< Byte-wise representation of the high threshold.
                uint16_t Value; ///< Combined 16-bit high threshold value.
            } HighThreshold; ///< Clear Channel High Threshold.
        };
    };
} TCS34003_CC_Threshold_t;

/**
 * @brief Enum for Clear Interrupt Persistence 0x8C (APERS 3:0)
 * Controls the rate of Clear channel interrupts to the host processor.
 */
typedef enum {
    TCS34003_APERS_EVERY_CYCLE       = 0x00, ///< Every RGBC cycle generates an interrupt.
    TCS34003_APERS_OUT_OF_RANGE_1    = 0x01, ///< Any value outside of threshold range.
    TCS34003_APERS_OUT_OF_RANGE_2    = 0x02, ///< 2 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_3    = 0x03, ///< 3 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_5    = 0x04, ///< 5 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_10   = 0x05, ///< 10 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_15   = 0x06, ///< 15 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_20   = 0x07, ///< 20 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_25   = 0x08, ///< 25 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_30   = 0x09, ///< 30 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_35   = 0x0A, ///< 35 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_40   = 0x0B, ///< 40 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_45   = 0x0C, ///< 45 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_50   = 0x0D, ///< 50 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_55   = 0x0E, ///< 55 consecutive values out of range.
    TCS34003_APERS_OUT_OF_RANGE_60   = 0x0F  ///< 60 consecutive values out of range.
} TCS34003_APERS_t;

/**
 * @brief Enum for WLONG bit in the CONFIG Register (0x8D)
 * Controls the wait long time configuration for the TCS3400.
 * Note: Bit 6 must always be set to 1.
 */
typedef enum {
    TCS34003_CONFIG_WLONG_DISABLE = 0x40, ///< WLONG Disabled. Wait time as defined in WTIME register.
    TCS34003_CONFIG_WLONG_ENABLE  = 0x42  ///< WLONG Enabled. Wait time is 12x longer than WTIME register.
} TCS34003_CONFIG_t;

/**
 * @brief Enum for AGAIN (Bits 1:0) in the CONTROL Register (0x8F)
 * Controls the RGBC Gain settings for the TCS3400.
 */
typedef enum {
    TCS34003_AGAIN_1X  = 0x00, ///< 1x Gain.
    TCS34003_AGAIN_4X  = 0x01, ///< 4x Gain.
    TCS34003_AGAIN_16X = 0x02, ///< 16x Gain.
    TCS34003_AGAIN_64X = 0x03  ///< 64x Gain.
} TCS34003_AGAIN_t;

/**
 * @brief Enum for ASIEN (Bit 5) in the AUX Register (0x90)
 * Enables or disables the ALS saturation detection interrupt.
 */
typedef enum {
    TCS34003_AUX_ASIEN_DISABLE = 0x00, ///< ALS Saturation Interrupt Disabled.
    TCS34003_AUX_ASIEN_ENABLE  = 0x20  ///< ALS Saturation Interrupt Enabled.
} TCS34003_AUX_t;

// Revision ID Register (REVID 0x91)
// ID Register (ID 0x92)

/**
 * @brief Structure for the STATUS Register (0x93)
 * Provides the internal status of the TCS3400 device.
 */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value; ///< Full register value.
        struct __attribute__((packed)) {
            uint8_t AVALID	: 1; ///< Bit 0 - RGBC Valid. Indicates RGBC cycle completion since AEN was asserted.
            uint8_t RES13		: 3; ///< Bits 1-3 - Reserved. Must be read as 0.
            uint8_t AINT		: 1; ///< Bit 4 - ALS Interrupt. Indicates ALS thresholds and persistence were met.
            uint8_t RES56		: 2; ///< Bits 5-6 - Reserved. Must be read as 0.
            uint8_t ASAT		: 1; ///< Bit 7 - ALS Saturation. Indicates sensor reached dynamic range upper limit.
        } BitField; ///< Bit field representation of the STATUS register.
    } Val;
} TCS34003_STATUS_t;

/**
 * @brief Structure for RGBC Data Registers (0x94 - 0x9B)
 * Holds 16-bit values for Clear, Red, Green, and Blue channel data.
 */
typedef struct __attribute__((packed)) {
    union {
        struct {
            uint16_t ClearChannel; ///< Clear/IR channel data (CDATAL, CDATAH).
            uint16_t RedChannel;   ///< Red channel data (RDATAL, RDATAH).
            uint16_t GreenChannel; ///< Green channel data (GDATAL, GDATAH).
            uint16_t BlueChannel;  ///< Blue channel data (BDATAL, BDATAH).
        };
        uint8_t FullArray[8]; ///< Combined RGBC channel data as a byte array.
    };
} TCS34003_RGBC_Data_t;

/**
 * @brief Structure to hold IR data with a FullArray for direct I2C transfers
 *        and bitwise access for individual bytes.
 */
typedef struct __attribute__((packed)) {
    union {
        uint16_t Value; ///< Combined 16-bit IR data value.
        uint8_t FullArray[2]; ///< Full 2-byte array for I2C transfers.
    };
} TCS34003_IR_Data_t;

/**
 * @brief Enum for IR Sensor Access (Bit 7) in the IR Register (0xC0)
 * Controls the mapping of the IR channel to the clear channel.
 */
typedef enum {
    TCS34003_IR_ACCESS_DISABLE = 0x00, ///< IR Sensor access disabled. Clear channel reports normal data.
    TCS34003_IR_ACCESS_ENABLE  = 0x80  ///< IR Sensor access enabled. Clear channel reports data from IR sensor.
} TCS34003_IR_Access_t;

// Clear Interrupt Registers
// 0xE4 IFORCE	Forces an interrupt (any value)
// 0xE6 CICLEAR Clear channel interrupt clear (any value)
// 0xE7 AICLEAR Clears all interrupts (any value)

typedef struct __attribute__((packed)) {
		TCS34003_RGBC_Data_t RGBC;		///
		TCS34003_IR_Data_t IR;		///
		float CCT;			///< Correlated Color Temperature (CCT) in Kelvin
		float Lux;			///< Luminance in lux
		float Irradiance;	///< Irradiance in W/m² based on sensor data and responsivity factors
} TCS34003_LightData_t;

typedef struct __attribute__((packed)) {
		TCS34003_REG_ENABLE_t ENABLE;	///< (ENABLE 0x80 R/W)
		TCS34003_ATIME_t ATIME;			///< (ATIME 0x81 R/W)
		TCS34003_WTIME_t WTIME;			///< (WTIME 0x83 R/W)
		TCS34003_CC_Threshold_t CC_TH;	///< (CC_TH 0x84-0x87 R/W)
		TCS34003_APERS_t APERS;			///< (APERS 0x8C R/W)
		TCS34003_CONFIG_t CONFIG;		///< (CONFIG 0x8D R/W)
		TCS34003_AGAIN_t AGAIN;			///< (AGAIN 0x8F R/W)
		TCS34003_AUX_t AUX;				///< (AUX 0x90 R/W)
		// REV_ID 0x91
		// ID 0x92
		TCS34003_STATUS_t STATUS;		///< (STATUS 0x93 R)
		TCS34003_IR_Access_t IR_Access;	///< (IR_Access 0xC0 R/W)
		uint8_t IFORCE;					///< (IFORCE 0xE4 R/W)
		uint8_t CICLEAR;				///< (CICLEAR 0xE6 R/W)
		uint8_t AICLEAR;				///< (AICLEAR 0xE7 R/W)
} TCS34003_REGISTERS_t;

/**
 * @brief Initializes the TCS3400 sensor.
 * Configures the sensor for minimum gain and sets the data-ready interrupt to trigger every second.
 * @retval HAL_StatusTypeDef HAL_OK if initialization is successful, otherwise an error code.
 */
HAL_StatusTypeDef TCS34003_Init(void);

/**
 * @brief Set the ENABLE register.
 * @param value The new ENABLE register value.
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetEnable(uint8_t value);

/**
 * @brief Set the ATIME register (RGBC Integration Time).
 * @param value Integration time (TCS34003_ATIME_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetIntegrationTime(TCS34003_ATIME_t value);

/**
 * @brief Set the WTIME register (Wait Time).
 * @param value Wait time (TCS34003_WTIME_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetWaitTime(TCS34003_WTIME_t value);

/**
 * @brief Set the Clear Channel Interrupt Threshold.
 * @param low Low threshold value.
 * @param high High threshold value.
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetClearChannelThreshold(uint16_t low, uint16_t high);

/**
 * @brief Enables interrupts by setting the AIEN bit in the ENABLE register.
 * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef TCS34003_EnableInterrupts(void);

/**
 * @brief Set the APERS register (Interrupt Persistence Filter).
 * @param value Persistence filter (TCS34003_APERS_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetInterruptPersistence(TCS34003_APERS_t value);

/**
 * @brief Set the CONFIG register.
 * @param value Configuration value (TCS34003_CONFIG_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetConfig(TCS34003_CONFIG_t value);

/**
 * @brief Set the CONTROL register (Gain Control).
 * @param value Gain control (TCS34003_AGAIN_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetGain(TCS34003_AGAIN_t value);

/**
 * @brief Set the AUX register (ALS Saturation Interrupt Enable).
 * @param value Saturation interrupt enable (TCS34003_AUX_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetAux(TCS34003_AUX_t value);

/**
 * @brief Set the IR register (IR Sensor Access).
 * @param value IR sensor access (TCS34003_IR_Access_t).
 * @retval HAL_StatusTypeDef HAL_OK if successful, error code otherwise.
 */
HAL_StatusTypeDef TCS34003_SetIRAccess(TCS34003_IR_Access_t value);

/**
 * @brief Clears all interrupts.
 * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef TCS34003_ClearInterrupts(void);

/**
 * @brief Forces an interrupt by writing to the IFORCE register.
 * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef TCS34003_ForceInterrupt(void);

HAL_StatusTypeDef TCS34003_GetStatus(void);

/**
 * @brief Reads the light data (CLEAR, RED, GREEN, BLUE, IR) and calculates
 *        Correlated Color Temperature (CCT), Luminance (lux), and Irradiance (W/m²).
 * @param[out] data Pointer to a TCS34003_LightData_t structure to store the light data and calculated values.
 * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise an error code.
 */
HAL_StatusTypeDef TCS34003_GetLightData(TCS34003_LightData_t* data);

#endif // TCS34003_H

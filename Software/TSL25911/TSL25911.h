#ifndef TSL25911_H
#define TSL25911_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

// TSL25911 I2C Address
#define TSL25911_I2C_ADDR (0x29 << 1) // Adjust based on hardware configuration

// Register addresses
#define TSL25911_REG_ENABLE     0x00
#define TSL25911_REG_CONTROL    0x01
#define TSL25911_REG_AILTL      0x04
#define TSL25911_REG_AILTH      0x05
#define TSL25911_REG_AIHTL      0x06
#define TSL25911_REG_AIHTH      0x07
#define TSL25911_REG_ID         0x12
#define TSL25911_REG_C0DATA     0x14
#define TSL25911_REG_C1DATA     0x16
#define TSL25911_REG_STATUS     0x13

// Enable register fields
#define TSL25911_ENABLE_POWERON 0x01
#define TSL25911_ENABLE_AEN     0x02
#define TSL25911_ENABLE_AIEN    0x10

typedef enum {
    TSL25911_GAIN_LOW  = 0x00, // 1x Gain
    TSL25911_GAIN_MED  = 0x10, // 25x Gain
    TSL25911_GAIN_HIGH = 0x20, // 428x Gain
    TSL25911_GAIN_MAX  = 0x30  // 9876x Gain
} TSL25911_GAIN_t;

typedef enum {
    TSL25911_INTEGRATION_100MS = 0x00, // 100ms Integration Time
    TSL25911_INTEGRATION_200MS = 0x01, // 200ms Integration Time
    TSL25911_INTEGRATION_300MS = 0x02, // 300ms Integration Time
    TSL25911_INTEGRATION_400MS = 0x03, // 400ms Integration Time
    TSL25911_INTEGRATION_500MS = 0x04, // 500ms Integration Time
    TSL25911_INTEGRATION_600MS = 0x05  // 600ms Integration Time
} TSL25911_INTEGRATION_t;

/* TSL25911 COMMAND Register Structure */
typedef struct __attribute__((packed)) {
    uint8_t ADDR_SF : 5;    // Bits [4:0]: Address field / Special function field
    uint8_t TRANSACTION : 2; // Bits [6:5]: Transaction type
    uint8_t CMD : 1;        // Bit 7: Command bit (must be set to 1)
} TSL25911_COMMAND_t;

/* TSL25911 ENABLE Register Structure */
typedef struct __attribute__((packed)) {
    uint8_t PON : 1;        // Bit 0: Power ON
    uint8_t AEN : 1;        // Bit 1: ALS Enable
    uint8_t Reserved1 : 2;  // Bits [3:2]: Reserved, write as 0
    uint8_t AIEN : 1;       // Bit 4: ALS Interrupt Enable
    uint8_t Reserved2 : 1;  // Bit 5: Reserved, write as 0
    uint8_t SAI : 1;        // Bit 6: Sleep After Interrupt
    uint8_t NPIEN : 1;      // Bit 7: No Persist Interrupt Enable
} TSL25911_ENABLE_t;

/* TSL25911 CONTROL Register Structure */
typedef struct __attribute__((packed)) {
    uint8_t ATIME : 3;      // Bits [2:0]: ALS Integration Time
    uint8_t Reserved1 : 1;  // Bit 3: Reserved, write as 0
    uint8_t AGAIN : 2;      // Bits [5:4]: ALS Gain Mode
    uint8_t Reserved2 : 1;  // Bit 6: Reserved, write as 0
    uint8_t SRESET : 1;     // Bit 7: System Reset
} TSL25911_CONTROL_t;

/* TSL25911 ALS Interrupt Threshold Registers Structure */
typedef struct __attribute__((packed)) {
    uint16_t AIL;           // ALS low threshold (0x04 and 0x05)
    uint16_t AIH;           // ALS high threshold (0x06 and 0x07)
    uint16_t NPAIL;         // No Persist ALS low threshold (0x08 and 0x09)
    uint16_t NPAIH;         // No Persist ALS high threshold (0x0A and 0x0B)
} TSL25911_ALS_Interrupt_Threshold_t;

/* TSL25911 PERSIST Register Structure */
typedef struct __attribute__((packed)) {
    uint8_t APERS : 4;      // Bits [3:0]: ALS Interrupt Persistence Filter
    uint8_t Reserved : 4;   // Bits [7:4]: Reserved, write as 0
} TSL25911_PERSIST_t;

/* TSL25911 PID Register Structure */
typedef struct __attribute__((packed)) {
    uint8_t Reserved1 : 3;  // Bits [2:0]: Reserved
    uint8_t PID : 2;        // Bits [4:3]: Package Identification
    uint8_t Reserved2 : 3;  // Bits [7:5]: Reserved
} TSL25911_PID_t;

/* TSL25911 STATUS Register Structure */
typedef struct __attribute__((packed)) {
    uint8_t AVALID : 1;     // Bit 0: ALS Valid
    uint8_t Reserved1 : 3;  // Bits [3:1]: Reserved, write as 0
    uint8_t AINT : 1;       // Bit 4: ALS Interrupt
    uint8_t NPINTR : 1;     // Bit 5: No-persist Interrupt
    uint8_t Reserved2 : 2;  // Bits [7:6]: Reserved, write as 0
} TSL25911_STATUS_t;

/* TSL25911 ALS Data Registers Structure */
typedef struct __attribute__((packed)) {
    uint16_t CH0_DATA;      // ALS CH0 data (0x14 and 0x15)
    uint16_t CH1_DATA;      // ALS CH1 data (0x16 and 0x17)
} TSL25911_ALS_Data_t;

/* Combined TSL25911 Register Structure */
typedef struct __attribute__((packed)) {
    TSL25911_COMMAND_t CMD;
    TSL25911_ENABLE_t EN;
    TSL25911_CONTROL_t CTRL;
    TSL25911_ALS_Interrupt_Threshold_t INT;
    TSL25911_PERSIST_t PERSIST;
    TSL25911_PID_t PID;
    uint8_t ID;             // Device Identification (0x50)
    TSL25911_STATUS_t STATUS;
    TSL25911_ALS_Data_t DATA;
} TSL25911_REGISTERS_t;

// Structure to hold light data
typedef struct __attribute__((packed)) {
    uint16_t FullSpectrum; // Full-spectrum (CHAN0) light data
    uint16_t Infrared;     // Infrared (CHAN1) light data
    uint16_t Visible;      // Visible light (FullSpectrum - Infrared)
    float Lux;             // Calculated lux value
} TSL25911_LightData_t;

// Function prototypes
HAL_StatusTypeDef TSL25911_Init(void);
HAL_StatusTypeDef TSL25911_Enable(void);
HAL_StatusTypeDef TSL25911_Disable(void);
HAL_StatusTypeDef TSL25911_SetGain(TSL25911_GAIN_t gain);
HAL_StatusTypeDef TSL25911_SetIntegrationTime(TSL25911_INTEGRATION_t integrationTime);
HAL_StatusTypeDef TSL25911_ReadLightData(TSL25911_LightData_t *lightData);
HAL_StatusTypeDef TSL25911_ReadID(uint8_t *id);

#endif // TSL25911_H

#ifndef ENS160_H
#define ENS160_H

#include "stm32f3xx_hal.h"

// ENS160 I2C address
#define ENS160_I2C_ADDRESS 0x53 << 1  // ADDR pin is high

// ENS160 Register Addresses
#define ENS160_PART_ID_ADDR        0x00  // Device Identity
#define ENS160_OPMODE_ADDR         0x10  // Operating Mode
#define ENS160_CONFIG_ADDR         0x11  // Interrupt Pin Configuration
#define ENS160_COMMAND_ADDR        0x12  // Additional System Commands
#define ENS160_TEMP_IN_ADDR        0x13  // Host Ambient Temperature Information
#define ENS160_RH_IN_ADDR          0x15  // Host Relative Humidity Information
#define ENS160_DEVICE_STATUS_ADDR  0x20  // Operating Mode
#define ENS160_DATA_AQI_ADDR       0x21  // Air Quality Index
#define ENS160_DATA_TVOC_ADDR      0x22  // TVOC Concentration
#define ENS160_DATA_ECO2_ADDR      0x24  // Equivalent CO2 Concentration
#define ENS160_DATA_T_ADDR         0x30  // Temperature used in calculations
#define ENS160_DATA_RH_ADDR        0x32  // Relative Humidity used in calculations
#define ENS160_DATA_MISR_ADDR      0x38  // Data Integrity Field
#define ENS160_GPR_WRITE_ADDR      0x40  // General Purpose Write Registers
#define ENS160_GPR_READ_ADDR       0x48  // General Purpose Read Registers

// ENS160 Register Sizes
#define ENS160_PART_ID_SIZE        2     // Size in bytes
#define ENS160_OPMODE_SIZE         1
#define ENS160_CONFIG_SIZE         1
#define ENS160_COMMAND_SIZE        1
#define ENS160_TEMP_IN_SIZE        2
#define ENS160_RH_IN_SIZE          2
#define ENS160_DEVICE_STATUS_SIZE  1
#define ENS160_DATA_AQI_SIZE       1
#define ENS160_DATA_TVOC_SIZE      2
#define ENS160_DATA_ECO2_SIZE      2
#define ENS160_DATA_T_SIZE         2
#define ENS160_DATA_RH_SIZE        2
#define ENS160_DATA_MISR_SIZE      1
#define ENS160_GPR_WRITE_SIZE      8
#define ENS160_GPR_READ_SIZE       8

typedef enum {
    ENS160_OPMODE_DEEP_SLEEP = 0x00,  // DEEP SLEEP mode (low-power standby)
    ENS160_OPMODE_IDLE       = 0x01,  // IDLE mode (low power)
    ENS160_OPMODE_STANDARD   = 0x02,  // STANDARD Gas Sensing Mode
    ENS160_OPMODE_RESET      = 0xF0   // RESET mode
} ENS160_OPMODE_t;

typedef struct __attribute__((packed)) {
    uint8_t INTEN   : 1;  // Bit 0: INTn pin enabled
    uint8_t INTDAT  : 1;  // Bit 1: INTn pin asserted for DATA registers
    uint8_t RESERVED1 : 1; // Bit 2: Reserved
    uint8_t INTGPR  : 1;  // Bit 3: INTn pin asserted for GPR registers
    uint8_t RESERVED2 : 1; // Bit 4: Reserved
    uint8_t INT_CFG : 1;  // Bit 5: INTn pin drive (0: Open drain, 1: Push/Pull)
    uint8_t INTPOL  : 1;  // Bit 6: INTn pin polarity (0: Active low, 1: Active high)
    uint8_t RESERVED3 : 1; // Bit 7: Reserved
} ENS160_CONFIG_t;

typedef enum {
    ENS160_COMMAND_NOP      = 0x00,  // No operation
    ENS160_COMMAND_GET_FWVER = 0x0E,  // Get firmware version
    ENS160_COMMAND_CLRGPR   = 0xCC   // Clear GPR read registers
} ENS160_COMMAND_t;

typedef struct __attribute__((packed)) {
    uint8_t version_major;   // GPR_READ4: Major version
    uint8_t version_minor;   // GPR_READ5: Minor version
    uint8_t version_release; // GPR_READ6: Release version
} ENS160_FirmwareVersion_t;

typedef enum {
    ENS160_VALIDITY_NORMAL_OPERATION = 0,  // Normal operation
    ENS160_VALIDITY_WARM_UP          = 1,  // Warm-Up phase
    ENS160_VALIDITY_INITIAL_STARTUP  = 2,  // Initial Start-Up phase
    ENS160_VALIDITY_INVALID_OUTPUT   = 3   // Invalid output
} ENS160_ValidityFlag_t;

typedef struct __attribute__((packed)) {
    uint8_t NEWGPR      : 1;  // Bit 0: High indicates new data available in GPR_READx registers
    uint8_t NEWDAT      : 1;  // Bit 1: High indicates new data available in DATA_x registers
    ENS160_ValidityFlag_t VALIDITY    : 2;  // Bits 2-3: Status (0: Normal, 1: Warm-Up, 2: Initial Start-Up, 3: Invalid output)
    uint8_t RESERVED    : 2;  // Bits 4-5: Reserved
    uint8_t STATER      : 1;  // Bit 6: High indicates an error detected
    uint8_t STATAS      : 1;  // Bit 7: High indicates an OPMODE is running
} ENS160_STATUS_t;

typedef enum {
    ENS160_AQI_EXCELLENT = 1,  // Excellent: No objections
    ENS160_AQI_GOOD      = 2,  // Good: No relevant objections, sufficient ventilation recommended
    ENS160_AQI_MODERATE  = 3,  // Moderate: Some objections, increased ventilation recommended
    ENS160_AQI_POOR      = 4,  // Poor: Major objections, intensified ventilation recommended
    ENS160_AQI_UNHEALTHY = 5   // Unhealthy: Situation not acceptable, intensified ventilation strongly recommended
} ENS160_AQI_UBA_t;

typedef struct __attribute__((packed)) {
    ENS160_AQI_UBA_t AQI_UBA   : 3;  // Bits 0-2: Air Quality Index according to UBA [1..5]
    uint8_t RESERVED  : 5;  // Bits 3-7: Reserved
} ENS160_AQI_t;

typedef enum {
    ENS160_CO2_EXCELLENT = 0,  // 400 - 600 ppm: Target
    ENS160_CO2_GOOD      = 1,  // 600 - 800 ppm: Average
    ENS160_CO2_FAIR      = 2,  // 800 - 1000 ppm: Optional ventilation
    ENS160_CO2_POOR      = 3,  // 1000 - 1500 ppm: Ventilation recommended
    ENS160_CO2_BAD       = 4   // >1500 ppm: Ventilation required
} ENS160_CO2_t;

typedef struct __attribute__((packed)) {
    uint8_t GPR_WRITE0;  // General Purpose WRITE Register 0 (0x40)
    uint8_t GPR_WRITE1;  // General Purpose WRITE Register 1 (0x41)
    uint8_t GPR_WRITE2;  // General Purpose WRITE Register 2 (0x42)
    uint8_t GPR_WRITE3;  // General Purpose WRITE Register 3 (0x43)
    uint8_t GPR_WRITE4;  // General Purpose WRITE Register 4 (0x44)
    uint8_t GPR_WRITE5;  // General Purpose WRITE Register 5 (0x45)
    uint8_t GPR_WRITE6;  // General Purpose WRITE Register 6 (0x46)
    uint8_t GPR_WRITE7;  // General Purpose WRITE Register 7 (0x47)
} ENS160_GPR_WRITE_t;

typedef struct __attribute__((packed)) {
    uint8_t GPR_READ0;  // General Purpose READ Register 0 (0x48)
    uint8_t GPR_READ1;  // General Purpose READ Register 1 (0x49)
    uint8_t GPR_READ2;  // General Purpose READ Register 2 (0x4A)
    uint8_t GPR_READ3;  // General Purpose READ Register 3 (0x4B)
    uint8_t GPR_READ4;  // General Purpose READ Register 4 (0x4C)
    uint8_t GPR_READ5;  // General Purpose READ Register 5 (0x4D)
    uint8_t GPR_READ6;  // General Purpose READ Register 6 (0x4E)
    uint8_t GPR_READ7;  // General Purpose READ Register 7 (0x4F)
} ENS160_GPR_READ_t;

// Structure to hold ENS160 air quality data
typedef struct __attribute__((packed)) {
    uint8_t AQI;       // Air Quality Index
    uint16_t TVOC;     // Total Volatile Organic Compounds in ppb
    uint16_t eCO2;     // Equivalent CO2 in ppm
} ENS160_Data_t;

// Structure for all registers
typedef struct __attribute__((packed)) {
   uint16_t PART_ID; // (Address 0x00)
   ENS160_OPMODE_t OPMODE; // (Address 0x10)
	ENS160_CONFIG_t CONFIG; // (Address 0x11)
	ENS160_COMMAND_t COMMAND; // (Address 0x12)
	uint16_t TEMP_IN; //TEMP_IN (Address 0x13)
	uint16_t RH_IN; //TEMP_IN (Address 0x15)
	ENS160_STATUS_t STATUS; //TEMP_IN (Address 0x20)
	ENS160_AQI_t AQI; //TEMP_IN (Address 0x21) 
	uint16_t TVOC; // DATA_TVOC (Address 0x22)
	uint16_t CO2; // DATA_ECO2 (Address 0x24)
	uint16_t ETHANOL; // DATA_ETOH (Address 0x22)
	uint16_t TEMP; // DATA_T (Address 0x30)
	uint16_t RH; // DATA_RH (Address 0x32)
	uint8_t MISR; // DATA_MISR (Address 0x38)
	ENS160_GPR_WRITE_t GPR_WRITE; // GPR_WRITE (Address 0x40)
	ENS160_GPR_READ_t GPR_READ; // GPR_WRITE (Address 0x48)
} ENS160_Registers_t;

// Function prototypes
HAL_StatusTypeDef ENS160_Init(void);
HAL_StatusTypeDef ENS160_UpdateEnvInputs(float temperatureC, float humidityPercent);
HAL_StatusTypeDef ENS160_ReadAllData(ENS160_Data_t *data);
HAL_StatusTypeDef ENS160_ReadAllRegisters(ENS160_Registers_t *registers);
HAL_StatusTypeDef ENS160_WriteRegister(uint8_t reg, uint8_t *pData, uint16_t size);
HAL_StatusTypeDef ENS160_SetOperatingMode(ENS160_OPMODE_t mode);
HAL_StatusTypeDef ENS160_ReadAQI(ENS160_AQI_t *aqi);
HAL_StatusTypeDef ENS160_ReadTVOC(uint16_t *tvoc);
HAL_StatusTypeDef ENS160_ReadCO2(uint16_t *eco2);

#endif // ENS160_H
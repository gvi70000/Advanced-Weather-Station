#ifndef ENS160_H
#define ENS160_H

#include "stm32f3xx_hal.h"

/** @brief ENS160 I2C Address */
#define ENS160_I2C_ADDRESS 0x53 << 1  // ADDR pin is high

/** @brief ENS160 PART ID */
#define ENS160_PART_ID 0x0160

/** @brief ENS160 Register Addresses */
#define ENS160_PART_ID_ADDR        0x00  /**< Device Identity */
#define ENS160_OPMODE_ADDR         0x10  /**< Operating Mode */
#define ENS160_CONFIG_ADDR         0x11  /**< Interrupt Pin Configuration */
#define ENS160_COMMAND_ADDR        0x12  /**< Additional System Commands */
#define ENS160_TEMP_IN_ADDR        0x13  /**< Host Ambient Temperature Information */
#define ENS160_RH_IN_ADDR          0x15  /**< Host Relative Humidity Information */
#define ENS160_DEVICE_STATUS_ADDR  0x20  /**< Device Status */
#define ENS160_DATA_AQI_ADDR       0x21  /**< Air Quality Index */
#define ENS160_DATA_TVOC_ADDR      0x22  /**< TVOC Concentration */
#define ENS160_DATA_ECO2_ADDR      0x24  /**< Equivalent CO2 Concentration */
#define ENS160_DATA_T_ADDR         0x30  /**< Temperature used in calculations */
#define ENS160_DATA_RH_ADDR        0x32  /**< Relative Humidity used in calculations */
#define ENS160_DATA_MISR_ADDR      0x38  /**< Data Integrity Field */
#define ENS160_GPR_WRITE_ADDR      0x40  /**< General Purpose Write Registers */
#define ENS160_GPR_READ_ADDR       0x48  /**< General Purpose Read Registers */

/** @brief ENS160 Register Sizes */
#define ENS160_PART_ID_SIZE        2  /**< Size of PART_ID register (bytes) */
#define ENS160_OPMODE_SIZE         1  /**< Size of OPMODE register (bytes) */
#define ENS160_CONFIG_SIZE         1  /**< Size of CONFIG register (bytes) */
#define ENS160_COMMAND_SIZE        1  /**< Size of COMMAND register (bytes) */
#define ENS160_TEMP_IN_SIZE        2  /**< Size of TEMP_IN register (bytes) */
#define ENS160_RH_IN_SIZE          2  /**< Size of RH_IN register (bytes) */
#define ENS160_DEVICE_STATUS_SIZE  1  /**< Size of STATUS register (bytes) */
#define ENS160_DATA_AQI_SIZE       1  /**< Size of AQI data register (bytes) */
#define ENS160_DATA_TVOC_SIZE      2  /**< Size of TVOC data register (bytes) */
#define ENS160_DATA_ECO2_SIZE      2  /**< Size of eCO2 data register (bytes) */
#define ENS160_DATA_T_SIZE         2  /**< Size of calculated temperature data (bytes) */
#define ENS160_DATA_RH_SIZE        2  /**< Size of calculated relative humidity data (bytes) */
#define ENS160_DATA_MISR_SIZE      1  /**< Size of data integrity field (bytes) */
#define ENS160_GPR_WRITE_SIZE      8  /**< Size of GPR write registers (bytes) */
#define ENS160_GPR_READ_SIZE       8  /**< Size of GPR read registers (bytes) */

/** @brief ENS160 Operating Modes */
typedef enum {
    ENS160_OPMODE_DEEP_SLEEP = 0x00, /**< DEEP SLEEP mode (low-power standby) */
    ENS160_OPMODE_IDLE       = 0x01, /**< IDLE mode (low power) */
    ENS160_OPMODE_STANDARD   = 0x02, /**< STANDARD Gas Sensing Mode */
    ENS160_OPMODE_RESET      = 0xF0  /**< RESET mode */
} ENS160_OPMODE_t;

/** @brief ENS160 Configuration Register Fields */
typedef struct __attribute__((packed)) {
    uint8_t INTEN			: 1; /**< Bit 0: Interrupt enabled */
    uint8_t INTDAT		: 1; /**< Bit 1: Interrupt asserted for DATA registers */
    uint8_t RESERVED1	: 1; /**< Bit 2: Reserved */
    uint8_t INTGPR		: 1; /**< Bit 3: Interrupt asserted for GPR registers */
    uint8_t RESERVED2	: 1; /**< Bit 4: Reserved */
    uint8_t INT_CFG		: 1; /**< Bit 5: INTn pin drive (0: Open drain, 1: Push/Pull) */
    uint8_t INTPOL		: 1; /**< Bit 6: INTn pin polarity (0: Active low, 1: Active high) */
    uint8_t RESERVED3	: 1; /**< Bit 7: Reserved */
} ENS160_CONFIG_t;

/** @brief ENS160 Command Set */
typedef enum {
    ENS160_COMMAND_NOP				= 0x00, /**< No operation */
    ENS160_COMMAND_GET_FWVER	= 0x0E, /**< Get firmware version */
    ENS160_COMMAND_CLRGPR			= 0xCC  /**< Clear GPR read registers */
} ENS160_COMMAND_t;

/** @brief ENS160 Status Register Fields */
typedef struct __attribute__((packed)) {
    uint8_t NEWGPR      : 1; /**< Bit 0: New data in GPR_READ registers */
    uint8_t NEWDAT      : 1; /**< Bit 1: New data in DATA registers */
    uint8_t VALIDITY    : 2; /**< Bits 2-3: Status validity */
    uint8_t RESERVED    : 2; /**< Bits 4-5: Reserved */
    uint8_t STATER      : 1; /**< Bit 6: Error detected */
    uint8_t STATAS      : 1; /**< Bit 7: Operating mode is running */
} ENS160_STATUS_t;


/** @brief Structure to hold ENS160 air quality data */
typedef struct __attribute__((packed)) {
    union {
        uint8_t Buffer[5];	/**< Raw data buffer for direct I2C read */
        struct __attribute__((packed)) {
            uint8_t AQI;		/**< Air Quality Index (1 byte) */
            uint16_t TVOC;	/**< Total Volatile Organic Compounds (2 bytes) */
            uint16_t eCO2;	/**< Equivalent CO2 concentration (2 bytes) */
        } Fields;
    };
} ENS160_Data_t;

/** @brief ENS160 Register Map Structure */
typedef struct __attribute__((packed)) {
    uint16_t PART_ID;         /**< PART_ID register (2 bytes) */
    ENS160_OPMODE_t OPMODE;		/**< Operating Mode register (1 byte) */
    ENS160_CONFIG_t CONFIG;   /**< Configuration register (1 byte) */
    ENS160_COMMAND_t COMMAND;	/**< Command register (1 byte) */
    uint16_t TEMP_IN;         /**< Host Ambient Temperature (2 bytes) */
    uint16_t RH_IN;           /**< Host Relative Humidity (2 bytes) */
    ENS160_STATUS_t STATUS;   /**< Device Status register (1 byte) */
    ENS160_Data_t DATA;				/**< Air Quality Index (1 byte) */
															/**< TVOC Concentration (2 bytes) */
															/**< Equivalent CO2 Concentration (2 bytes) */
    uint16_t DATA_T;          /**< Temperature used in calculations (2 bytes) */
    uint16_t DATA_RH;         /**< Relative Humidity used in calculations (2 bytes) */
    uint8_t DATA_MISR;        /**< Data Integrity Field (1 byte) */
    uint8_t GPR_WRITE[8];     /**< General Purpose Write Registers (8 bytes) */
    uint8_t GPR_READ[8];      /**< General Purpose Read Registers (8 bytes) */
} ENS160_Registers_t;

/** @brief Function Prototypes */
HAL_StatusTypeDef ENS160_Init(void);
HAL_StatusTypeDef ENS160_UpdateEnvInputs(float temperatureC, float humidityPercent);
HAL_StatusTypeDef ENS160_ReadAllData(ENS160_Data_t *data);
HAL_StatusTypeDef ENS160_ReadAllRegisters(ENS160_Registers_t *registers);
HAL_StatusTypeDef ENS160_SetOperatingMode(ENS160_OPMODE_t mode);

#endif // ENS160_H

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "BMP581.h"
#include "HDC302x.h"

#include "AS3935.h"
#include "AS7331.h"
#include "ENS160.h"
#include "TCS34717.h"
#include "TSL25911.h"
#include "PGA460.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
	// Constants
	#define R_D 287.05									// Specific gas constant for dry air (J/kg·K)
	#define R_V 461.495									// Specific gas constant for water vapor (J/kg·K)
	#define GAMMA 1.4										// Adiabatic index for air
	#define L 0.0065										// Temperature lapse rate (K/m)
	#define T0 288.15										// Standard temperature at sea level (K)
	#define P0 101325.0									// Standard pressure at sea level (Pa)
	#define G 9.80665										// Gravitational acceleration (m/s²)
	#define M 0.0289644									// Molar mass of air (kg/mol)
	#define R 8.3144598									// Universal gas constant (J/(mol·K))
	#define KELVIN_OFFSET 273.15				// Conversion from Celsius to Kelvin
	#define RH_DIVISOR 100.0						// Converts RH percentage to a fraction
	#define WATER_VAPOR_EFFECT	0.6077	// Effect of water vapor on speed of sound
	#define SATURATION_CONSTANT 6.1078	// Constant for saturation vapor pressure calculation

	// Reflection Path Length
	#define DISTANCE 0.34308						// Total reflection path length in meters
	#define US_TO_SEC 1e-6							// Conversion factor from microseconds to seconds
	
	#ifndef M_PI
		#define M_PI 3.14159265358979323846
	#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
		HAL_StatusTypeDef status;
		volatile uint8_t HDC3020_1_Ready, AS3935_Ready, BMP581_Ready, HDC3020_2_Ready, ENS160_Ready, AS7331_Ready, TSL25911_Ready, TCS34717_Ready;
		BMP581_sensor_data_t BMP581_Data;

		HDC302x_t hdc3020_sensor_1; // Sensor connected to 0x44
		HDC302x_t hdc3020_sensor_2; // Sensor connected to 0x45
		HDC302x_Data_t HDC302x_Data_1;
		HDC302x_Data_t HDC302x_Data_2;
		
		
    double height = 500.0;   // Height in meters
    double T = 20.0;         // Temperature in °C
    double RH = 50.0;        // Relative Humidity in %
    double P = 95460.94;     // Pressure in Pa
		
    uint32_t ToF_up[3] = {0};
    uint32_t ToF_down[3] = {0};
    double wind_speed = 0.0, wind_direction = 0.0;

    uint8_t dataBuffer[4];
		
typedef struct __attribute__((packed)) {
    union {
        uint8_t Value;
        struct __attribute__((packed)) {
            uint8_t BMP        : 1;  // Bit 0 indicates BMP581 data availability
            uint8_t HDC2       : 1;  // Bit 1 indicates first HDC3020 data availability
            uint8_t HDC1       : 1;  // Bit 2 indicates second HDC3020 data availability
            uint8_t RESERVED   : 5;  // Bits 3 to 7 reserved for future use
        } BitField;
    } Status; // Status bitfield indicating sensor data availability

    float Temperature;								// Combined/average temperature (°C)
    float RH;													// Combined/average relative humidity (%)
    float Pressure;										// Pressure (Pa)
		float SoundSpeed;									//Calculated souns spee in air for specific conditions
		AS3935_Distance_t StormDistance;	// Distance to storm, if any 
		ENS160_Data_t Air;								// Air quality data
		AS7331_UVData_t UV_Data;					//UVA, UVB and UVC
		TSL25911_LightData_t LightData;		//
		TCS34717_CRGB_t CRGB;							// Clear, Red, Green Blue light
} EnvData_t;

EnvData_t envData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// Function to calculate the speed of sound
void calculateSpeedOfSound(double height) {
    double T_k = envData.Temperature + KELVIN_OFFSET;

    float P_total = envData.Pressure;
    if (height != 0) {
        if (height > 0) {
            P_total = P0 * pow((1 - (L * height) / T0), (G * M) / (R * L));
        } else {
            P_total = P0 * pow((1 + (L * fabs(height)) / T0), (G * M) / (R * L));
        }
    }

    double P_sat = SATURATION_CONSTANT * pow(10, (7.5 * envData.Temperature) / (envData.Temperature + 237.3)) * 100;
    double P_v = P_sat * (envData.RH / RH_DIVISOR);  // Partial pressure of water vapor
    double P_d = P_total - P_v;             // Partial pressure of dry air
    double H = P_v / P_d;

    envData.SoundSpeed = sqrt(GAMMA * R_D * T_k * (1 + WATER_VAPOR_EFFECT * H));
}

// Function to calculate wind speed and direction
void calculateWind(uint32_t ToF_up[3], uint32_t ToF_down[3], double speed_of_sound,
                   double *wind_speed, double *wind_direction) {
    double delta_t[3] = {0.0}, avg_t[3] = {0.0};
    double v_x = 0.0, v_y = 0.0;

    for (int i = 0; i < 3; i++) {
        double ToF_up_sec = ToF_up[i] * US_TO_SEC;
        double ToF_down_sec = ToF_down[i] * US_TO_SEC;

        delta_t[i] = ToF_down_sec - ToF_up_sec;
        avg_t[i] = (ToF_up_sec + ToF_down_sec) / 2;

        if (fabs(avg_t[i]) < 1e-9) {
            printf("Error: avg_t[%d] is too small!\n", i);
            *wind_speed = 0.0;
            *wind_direction = 0.0;
            return;
        }
    }

    v_x = (DISTANCE / 2.0) * (delta_t[0] / avg_t[0]);
    v_y = (DISTANCE / 2.0) * (delta_t[1] / avg_t[1]);

    *wind_speed = sqrt(v_x * v_x + v_y * v_y);
    *wind_direction = atan2(v_y, v_x) * (180.0 / M_PI);
    if (*wind_direction < 0) {
        *wind_direction += 360.0;
    }
}
									 
// Interrupt callback function for BMP581 and BNO086
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == INT1_HDC3020_Pin) { //PA7
        HDC3020_1_Ready = 1;
    }
		if(GPIO_Pin == INT_AS3935_Pin) { //PB0
        AS3935_Ready = 1;
    }
    if(GPIO_Pin == BMP_INT_Pin) { //PB1
        BMP581_Ready = 1;
    }
    if(GPIO_Pin == INT2_HDC3020_Pin) { //PB2
        HDC3020_2_Ready = 1;
    }
    if(GPIO_Pin == INT_ES160_Pin) { //PB12
        ENS160_Ready = 1;
    }
    if(GPIO_Pin == INT_AS7331_Pin) { //PB14
        AS7331_Ready = 1;
    }
    if(GPIO_Pin == INT_TSL25911_Pin) { //PB15
        TSL25911_Ready = 1;
    }
    if(GPIO_Pin == INT_TCS34717_Pin) { //PC6
        TCS34717_Ready = 1;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Sensors on main board
//* 01. AMS AS7331				UVA, UVB and UVC			connected on I2C2 with Interrupt on PB14		0xE8 = 0x74 << 1
//* 02. AMS TSL25911FN		Ambient light					connected to I2C2 with Interrupt on PB15		0x52 = 0x29 << 1
//* 03. AMS TCS34717FN		Color/clear light			connected to I2C1 with Interrupt on PC6			0x52 = 0x29 << 1
//* 04. ScioSense ENS160	Multi-Gas Sensor			connected to I2C2 with Interrupt on PB12		0xA6 = 0x53 << 1
//* 05. ScioSense AS3935	Lightning Detector		connected to I2C2 with Interrupt on PB0			0x06 = 0x03 << 1
// Sensors on flex board
//* 06. Bosch BMP581	Pressure Temperature			connected to I2C2 with Interrupt on PB1		0x8C = 0x46 << 1
//* 07. TI HDC3020	Temperature	RH							connected to I2C2 with Interrupt on PA7		0x88 = 0x44 << 1
//* 08. TI HDC3020	Temperature	RH							connected to I2C2 with Interrupt on PB2		0x8A = 0x45 << 1
// 09. TI PGA460 Ultrasonic signal processor	connected to USART1 with IO on PC13
// 10. TI PGA460 Ultrasonic signal processor	connected to USART5 with IO on PB5
// 11. TI PGA460 Ultrasonic signal processor	connected to UART4  with IO on PA15
// 12. RTK GPS Quectel LG290P									connected to USART3 with IO on PA15

// 13. Olimex ESP32-PoE2											connected to USART2 with O on PA4 and INT on PA5

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
		envData.Status.Value = 0;
		envData.Temperature = 0.0;
		envData.Pressure = 0.0;
		envData.RH = 0.0;
    if (BMP581_Init() == HAL_OK) {
        printf("BMP581 initialized successfully.\n");
    } else {
        printf("Failed to initialize BMP581.\n");
        Error_Handler();
    }
    // Initialize Sensor 1 (I2C address: 0x44 << 1)
    status = HDC302x_Init(&hdc3020_sensor_1, HDC302X_SENSOR_1_ADDR);
    if (status == HAL_OK) {
        printf("HDC3020 Sensor 1 (0x44) initialized successfully.\n");
    } else {
        printf("Failed to initialize HDC3020 Sensor 1 (0x44).\n");
        Error_Handler();
    }

    // Initialize Sensor 2 (I2C address: 0x45 << 1)
    status = HDC302x_Init(&hdc3020_sensor_2, HDC302X_SENSOR_2_ADDR);
    if (status == HAL_OK) {
        printf("HDC3020 Sensor 2 (0x45) initialized successfully.\n");
    } else {
        printf("Failed to initialize HDC3020 Sensor 2 (0x45).\n");
        Error_Handler();
    }
    // Initialize AS3935
    if (AS3935_Init() != HAL_OK) {
        printf("AS3935 initialization failed.\n");
        Error_Handler();
    } else {
        printf("AS3935 initialized successfully.\n");
    }
    // Initialize ENS160
    if (ENS160_Init() != HAL_OK) {
        printf("ENS160 initialization failed.\n");
        Error_Handler();
    } else {
        printf("ENS160 initialized successfully.\n");
    }
    // Initialize AS7331
    if (AS7331_Init() != HAL_OK) {
        printf("AS7331 initialization failed.\n");
        Error_Handler();
    }

	TCS34717_Enable();
    // Initialize TSL25911 for outdoor sunlight use
    if (TSL25911_Init() != HAL_OK) {
        printf("TSL25911 initialization failed.\n");
        Error_Handler();
    } else {
        printf("TSL25911 initialized successfully.\n");
    }
	PGA460_Init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // If we have received data from all sensor (2x HDC3020 + BMP581)
		if(envData.Status.Value == 7){
			envData.Temperature = (BMP581_Data.temperature + HDC302x_Data_1.Temperature + HDC302x_Data_2.Temperature)/3.0;
			envData.RH = (HDC302x_Data_1.Humidity + HDC302x_Data_2.Humidity)/2.0;
			ENS160_UpdateEnvInputs(envData.Temperature, envData.RH);
			envData.Pressure = BMP581_Data.pressure;
			calculateSpeedOfSound(height);
			printf("Speed of Sound: %.2f m/s\n", envData.SoundSpeed);
			
			for (int i = 0; i < 3; i++) {
					if (PGA460_GetUltrasonicMeasurement(i, dataBuffer, 1) == HAL_OK) {
							ToF_up[i] = (dataBuffer[0] << 24 | dataBuffer[1] << 16 | dataBuffer[2] << 8 | dataBuffer[3]);
					}
					if (PGA460_GetUltrasonicMeasurement(i, dataBuffer, 1) == HAL_OK) {
							ToF_down[i] = (dataBuffer[0] << 24 | dataBuffer[1] << 16 | dataBuffer[2] << 8 | dataBuffer[3]);
					}
			}

			calculateWind(ToF_up, ToF_down, envData.SoundSpeed, &wind_speed, &wind_direction);

			printf("Wind Speed: %.2f m/s\n", wind_speed);
			printf("Wind Direction: %.2f degrees\n", wind_direction);
			envData.Status.Value = 0;
		}
		if(BMP581_Ready){
			BMP581_Ready = 0;
			BMP581_Get_TempPressData(&BMP581_Data); 
			envData.Status.BitField.BMP = 1;
		}
    if(HDC3020_1_Ready){
			HDC3020_1_Ready = 0;
			HDC302x_ReadData(&hdc3020_sensor_1, &HDC302x_Data_1); 
			envData.Status.BitField.HDC1 = 1;
		}
    if(HDC3020_2_Ready){
			HDC3020_2_Ready = 0;
			HDC302x_ReadData(&hdc3020_sensor_2, &HDC302x_Data_2); 
			envData.Status.BitField.HDC2 = 1;
		}
		if(AS3935_Ready){
			AS3935_Ready = 0;
			envData.StormDistance = AS3935_GetDistanceToStorm();
        uint8_t intVal = AS3935_ReadInterruptStatus();

        switch (intVal) {
            case INT_NH:  // Noise level too high
                printf("AS3935: Noise detected.\n");
                break;

            case INT_D:  // Disturber detected
                printf("AS3935: Disturber detected.\n");
                break;

            case INT_L:  // Lightning detected
                printf("AS3935: Lightning strike detected!\n");

                // Get distance to the storm
                uint8_t distance = AS3935_GetDistanceToStorm();
                printf("AS3935: Distance to storm: %d km.\n", distance);

                // Get lightning energy
                uint32_t energy = AS3935_GetLightningEnergy();
                printf("AS3935: Lightning energy: %d.\n", energy);
                break;

            default:
                printf("AS3935: Unknown interrupt.\n");
                break;
        }
		}
		if(ENS160_Ready) {
			ENS160_Ready = 0;
			ENS160_ReadAllData(&envData.Air);
		}
		if(AS7331_Ready) {
			AS7331_Ready = 0;
			AS7331_ReadUVData(&envData.UV_Data);
		}
		if(TSL25911_Ready) {
			TSL25911_Ready = 0;
			TSL25911_ReadLightData(&envData.LightData);
		}
		if(TCS34717_Ready) {
			TCS34717_Ready = 0;
			TCS34717_ReadRGBCData(&envData.CRGB);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

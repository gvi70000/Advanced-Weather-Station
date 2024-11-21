# Advanced-Weather-Station
Advanced Weather Station with High-Precision Sensors, GNSS RTK, and Ultrasonic Wind Measurement
Advanced Weather Station with High-Precision Sensors, GNSS RTK, and Ultrasonic Wind Measurement

Please read Liability Disclaimer and License Agreement CAREFULLY

This article introduces an advanced weather station equipped with cutting-edge sensors for comprehensive environmental monitoring. The system is built around the STM32F302RBTx microcontroller, which processes data from an array of sensors, and the Olimex ESP32-PoE2, which handles data transmission over Ethernet with Power over Ethernet (PoE) support. With detailed KiCAD project files and STEP models, this weather station can be easily replicated and adapted for various applications.

System Overview
The weather station is designed for high precision and reliability, leveraging a combination of environmental sensors for ultraviolet radiation, light intensity, air quality, temperature, humidity, pressure, lightning detection, and wind measurement. These sensors are processed by the STM32F302RBTx, which aggregates the data and transmits it to a server for real-time monitoring and analysis.

![Weather-Station-PCB-Top](https://github.com/user-attachments/assets/cb2f782f-c209-4e87-9f64-742537d4fa33)

![Weather-Station-PCB-Bottom](https://github.com/user-attachments/assets/3a2af3d6-6282-4d53-a5e3-fd0b3560f18d)

Key Features
Comprehensive Sensor Suite:

Measures a wide range of environmental parameters, including UV radiation, ambient light, air quality, temperature, humidity, pressure, wind speed, and direction.
Detects lightning activity and provides GNSS-based precise location data.
STM32F302RBTx Microcontroller:

Acts as the primary processing unit, handling data acquisition and preprocessing from all sensors.
Ensures reliable communication with the ESP32-PoE2 for data transmission.
Olimex ESP32-PoE2:

Handles Ethernet communication with PoE for power and connectivity.
Transmits processed sensor data to the network for further analysis.
Durability and Precision:

Optical sensors are protected by optical glass plates, see AliExpress.
Ultrasonic transducers are using stainless steel discs as reflectors for enhanced durability, see AliExpress.
Sensors and Modules
Optical Sensors (Protected by Optical Glass)
AMS AS7331 (UVA, UVB, UVC):

Connection: I2C2 (Interrupt on PB14, Address: 0xE8).
Use Case: Tracks ultraviolet radiation levels for health and safety monitoring.
AMS TSL25911FN (Ambient Light):

Connection: I2C2 (Interrupt on PB15, Address: 0x52).
Use Case: Monitors daylight intensity for solar energy and weather prediction.
AMS TCS34717FN (Color and Clear Light):

Connection: I2C1 (Interrupt on PC6, Address: 0x52).
Use Case: Provides spectral data for atmospheric clarity and color analysis.
Gas and Lightning Detection
ScioSense ENS160 (Air Quality):

Connection: I2C2 (Interrupt on PB12, Address: 0xA6).
Use Case: Detects CO2, VOCs, and other gases for air quality assessment.
ScioSense AS3935 (Lightning Detector):

Connection: I2C2 (Interrupt on PB0, Address: 0x06).
Use Case: Detects storm activity for early warning systems.
Environmental Monitoring
Bosch BMP581 (Pressure and Temperature):

Connection: I2C2 (Interrupt on PB1, Address: 0x8C).
Use Case: Tracks atmospheric pressure and temperature for weather forecasting.
TI HDC3020 (x2 - Temperature and Humidity):

Sensor 1: I2C2 (Interrupt on PA7, Address: 0x88).
Sensor 2: I2C2 (Interrupt on PB2, Address: 0x8A).
Use Case: Measures ambient temperature and relative humidity for comfort analysis.
![Pressure-Temperature-RH](https://github.com/user-attachments/assets/0ede641f-0e14-4ad0-a5df-11ed8c95885f)
Ultrasonic Wind Measurement
TI PGA460 (Ultrasonic Signal Processors):
Sensors connected to:
USART1 (IO on PC13).
USART5 (IO on PB5).
UART4 (IO on PA15).
Ultrasonic reflector 304 stainless steel discs (40mm diameter) for robustness.
Ultrasonic transmitter/Receiver UTR-1440K-TT-R
Use Case: Measures wind speed and direction with high precision.
GNSS RTK Module
Quectel LG290P GNSS Module:
Connection: USART3 (IO on PA15).
Uses RTK GPS Antenna for Beidou GLONASS GALILEO see AliExpress.
Supports multi-frequency L1, L2, L5 for sub-centimeter positioning accuracy.
Use Case: Provides precise location data for georeferenced weather measurements.
Communication Module
Olimex ESP32-PoE2:
Connection: USART2 (IO on PA4, Interrupt on PA5).
PoE-enabled for simultaneous data transmission and power delivery.
Use Case: Sends weather data to a central server or cloud for real-time analysis and visualization.
Mechanical and Electrical Design
Optical Protection:

Optical sensors are covered with 25.4x25.4x2mm optical UV transparent glass plates to prevent damage from environmental factors like dust and rain.
Durable Ultrasonic Sensor Mount:

Ultrasonic transducers are mounted on 3D printed enclosure from ASA made by JLCPCB, the reflector plates are 40mm diameter 304 stainless steel discs, ensuring stability and accuracy in wind measurements.
Precise Data Acquisition:

STM32F302RBTx microcontroller aggregates sensor data, compensates for temperature variations, and ensures synchronization across all inputs.
High-Precision Components:

Sensors are individually calibrated, and temperature compensation algorithms are applied to minimize error.
Workflow
Data Acquisition:

Sensors collect data on UV radiation, light intensity, air quality, temperature, humidity, pressure, wind speed, and lightning activity.
GNSS RTK provides high-precision location data.
Processing:

The STM32F302RBTx processes raw data, applies calibration and compensation algorithms, and packages the information for transmission.
Transmission:

The ESP32-PoE2 transmits processed data over Ethernet to a server or cloud platform for real-time visualization.
Files Provided
KiCAD Project Files: Include detailed schematics, PCB layout, and bill of materials for easy replication download here and here.  
STEP Models: Provide 3D models of the weather station for integration into larger systems or enclosures - download here.
Applications
Meteorological Research:

Collects accurate data for climate studies and weather forecasting.
Agricultural Monitoring:

Tracks environmental conditions to optimize crop growth and prevent damage from extreme weather.
Smart Cities:

Integrates into urban networks for monitoring air quality and weather conditions.
Renewable Energy:

Supports solar tracking and wind energy applications with precise environmental measurements.
Advantages
High Precision: GNSS RTK and advanced sensors ensure accuracy.
Durable Design: Optical glass and stainless steel components protect sensitive equipment.
Scalability: Modular design allows easy customization and upgrades.
Real-Time Data: PoE and Ethernet enable seamless network integration.
This weather station is a versatile and reliable solution for real-time environmental monitoring. With its combination of robust design, high-precision sensors, and advanced processing, it is ideal for a wide range of applications, from scientific research to smart city infrastructure. Detailed KiCAD and STEP files make this project accessible for further development and customization.

![Weather-Station](https://github.com/user-attachments/assets/37c6c1e1-a923-4c17-a6b9-b05491661c2b)


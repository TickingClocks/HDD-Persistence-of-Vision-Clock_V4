/*
 * NCT175.h
 *
 *  Created on: Aug 18, 2022
 *      Author: Kadin Whiting
 *
 *  The NTC175 is a PCB temperature sensor. The sensor has a temperature range of -55°C to +125°C
 *
 *  The sensor communicates over I2C. The sensor has 3 address setting pins. The I2C address of the
 *  device is determined by how these pins are connected.
 *
 *  There are 3 NTC175 temperature sensors in the HDD Clock V4.0.
 *  	1. Measuring the PCB temperature of the LED board (display)
 *  	2. Measuring the PCB temperature of the driver board near the buck converter circuit
 *  	3. Measuring the ambient air temperature (V4.0 this sensor was routed wrong and instead gets the
 *  											 (PCB temperature near the buttons)
 */


#ifndef SRC_NCT175_H_
#define SRC_NCT175_H_

#include "main.h"

extern I2C_HandleTypeDef 			hi2c1;

//define the 3 sensor I2C addresses on the HDDCLK V4.0
#define driverBoardSensorAddress  	  0x48 << 1 //Driver Board PCB Temperature
#define ambientAirSensorAddress  	    0x49 << 1 //Ambient Air Temperature (V4.0 PCB is routed poorly for ambient air)
#define LEDBoardSensorAddress  		    0x4C << 1 //LED Board PCB Temperature

//define the register addresses in the NCT175 temperature sensor
#define tempRegister 				          0x00 //temperature register
#define NCT175_configRegister 		    0x01 //configuration register
#define THYSTRegister				          0x02 //temperature hysteresis register
#define TOSRegister					          0x03 //temperature over temperature register
#define oneShotRegister				        0x04 //one shot register

/*
 * DEFAULT CONFIG REG SETTINGS:
 *  One Shot mode is off - temperature is read every 60ms
 *  The OS pin will trigger when OS temperature is exceeded 1 time
 *  OS pin is active LOW
 *  Comparator mode selected
 *  Normal power mode
 */
#define NCT175_configRegVal			      0x00 //0x00=default
#define	THYSTRegVal					          0x4B00 //default (75°C)
#define TOSRegVal					            0x5000 //default (80°C)

#define NCT175_I2C_Delay			        100

//function prototypes
void NCT175_Init(uint8_t sensor);
void NCT175_write8BitRegister(uint8_t sensorAddress, uint8_t NCT175Register, uint8_t byte);
void NCT175_write16BitRegister(uint8_t sensorAddress, uint8_t NCT175Register, uint16_t twoBytes);
uint8_t NCT175_read8BitRegister(uint8_t sensorAddress, uint8_t NCT175Register);
uint16_t NCT175_read16BitRegister(uint8_t sensorAddress, uint8_t NCT175Register);
float NCT175_readTemperature(uint8_t sensor, uint8_t units);
float NCT175_CtoF(float C);

#endif /* SRC_NCT175_H_ */


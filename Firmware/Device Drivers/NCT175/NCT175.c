/*
 * NCT175.c
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

#include "NCT175.h"

/*
 * @Summary	Writes data to the 8 bit registers (configuration and one-shot registers)
 * @Inputs	NCT175Register - desired register to be written to
 * 			byte - data to write to the register
 * @Return	None
 */
void NCT175_write8BitRegister(uint8_t sensorAddress, uint8_t NCT175Register, uint8_t byte){
	uint8_t buf[2] = {NCT175Register, byte};
	HAL_I2C_Master_Transmit(&hi2c1, sensorAddress, buf, 2, NCT175_I2C_Delay);
}

/*
 * @Summary	Writes data to the 16 bit registers (THYST and TOS registers)
 * @Inputs	NCT175Register - desired register to be written to
 * 			twoBytes - data to write to the register
 * @Return	None
 */
void NCT175_write16BitRegister(uint8_t sensorAddress, uint8_t NCT175Register, uint16_t twoBytes){
	uint8_t buf[3] = {NCT175Register, twoBytes>>8, twoBytes};
	HAL_I2C_Master_Transmit(&hi2c1, sensorAddress, buf, 3, NCT175_I2C_Delay);
}

/*
 * @Summary	Reads the value from an 8 bit register
 * @Inputs	NC175Register - The desired register to read from
 * @Return	The 8 bit data found on the requested register
 */
uint8_t NCT175_read8BitRegister(uint8_t sensorAddress, uint8_t NCT175Register){
	uint8_t registerValue;
	HAL_I2C_Master_Transmit(&hi2c1, sensorAddress, &NCT175Register, 1, NCT175_I2C_Delay);
	HAL_I2C_Master_Receive(&hi2c1, sensorAddress, &registerValue, 1, NCT175_I2C_Delay);
	return registerValue;
}

/*
 * @Summary	Reads the value from a 16 bit register
 * @Inputs	NCT175Register - The desired register to read from
 * @Return	the 16 bit data found on the requested register
 */
uint16_t NCT175_read16BitRegister(uint8_t sensorAddress, uint8_t NCT175Register){
	uint8_t buf[2] = {0x00, 0x00};
	HAL_I2C_Master_Transmit(&hi2c1, sensorAddress, &NCT175Register, 1, NCT175_I2C_Delay);
	HAL_I2C_Master_Receive(&hi2c1, sensorAddress, &buf[0], 2, NCT175_I2C_Delay);
	return (buf[0]<<8)+buf[1];
}


/*
 * @Summary	Initializes the NCT175 temperature sensor with the desired settings
 * 			specifically for the HDDCLK V4.0 project
 * @Inputs	None
 * @return	None
 */
void NCT175_Init(uint8_t sensor){
	uint8_t sensorAddress;
	switch (sensor){
	case 1:
		sensorAddress = driverBoardSensorAddress;
		break;
	case 2:
		sensorAddress = ambientAirSensorAddress;
		break;
	case 3:
		sensorAddress = LEDBoardSensorAddress;
		break;
	default:
		sensorAddress = LEDBoardSensorAddress;
		break;
	}
	NCT175_write8BitRegister(sensorAddress, NCT175_configRegister, NCT175_configRegVal);
	NCT175_write16BitRegister(sensorAddress, THYSTRegister, THYSTRegVal);
	NCT175_write16BitRegister(sensorAddress, TOSRegister, TOSRegVal);
}

/*
 * @summary	This function reads the temperature from one
 * 			of the 3 temperature sensors
 * @inputs	sensor:	1 - Driver Board PCB Sensor
 * 					2 - Ambient Air Sensor
 * 					3 - LED Board PCB Sensor
 * 			units:	1 - degrees F
 * 					2 - degrees C
 * @return	float - temperature
 */
float NCT175_readTemperature(uint8_t sensor, uint8_t units){
	uint16_t NCT175ADCValue;
	float temperatureC = 0.0;

	switch(sensor){
	case 1:
		NCT175ADCValue = NCT175_read16BitRegister(driverBoardSensorAddress, tempRegister)>>4;
		break;
	case 2:
		NCT175ADCValue = NCT175_read16BitRegister(ambientAirSensorAddress, tempRegister)>>4;
		break;
	case 3:
		NCT175ADCValue = NCT175_read16BitRegister(LEDBoardSensorAddress, tempRegister)>>4;
		break;
	default: //invalid entry will return the LED board temperature
		NCT175ADCValue = NCT175_read16BitRegister(LEDBoardSensorAddress, tempRegister)>>4;
		break;
	}
	temperatureC = NCT175ADCValue/16.0;

	switch(units){
	case 1:
		//return degrees F
		return NCT175_CtoF(temperatureC);
		break;
	case 2:
		//return degrees C
		return temperatureC;
		break;
	default:
		return 0x00; //error - return 0 degrees
		break;
	}
}

/*
 * @Summary	This function converts degrees C to degrees F
 * @Inputs	Degrees C
 * @Return	Degrees F
 */
float NCT175_CtoF(float C){
	return (C*1.8)+32.0;
}

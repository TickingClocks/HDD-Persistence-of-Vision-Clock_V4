/*
 * DS1307_Basic.c
 *
 *  Created on: Sep 2, 2022
 *      Author: Kadin Whiting
 */


#include "main.h"
#include "DS1307_Basic.h"

/**
* @Summary 	This function clears the CH bit enabling the RTC W/O losing seconds value and
* 		 	sets 12 hour time w/o losing hours value
*
* @param  NONE
*
* @retval NONE
*/
void RTCInit(void){

	HAL_StatusTypeDef ret;

	uint8_t secondsHold[1];
	uint8_t hoursHold[1];

	//clear CH bit
	//read current value in register
	ret = HAL_I2C_Mem_Read(&hi2c1, (RTCAddress<<1) | 0x01, secondsRegister, I2C_MEMADD_SIZE_8BIT, secondsHold, 1, 100);

	//clear CH bit
	secondsHold[0] &= (~0x80);
	//place same seconds value as was just retrieved - ensure CH bit is DISABLED
	ret = HAL_I2C_Mem_Write(&hi2c1, (RTCAddress<<1), secondsRegister, I2C_MEMADD_SIZE_8BIT, secondsHold, 1, 100);

	//set 12 hour time
	//read current value in register
	ret = HAL_I2C_Mem_Read(&hi2c1, (RTCAddress<<1) | 0x01, hoursRegister, I2C_MEMADD_SIZE_8BIT, hoursHold, 1, 100);

	//set 12 hour time bit
	hoursHold[0] |= (0x01<<6);
	//place same hours value as was just retrieved - ensure 12 hour bit is ENABLED
	ret = HAL_I2C_Mem_Write(&hi2c1, (RTCAddress<<1), hoursRegister, I2C_MEMADD_SIZE_8BIT, hoursHold, 1, 100);
}

/**
* @Summary This function sets the seconds value in the RTC
* @param  userSeconds _ desired user seconds (0-59)
* @retval NONE
*/
void RTCSetSeconds(uint8_t userSeconds){

	//ensure seconds value is valid
	if(userSeconds > 59) return;

	HAL_StatusTypeDef ret;
	uint8_t rem;
	uint8_t baseSeconds;
	uint8_t tensSeconds;
	uint8_t BCDSeconds;
	uint8_t dataBuf[1];
	dataBuf[0] = 0x00;

	/*
	 * seconds data is in BCD format where the
	 * tens seconds are located in bits 4, 5, 6
	 * seconds are located in bits 0, 1, 2, 3
	 * bit 7 is the CH (clock halt) bit
	 */
	if(userSeconds > 9){
		rem = userSeconds % 10;
		tensSeconds = (userSeconds - rem)/10;
		baseSeconds = rem;
	}else{
		tensSeconds = 0;
		baseSeconds = userSeconds;
	}

	tensSeconds = tensSeconds<<4;
	BCDSeconds = tensSeconds | baseSeconds;

	//BCD format with bit 7 (CH bit) set LOW
	dataBuf[0] = BCDSeconds;

	//place seconds value in seconds register
	ret = HAL_I2C_Mem_Write(&hi2c1, (RTCAddress<<1), secondsRegister, I2C_MEMADD_SIZE_8BIT, dataBuf, 1, HAL_MAX_DELAY);
}

/**
* @Summary This function sets the minutes value in the RTC
* @param  userMinutes _ desired user minutes (0-59)
* @retval NONE
*/
void RTCSetMinutes(uint8_t userMinutes){

	//ensure seconds value is valid
	if(userMinutes > 59) return;

	HAL_StatusTypeDef ret;
	uint8_t rem;
	uint8_t baseMinutes;
	uint8_t tensMinutes;
	uint8_t BCDMinutes;
	uint8_t dataBuf[1];
	dataBuf[0] = 0x00;

	/*
	 * minutes data is in BCD format where the
	 * tens minutes are located in bits 4, 5, 6
	 * minutes are located in bits 0, 1, 2, 3
	 * bit 7 is always 0
	 */
	if(userMinutes > 9){
		rem = userMinutes % 10;
		tensMinutes = (userMinutes - rem)/10;
		baseMinutes = rem;
	}else{
		tensMinutes = 0;
		baseMinutes = userMinutes;
	}

	tensMinutes = tensMinutes<<4;
	BCDMinutes = tensMinutes | baseMinutes;
	BCDMinutes &= (~0b10000000); //MSB is always 0

	//BCD format with bit 7 (CH bit) set LOW
	dataBuf[0] = BCDMinutes;

	//place minutes value in minutes register
	ret = HAL_I2C_Mem_Write(&hi2c1, (RTCAddress<<1), minutesRegister, I2C_MEMADD_SIZE_8BIT, dataBuf, 1, HAL_MAX_DELAY);
}

/**
* @brief This function sets the hours value in the RTC
*
* @param  userHours _ desired user hours (1-12)
*
* @retval NONE
*/
void RTCSetHours(uint8_t userHours){

	//ensure hours vlaue is valid (only 12 hour time allowed)
	if(userHours > 59) return;
	if(userHours < 1) return;

	HAL_StatusTypeDef ret;
	uint8_t rem;
	uint8_t baseHours;
	uint8_t tensHours;
	uint8_t BCDHours;
	uint8_t dataBuf[1];
	uint8_t hourTimeSetting[1];
	hourTimeSetting[0] = 0b01000000; //set 12 hour time
	dataBuf[0] = 0x00;

	/*
	 * hours data is in BCD format where the
	 * tens hours is located in bit 4,
	 * hours are located in bits 0, 1, 2, 3
	 * bit 5 is the AM/PM bit or second tens hours in 24 hour time
	 * bit 6 is the 12/24 hour selection (DONT CHANGE THIS BIT)
	 * bit 7 is always 0
	 */
	if(userHours > 9){
		rem = userHours % 10;
		tensHours = (userHours - rem)/10;
		baseHours = rem;
	}else{
		tensHours = 0;
		baseHours = userHours;
	}

	tensHours = tensHours<<4;
	BCDHours = tensHours |= baseHours;
	BCDHours |= hourTimeSetting[0]; //ensure we stay at 12 hour time
	BCDHours &= (~0b10000000); //bit 7 is always 0

	//BCD format with bit 7 (CH bit) set LOW
	dataBuf[0] = BCDHours;

	//place same hours value as was  just retrieved - ensure 12 hour bit is ENABLED
	ret = HAL_I2C_Mem_Write(&hi2c1, (RTCAddress<<1), hoursRegister, I2C_MEMADD_SIZE_8BIT, dataBuf, 1, HAL_MAX_DELAY);
}

/**
* @brief This function sets AM/PM
*
* @param  AMPM _ user AM/PM selection
* 					0 _ AM
* 					1 _ PM
*
* @retval NONE
*/
void RTCSetAMPM(uint8_t AMPM){

	if(AMPM > 1) return;

	HAL_StatusTypeDef ret;
	uint8_t hoursRegHold[1];
	uint8_t setHoursReg[1];
	hoursRegHold[0] = 0x00;
	setHoursReg[0] = 0x00;

	//read current value in register
	ret = HAL_I2C_Mem_Read(&hi2c1, (RTCAddress<<1) | 0x01, hoursRegister, I2C_MEMADD_SIZE_8BIT, hoursRegHold, 1, HAL_MAX_DELAY);

	//bit 5 is AM/PM bit. Logic high is PM
	if(AMPM){
		setHoursReg[0] = hoursRegHold[0] | 0b00100000; //set bit 5 HIGH for PM
	}else{
		setHoursReg[0] = hoursRegHold[0] & ~0b00100000; //set bit 5 LOW for PM
	}

	//write new value in register
	ret = HAL_I2C_Mem_Write(&hi2c1, (RTCAddress<<1), hoursRegister, I2C_MEMADD_SIZE_8BIT, setHoursReg, 1, HAL_MAX_DELAY);
}

/**
* @Summary This function reads the seconds value from RTC
* @param  NONE
* @retval uint8_t _ seconds retrieved from RTC
*/
uint8_t RTCReadSeconds(void){

	HAL_StatusTypeDef ret;
	uint8_t secondsReg[1];
	uint8_t tensSeconds = 0x00;
	uint8_t baseSeconds = 0x00;
	uint8_t fullSeconds = 0;
	secondsReg[0] = 0x00;

	//read current value in register
	ret = HAL_I2C_Mem_Read(&hi2c1, (RTCAddress<<1) | 0x01, secondsRegister, I2C_MEMADD_SIZE_8BIT, secondsReg, 1, HAL_MAX_DELAY);

	tensSeconds = ((secondsReg[0] & ~0b10001111)>>4);
	tensSeconds*=10;
	baseSeconds = secondsReg[0] & ~0b11110000;
	fullSeconds = tensSeconds + baseSeconds;

	return fullSeconds;

}

/**
* @Summary This function reads the minutes value from RTC
* @param  NONE
* @retval uint8_t _ minutes retrieved from RTC
*/
uint8_t RTCReadMinutes(void){

	HAL_StatusTypeDef ret;
	uint8_t minutesReg[1];
	uint8_t tensMinutes = 0x00;
	uint8_t baseMinutes = 0x00;
	uint8_t fullMinutes = 0;
	minutesReg[0] = 0x00;

	//read current value in register
	ret = HAL_I2C_Mem_Read(&hi2c1, (RTCAddress<<1) | 0x01, minutesRegister, I2C_MEMADD_SIZE_8BIT, minutesReg, 1, HAL_MAX_DELAY);

	tensMinutes = ((minutesReg[0] & ~0b10001111)>>4);
	tensMinutes*=10;
	baseMinutes = minutesReg[0] & ~0b11110000;
	fullMinutes = tensMinutes + baseMinutes;

	return fullMinutes;

}

/**
* @Summary This function reads the hours value from RTC
* @param  NONE
* @retval uint8_t _ hours retrieved from RTC
*/
uint8_t RTCReadHours(void){

	HAL_StatusTypeDef ret;
	uint8_t hoursReg[1];
	uint8_t tensHours = 0x00;
	uint8_t baseHours = 0x00;
	uint8_t fullHours = 0;
	hoursReg[0] = 0x00;

	//read current value in register
	ret = HAL_I2C_Mem_Read(&hi2c1, (RTCAddress<<1) | 0x01, hoursRegister, I2C_MEMADD_SIZE_8BIT, hoursReg, 1, HAL_MAX_DELAY);

	tensHours = ((hoursReg[0] & ~0b11101111)>>4); //only including code for 12 hour time
	tensHours*=10;
	baseHours = hoursReg[0] & ~0b11110000;
	fullHours = tensHours + baseHours;

	return fullHours;

}

/**
* @Summary This function reads AM/PM from RTC
* @param  NONE
* @retval uint8_t _ AM/PM
*    					-0 AM
*    					-1 PM
*/
uint8_t RTCReadAMPM(void){

	HAL_StatusTypeDef ret;
	uint8_t hoursReg[1];
	uint8_t AMPM_Location = 0b11011111; //bit 5 is AM/PM
	uint8_t AMPM = 0;
	hoursReg[0] = 0x00;

	//read current value in register
	ret = HAL_I2C_Mem_Read(&hi2c1, (RTCAddress<<1) | 0x01, hoursRegister, I2C_MEMADD_SIZE_8BIT, hoursReg, 1, HAL_MAX_DELAY);

	//see if AM/PM bit is high or low
	AMPM = hoursReg[0] & ~AMPM_Location;

	if(AMPM){
		return 1; //PM
	}else{
		return 0; //AM
	}

}

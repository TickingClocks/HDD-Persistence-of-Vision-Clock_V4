/*
 * DS1307.h
 *
 *  Created on: Oct 31, 2021
 *      Author: Kadin Whiting
 *
 *  This is a basic library for the DS1307 RTC Chip
 *  This library includes functions to
 *  -disable CH (clock halt) bit and set 12 hour time mode _ void RTCInit(void)
 *  -set DS1307 seconds value _ void RTCSetSeconds(uint8_t userSeconds)
 *  -set DS1307 minutes value _ void RTCSetSeconds(uint8_t userMinutes)
 *  -set DS1307 hours value _ void RTCSetHours(uint8_t userHours)
 *  -set DS1307 AM/PM value _ void RTCSetAMPM(uint8_t AMPM)
 *  -read DS1307 seconds value _ uint8_t RTCReadSeconds(void)
 *  -read DS1307 minutes value _ uint8_t RTCReadMinutes(void)
 *  -read DS1307 hours value _ uint8_t RTCReadHours(void)
 *  -read DS1307 AM/PM value _ uint8_t RTCReadAMPM(void)
 */

#ifndef INC_DS1307_H_
#define INC_DS1307_H_

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void RTCInit(void);
uint8_t RTCReadAMPM(void);
uint8_t RTCReadHours(void);
uint8_t RTCReadMinutes(void);
uint8_t RTCReadSeconds(void);
void RTCSetAMPM(uint8_t AMPM);
void RTCSetHours(uint8_t userHours);
void RTCSetMinutes(uint8_t userMinutes);
void RTCSetSeconds(uint8_t userSeconds);


//address'
#define RTCAddress  0b1101000
#define secondsRegister  0x00
#define minutesRegister  0x01
#define hoursRegister  0x02


#endif /* INC_DS1307_H_ */

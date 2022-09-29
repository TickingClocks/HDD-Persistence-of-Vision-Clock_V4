/*
 * APA102-2020.h
 *
 *  Created on: Aug 18, 2022
 *      Author: Kadin
 *
 *  The APA102-2020 is a small-package digital, individually addressable LED
 *  The small 2020 package allows for me to place 116 LEDs on the LED board
 *  while using convenient numbers that are divisible by 12 (for clock functions)
 *
 *  The APA102-2020 LED uses an SPI protocol to relay color and brightness information
 *  These LEDs can be communicated with at very fast speeds.
 *
 *  The APA102-2020 requires 4 bytes for each LED
 *  	1 byte for brightness (default this to 255 for my application)
 *  	1 byte for red color data
 *  	1 byte for green color data
 *  	1 byte for blue color data
 *
 *  The communication protocol for these LEDs
 *  1. Send 4 starting bytes - all 0
 *  2. Send color data in this order
 *  	1. blue
 *  	2. green
 *  	3. red
 *  3. Send 4 ending bytes - all 1
 *
 *  There are 2 separate APA102-2020 LED Lines on the HDDCLK V4.0
 *  	1. LED line is the main LED board display LEDs
 *  	2. LED line is the single LED on the back of the driver board.
 */

#ifndef INC_APA102_2020_H_
#define INC_APA102_2020_H_

#include "main.h"

extern SPI_HandleTypeDef 	hspi2;
extern SPI_HandleTypeDef 	hspi5;

//LED Variables
extern uint8_t 				ledBuffer[120][4];
extern uint8_t 				ledBuffer2[3][4];


//function prototypes
void setLED(uint8_t line, uint8_t LED, uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue);
void updateLEDs(uint8_t line);
void setDisplayAllOneColor(uint8_t red, uint8_t green, uint8_t blue);
void LEDsReset(void);

#endif /* INC_APA102_2020_H_ */



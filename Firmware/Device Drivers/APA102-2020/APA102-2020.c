/*
 * APA102-2020.c
 *
 *  Created on: Aug 18, 2022
 *      Author: Kadin
 */


#include "APA102-2020.h"


/*
 * @summary	Updates the LED buffers with color and brightness values
 * @inputs	line: which LED line
 * 				1 - Display
 * 				2 - Rear LED
 * 			LED - Which LED in line (start at 0)
 * 			brightness - 0-32 brightness value
 * 			red - red value (0-255)
 * 			green - green value (0-255)
 * 			blue - blue value (0-255)
 * @return	none
 */
void setLED(uint8_t line, uint8_t LED, uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue){

	brightness = 255; //I'm simplifying things by making brightness constant. value 255 works best

	switch (line){
	case 1: //display
		ledBuffer[LED+1][0] = brightness;
		ledBuffer[LED+1][1] = blue; //b
		ledBuffer[LED+1][2] = green;//g
		ledBuffer[LED+1][3] = red;//r
		break;

	case 2: //rear LED
		ledBuffer2[LED+1][0] = brightness;
		ledBuffer2[LED+1][1] = blue;
		ledBuffer2[LED+1][2] = green;
		ledBuffer2[LED+1][3] = red;
		break;
	}
}


/*
 * @summary	This function sends the LED buffer
 * 			data to the LEDs via SPI
 * @input	line
 * 				1 - display
 * 				2 - back LED
 * @return	none
 */
void updateLEDs(uint8_t line){
	HAL_StatusTypeDef status;
	switch(line){
	case 1:
		//display
		status = HAL_SPI_Transmit(&hspi2, ledBuffer, 120 * 4, 100); //error is likely due to 2D array?
		break;
	case 2:
		//rear LED
		status = HAL_SPI_Transmit(&hspi5, ledBuffer2, (3 * 4), 100); //error is likely due to 2D array?
		break;
	}
}

void setDisplayAllOneColor(uint8_t red, uint8_t green, uint8_t blue){
	//reset LED data
	for(uint8_t e=1; e<118; e++){
		ledBuffer[e][0] = 255;
		ledBuffer[e][1] = blue;
		ledBuffer[e][2] = green;
		ledBuffer[e][3] = red;
	}
}

/*
 * @summary	This function clears the LED buffers
 * @Inputs	none
 * @return	none
 */
void LEDsReset(void){

	//reset main LED array
	//beginning bytes
	for(uint8_t e=0; e<4; e++){
		ledBuffer[0][e] = 0;
	}
	//reset LED data
	for(uint8_t e=1; e<118; e++){
		ledBuffer[e][0] = 255;
		ledBuffer[e][1] = 0;
		ledBuffer[e][2] = 0;
		ledBuffer[e][3] = 0;
	}
	//ending bytes
	for(uint8_t e=0; e<4; e++){
		ledBuffer[118][e] = 1;
	}

	//reset second LED array
	//beginning bytes
	for(uint8_t e=0; e<4; e++){
		ledBuffer2[0][e] = 0;
	}
	//reset LED data
	ledBuffer2[1][0] = 255;
	ledBuffer2[1][1] = 0;
	ledBuffer2[1][2] = 0;
	ledBuffer2[1][3] = 0;
	//ending bytes
	for(uint8_t e=0; e>4; e++){
		ledBuffer2[2][e] = 1;
	}
}

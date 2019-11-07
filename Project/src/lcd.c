
/* LCD Code for ECE 355 Project */


#include <stdio.h>
#include "stm32f0xx_spi.c"
#include "stm32f0xx_rcc.c"
#include "stm32f0xx_gpio.c"
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"


/* Pragma's */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/*
Functions
	Initialize the LCD
	Initialize GPIO Pins
	Initialize SPI
	Initialize timers [Latch Clock: LCK][Shift Register Clock: SCK]

	Convert to ASCII <- done in write functions
	Clear Display
	Write Display
		Write Frequency
		Write Resistance
	Delay
*/

/* Global Variable Declarations */

/* Initialization Code */

void myGPIO_Init(){
	// initilize all pins that are associated with the LCD.
	/*
		LCD_D4
		LCD_D5
		LCD_D6
		LCD_D7
		RS
		EN
	 */
}

void myLCK_Init(){
	// initialize LCK
}

void mySCK_Init(){
	// initialize SCK
}

void mySPI_Init(){
	// call all SPI initilization Functions
}

void myLCD_init(){
	// change to use 4 bit interface
}

/* Functions */

void write_Freq(uint32_t frequency){ // takes in frequency and displays it.
	//take in freq
	//convert to ASCII
	//Write to LCD
}

void write_Res(uint32_t resistance){ // takes in resistance and displays it.
	//take in resistance
	//convert to ASCII
	//Write to LCD
}

void clear_LCD(){
	// Clear the screen to be able to take new values.
}

void Delay(uint32_t time){ // Time is in milliseconds
	// delay the system. ....
}






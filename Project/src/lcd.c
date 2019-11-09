
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

#define myTIM14_PRESCALER ((uint16_t)0x0000) 		/* Clock prescaler for TIM14 timer: ADD STUFF */
#define myTIM14_PERIOD ((uint32_t)0xFFFFFFFF)	 	/* Maximum possible setting for overflow */

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
	 WHAT PINS ARE WE USING
	 */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 			// Enable clock for GPIOA peripheral. Relevant register: RCC->AHBENR

    // Configure PA0
    GPIOA->MODER &= ~(GPIO_MODER_MODER0);		// Configure PA0 as input. Relevant register: GPIOA->MODER. [00]
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1); 		// Ensure no pull-up/pull-down for PA1. Relevant register: GPIOA->PUPDR

    // Configure PA1
    GPIOA->MODER &= ~(GPIO_MODER_MODER1); 		// Configure PA1 as input. Relevant register: GPIOA->MODER. [00]
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);		// Ensure no pull-up/pull-down for PA0. Relevant register: GPIOA->PUPDR

    // Configure PA4 as an analog mode pin
    GPIOA->MODER &= ~(GPIO_MODER_MODER4);		// Configure PA4 as output. Relevant register: GPIOA->MODER. [11]
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);		// Ensure no pull-up/pull-down for PA4. Relevant register: GPIOA->PUPDR

}

void myLCK_Init(){
	// initialize LCK
}

void mySCK_Init(){
	// initialize SCK
}

void mySPI_Init(){
	// call all SPI initilization Functions

	//Given in class
	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

    SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct->SPI_BaudRatePrescaler = ... ;
    SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct->SPI_CRCPolynomial = 7;

    SPI_Init(SPI1, SPI_InitStruct);
    SPI_Cmd(SPI1, ENABLE);
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

void Delay_Init() {

    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;			//Enable clock for TIM2 peripheral. Relevant register: RCC->APB1ENR

    /* Configure TIM2: buffer auto-reload, count up, stop on overflow,
       enable update events, interrupt on overflow only */

    TIM14->CR1 = ((uint16_t)0x008C); 			// Relevant register: TIM2->CR1
    TIM14->PSC = myTIM14_PRESCALER; 				// Set clock prescaler value
    TIM14->ARR = myTIM14_PERIOD; 					// Set auto-reloaded delay
    TIM14->EGR = ((uint16_t)0x0001); 			// Update timer registers.  Relevant register: TIM2->EGR

    //MAYBE CHANGE PRIORITY
    NVIC_SetPriority(TIM14_IRQn, 0); 			// Relevant register: NVIC->IP[3], or use NVIC_SetPriority. Assign TIM2 interrupt priority = 0 in NVIC.
    NVIC_EnableIRQ(TIM14_IRQn); 					// Enable TIM2 interrupts in NVIC. Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ.

    TIM14->DIER |= TIM_DIER_UIE; 				// Enable update interrupt generation  Relevant register: TIM14->DIER.

    //They added a line to activate timer, not sure if necessary
}


void Delay(uint32_t time){ // Time is in milliseconds
	// delay the system. ....

	TIM14->CNT |= 0x00;     //Clear timer

	TIM14->ARR = time;      //Autoload time into timer

    TIM14->EGR = ((uint16_t)0x0001); // Update timer registers

    TIM14->CR1 |= TIM_CR1_CEN;      //Enabled counter

    while ((TIM14->SR & TIM_SR_UIF) != 0); //Loop until interrupt flag gets set

    TIM14->SR &= ~(TIM_SR_UIF);				// Clear update interrupt flag.
    TIM14->CR1 |= TIM_CR1_CEN;		 		// Restart stopped timer.


}






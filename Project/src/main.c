//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//
// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------
// Code written by:
// 		Joshua Wagner
//		Sam Kosman
//

/******************************************** Includes **************************************************/


#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

/******************************************** Pragmas **************************************************/

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/******************************************** Defines **************************************************/

#define myTIM2_PRESCALER ((uint16_t)0x0000) 		/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)	 	/* Maximum possible setting for overflow */

#define myTIM3_PRESCALER ((uint16_t)0x40)
//#define myTIM3_PERIOD ((uint32_t)0xFFFFFFFF)

//#define myTIM14_PRESCALER ((uint16_t)0x12C0) 		/* Clock prescaler for TIM14 timer equals 4800 to delay the counter by 1 ms*/
//#define myTIM14_PERIOD ((uint32_t)0xFFFFFFFF)	 	/* Maximum possible setting for overflow */

#define SPI_Direction_1Line_Tx ((uint16_t)0xC000)
#define SPI_Mode_Master ((uint16_t)0x0104)
#define SPI_DataSize_8b ((uint16_t)0x0700)
#define SPI_CPOL_Low ((uint16_t)0x0000)
#define SPI_CPHA_1Edge ((uint16_t)0x0000)
#define SPI_NSS_Soft SPI_CR1_SSM
#define SPI_FirstBit_MSB ((uint16_t)0x0000)
#define SPI_CR1_SSM ((uint16_t)0x0200)

/******************************************** Initialization Functions (MIGHT NOT BE NEEDED)**************************************************/

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void mySPI_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void myLCD_Init(void);

/******************************************** Global Variables **************************************************/

unsigned int edge = 0; 								// Edge counter
uint32_t pulse_count = 0;
uint32_t adc_offset = 0;
uint32_t LCD_command = 0x00;

float freq = 0.00;

//uint32_t ADC_MAX_Value = ;  						//ADC supply requirements: 2.4 V to 3.6 V
//uint32_t ADC_resolution = 4096 ;

//uint32_t DAC_MAX_Value = ;
//uint32_t DAC_resolution = ;
/******************************************** ADC Potentiometer Code **************************************************/

uint32_t ADC_pot(){
	//drink up bitches and start conversion
	ADC1->CR |= ADC_CR_ADSTART;
	trace_printf("Conversion Therapy...\n");
	//wait for end of conversion
	while(!(ADC1->ISR & ADC_ISR_EOC));
	trace_printf("... Complete!\n");
	//reset flag
	ADC1->ISR &= ~(ADC_ISR_EOC);

	//push value
	uint32_t potential = ((ADC1->DR)& ADC_DR_DATA) - adc_offset;
	trace_printf("\nADC_DR_DATA value: %u\n",ADC_DR_DATA);
	trace_printf("\nOffset: %u\n", adc_offset);
	return potential;
	// CONVERSION COMPLETE WELCOME JESUS YOU HAVE UNLOCKED YOUR POTENTIAL
}

/******************************************** Main Code **************************************************/

int main(int argc, char* argv[]) {

	trace_printf("This is the Final Project...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myGPIOB_Init();		/* Initialize I/O port PB */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myTIM3_Init();		/* Initialize timer TIM3 */
	mySPI_Init();		/* Initialize SPI */
	myADC_Init();		/* Initialize ADC */
	myDAC_Init();		/* Initialize DAC */
	myLCD_Init();		/* Initialize LCD */
	myEXTI_Init();		/* Initialize EXTI */

	uint32_t input = ADC_pot();
	trace_printf("\nADC Value: %u\n", input);

	/* Store DAC Value in Output buffer */
	DAC->DHR12R1 = input;
	trace_printf("\nDAC Output buffer: %u\n", DAC->DHR12R1);
	uint32_t resistance	= input;

	while (1){

		/*
		We will be measuring the analog value into Pin 0 here.
		We could technically do this as a function and just call it here.
		Call ADC function return value
		Call DAC Function with ADC value Push to Opto.
		Calculate resistance
		push to LCD.
		 */
	}
	return 0;
}
/******************************************** Initialization Code **************************************************/

void myGPIOA_Init(){
	trace_printf("\nGPIOA Initializing\n");
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 			// Enable clock for GPIOA peripheral. Relevant register: RCC->AHBENR

	// Configure PA0 for potentiometer
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);		// Configure PA0 as input. Relevant register: GPIOA->MODER. [00]
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1); 		// Ensure no pull-up/pull-down for PA1. Relevant register: GPIOA->PUPDR

	// Configure PA1 for 555
	GPIOA->MODER &= ~(GPIO_MODER_MODER1); 		// Configure PA1 as input. Relevant register: GPIOA->MODER. [00]
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);		// Ensure no pull-up/pull-down for PA0. Relevant register: GPIOA->PUPDR

	// Configure PA4 as an analog mode pin for opto
	GPIOA->MODER &= ~(GPIO_MODER_MODER4);		// Configure PA4 as output. Relevant register: GPIOA->MODER. [11]
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);		// Ensure no pull-up/pull-down for PA4. Relevant register: GPIOA->PUPDR
	trace_printf("\nGPIOA Initialized\n");
}

// initialize all pins that are associated with the LCD including LCK.
void myGPIOB_Init(){
	trace_printf("\nGPIOB Initializing\n");

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 			// Enable clock for GPIOB peripheral. Relevant register: RCC->AHBENR

	// Configure PB3 as SPI-SCK:M21
	GPIOB->MODER &= ~(GPIO_MODER_MODER3_1);		// Configure PB3 as alternate function. Relevant register: GPIOB->MODER. [11]
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3); 		// Ensure no pull-up/pull-down for PA1. Relevant register: GPIOB->PUPDR

	// Configure PB5 as SPI-MOSI:M17
	GPIOB->MODER &= ~(GPIO_MODER_MODER5_1); 	// Configure PB3 as alternate function. Relevant register: GPIOB->MODER. [11]
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);		// Ensure no pull-up/pull-down for PA0. Relevant register: GPIOB->PUPDR

	// Configure PB4 as LCK:M25
	GPIOB->MODER &= ~(GPIO_MODER_MODER4_0);		// Configure PA4 as output. Relevant register: GPIOA->MODER. [11]
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);		// Ensure no pull-up/pull-down for PA4. Relevant register: GPIOA->PUPDR

	trace_printf("\nGPIOB Initialized\n");
}

//Initialize SPI by calling all SPI initialization Functions
void mySPI_Init(){
	trace_printf("\nSPI Initializing\n");

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; 		//Enable SPI1 clock

	//Given in class
	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;	//Biggest pre-scaler
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, SPI_InitStruct);
	SPI_Cmd(SPI1, ENABLE);

	trace_printf("\nSPI Initialized\n");
}
//Initialize TIM2 clock to be used internal interrupts
void myTIM2_Init(){
	trace_printf("\nTIM 2 Initializing\n");
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;			//Enable clock for TIM2 peripheral. Relevant register: RCC->APB1ENR

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	   enable update events, interrupt on overflow only */

	TIM2->CR1 = ((uint16_t)0x008C); 			// Relevant register: TIM2->CR1
	TIM2->PSC = myTIM2_PRESCALER; 				// Set clock prescaler value
	TIM2->ARR = myTIM2_PERIOD; 					// Set auto-reloaded delay
	TIM2->EGR = ((uint16_t)0x0001); 			// Update timer registers.  Relevant register: TIM2->EGR

	NVIC_SetPriority(TIM2_IRQn, 0); 			// Relevant register: NVIC->IP[3], or use NVIC_SetPriority. Assign TIM2 interrupt priority = 0 in NVIC.
	NVIC_EnableIRQ(TIM2_IRQn); 					// Enable TIM2 interrupts in NVIC. Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ.

	TIM2->DIER |= TIM_DIER_UIE; 				// Enable update interrupt generation  Relevant register: TIM2->DIER.
	trace_printf("\nTIM 2 Initialized\n");
}

//Initialize TIM3 clock to be used for Delay function for LCD
void myTIM3_Init(){
	trace_printf("\nTIM 3 Initializing\n");

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;			//Enable clock for TIM3 peripheral. Relevant register: RCC->APB1ENR


	TIM3->CR1 = ((uint16_t)0x10); 				// Relevant register: TIM3->CR1
	TIM3->PSC = myTIM3_PRESCALER; 				// Set clock prescaler value
	trace_printf("\nTIM 3 Initialized\n");
}

// Initialize  LCD. need to change to 4bit Mode
void myLCD_Init(){
	trace_printf("\nLCD Initializing\n");

	mySPI_SendData(0x03);
	mySPI_SendData(0x83);
	mySPI_SendData(0x03);

	Delay(1);

	mySPI_SendData(0x03);
	mySPI_SendData(0x83);
	mySPI_SendData(0x03);

	Delay(1);

	mySPI_SendData(0x03);
	mySPI_SendData(0x83);
	mySPI_SendData(0x03);

	mySPI_SendData(0x02);
	mySPI_SendData(0x82);
	mySPI_SendData(0x02);



	mySPI_sendControl(0x2,LCD_command ); // set to 4-bit mode
	//Delay(1);
	mySPI_sendControl(0x28, LCD_command);
	mySPI_sendControl(0x0C, LCD_command);
	mySPI_sendControl(0x06, LCD_command);
	mySPI_sendControl(0x01, LCD_command);

	//clear_LCD();
	//Delay(1);
	trace_printf("\nLCD Initialized\n");
}
void myEXTI_Init(){									// Initializing EXTI
	trace_printf("\nEXTI 1 Initializing\n");
	SYSCFG->EXTICR[0]= SYSCFG_EXTICR1_EXTI1_PA; 	// Map EXTI1 line to PA1. Relevant register: SYSCFG->EXTICR[0]
	EXTI->RTSR |= EXTI_RTSR_TR1; 					// EXTI1 line interrupts: set rising-edge trigger.Relevant register: EXTI->RTSR
	EXTI->IMR |= EXTI_IMR_MR1; 						// Unmask interrupts from EXTI1 line. Relevant register: EXTI->IMR. unmasked so it is not ignored.

	NVIC_SetPriority(EXTI0_1_IRQn , 0); 			// Assign EXTI1 interrupt priority = 0 in NVIC. Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_EnableIRQ(EXTI0_1_IRQn) ; 					// Enable EXTI1 interrupts in NVIC. Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	trace_printf("\nEXTI 1 Initialized\n");
}

// 12-bit -> stored in a left aligned or right aligned 16-bit data register. can set thresholds Upper and lower.
void myADC_Init(){
	trace_printf("\nADC Initializing\n");
	trace_printf("\nADC knock knock\n");

	//Enable ADC RCC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;				//Enable clock for ADC peripheral. Relevant register: RCC->APB2ENR

	if(ADC_CR_ADEN == 0x00) {						// If calibration is not complete
		trace_printf("\nADC unlock that door\n");
		ADC1->CR |= ADC_CR_ADCAL;					// Enable calibration
		while(ADC_CR_ADCAL == 1);					// Wait for calibration to complete
		adc_offset = ((ADC1->DR)& ADC_DR_DATA);		// Store calibration factor [6:0] from ADC_DR
		trace_printf("\nADC SHE UNLOCKED ENTER AT OWN RISK\n");
	}

	//Configure ADC: continuous and overrun
	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;				//overrun
	ADC1->CFGR1 |= ADC_CFGR1_CONT;					//continuous
	//ADC1->CFGR1 |= ADC_CFGR1_RES;					//set resolution (maybe to 12 = [00]) (I think it is already set there?)

	//Select operating channel (output pin) - ADC_IN0
	ADC1->CHSELR |= ADC_CHSELR_CHSEL0;				//Channel IN0 is selected to be converted
	ADC1->CR |= ADC_CR_ADEN;						//Enabled ADC


	trace_printf("\nADC TEEEHEE\n");				// Waiting for ADC to be ready for conversion
	while(ADC_ISR_ADRDY == 0);						// True when not ready for conversion
	trace_printf("\nADC IS IN DA HOUSE\n");
	trace_printf("\nADC Initialized\n");
}

void myDAC_Init(){
	// Enable clock
	trace_printf("\nDAC Initializing\n");
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;				//Enable clock for DAC peripheral. Relevant register: RCC->APB1ENR

	// Enable Output buffer
	DAC->CR |= DAC_CR_BOFF1;

	// Enable DAC
	DAC->CR |= DAC_CR_EN1;
	trace_printf("\nDAC Initialized\n");
}
/******************************************** SPI Code **************************************************/

void mySPI_SendData(uint8_t data) {
	trace_printf("\nSending DATA ..... \n");

	GPIOB->BRR = GPIO_Pin_4;					/* Force you LCK signal to 0 */

	/* Wait until SPI1 is ready (TXE = 1 or BSY = 0) */
	while((SPI1->SR & SPI_SR_TXE) == 0x00 || (SPI1->SR & SPI_SR_BSY) != 0x00);

	SPI_SendData8(SPI1, data);					/* Assumption: your data holds 8 bits to be sent*/

	while((SPI1->SR & SPI_SR_BSY) != 0x00);		/* Wait until SPI1 is not busy (BSY = 0) */

	GPIOB->BRR = GPIO_Pin_4;					/* Force your LCK signal to 1 */
	trace_printf("\nSend DATA Success.\n");
}

void mySPI_sendControl(uint8_t address, uint8_t type){ //sends LCD control commands including addressing, clearing,
	trace_printf("\nSend Control start \n");

	uint8_t highhalf = (0xF0 & (address>>4)); 	// generate highhalf address. Shifted 4 to left
	uint8_t lowhalf = (0x0F & address); 		// generate lowhalf address

	mySPI_SendData((0x00 | type | highhalf)); 	// disable LCD, push type, highhalf,
	mySPI_SendData((0x80 | type | highhalf)); 	// enable LCD, push type, highhalf,
	mySPI_SendData((0x00 | type | highhalf)); 	// disable LCD, push type, highhalf,

	mySPI_SendData((0x00 | type | lowhalf)); 	// disable LCD, push type, highhalf,
	mySPI_SendData((0x80 | type | lowhalf)); 	// enable LCD, push type, highhalf,
	mySPI_SendData((0x00 | type | lowhalf)); 	// disable LCD, push type, highhalf,

	trace_printf("\nThe send has been Controlled.\n");
}

/******************************************** LCD Helper Functions **************************************************/

void write_Freq(float frequency){ // ( AKA_ write High Part of LCD) takes in frequency and displays it.
	trace_printf("\nWriting on the Frequency\n");

	set_Address(0,0);
	char disp[7];
	char freq_buffer[4];
	uint32_t freq =(uint32_t) frequency;

	itoa(freq,freq_buffer,10);		// convert number to string array

	disp[0] =  'F';					// load everything into buffer.
	disp[1] =  ':';
	disp[2] = freq_buffer[3];
	disp[3] = freq_buffer[2];
	disp[4] = freq_buffer[1];
	disp[5] = freq_buffer[0];
	disp[6] =  'H';
	disp[7] =  'z';

	for(int i=0; i<7; i++) {
		mySPI_sendControl(disp[i], 0x40);
	}
	trace_printf("\n Freq written\n");
}

// ( AKA_ Write Low part of LCD) takes in resistance and displays it.
void write_Res(float resistance){
	trace_printf("\nWriting on the Resistance\n");

	set_Address(0,1);
	char disp[7];
	char res_buffer[4];
	uint32_t res =(uint32_t) resistance;

	itoa(res,res_buffer,10);	// convert number to string array

	disp[0] =  'R';				// load everything into buffer.
	disp[1] =  ':';
	disp[2] = res_buffer[3];
	disp[3] = res_buffer[2];
	disp[4] = res_buffer[1];
	disp[5] = res_buffer[0];
	disp[6] =  'O';
	disp[7] =  'h';

	for(int i=0; i<7; i++) {
		mySPI_sendControl(disp[i], 0x40);
	}

	trace_printf("\n viva la reistance\n");
}

//Set the address for where to print on the LCD screen
void set_Address(uint8_t row, uint8_t column) {
	trace_printf("\nSetting address...\n");

	uint8_t address = ((row*0x40) | (column+0x80));		//Since row is in 0x40 intervals and in order DDRAM address, DB7 must be 1
	mySPI_sendControl(address, LCD_command);
	
	trace_printf("\nAddress all set\n");
}
// Clear the screen to be able to take new values.
void clear_LCD(){
	trace_printf("\nClearing\n");

	mySPI_sendControl(0x01, LCD_command);			// Clear Display

	trace_printf("\ncleared that display\n");
}

void Delay(uint16_t time){ // delay the system. .... Time is in milliseconds
	trace_printf("\nI'm just stalling. aka Delaying\n");

	uint16_t clock = time*(750); 					// calculate number of clock cycles that correspond to ms time (12000/16).

	TIM3->CNT = clock;    							//Set clock into counter
	TIM3->CR1 |= TIM_CR1_CEN;     					//Enabled counter

	while ((TIM3->CNT & 0xffff) != 0); 				//Loop until counter becomes empty

	TIM3->CR1 &= ~TIM_CR1_CEN;		 				// Restart stopped timer.

	trace_printf("\nYou have been delayed. You are free to go\n");
}

/******************************************** Interrupt Handler Code **************************************************/

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler() {
	trace_printf("\n Tim 2 is Interrupting you\n");
	if ((TIM2->SR & TIM_SR_UIF) != 0) 	{ 				// Check if update interrupt flag is indeed set
		trace_printf("\n*** Overflow! ***\n");
		TIM2->SR &= ~(TIM_SR_UIF);						// Clear update interrupt flag. Relevant register: TIM2->SR. UIF : Update Interrupt flag
		TIM2->CR1 |= TIM_CR1_CEN;		 				// Restart stopped timer. Relevant register: TIM2->CR1
	}
	trace_printf("\nScrew you Tim 2\n");
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler(){
	trace_printf("\n EXTI 1 is Interrupting you\n");
	EXTI->IMR &= ~EXTI_IMR_MR1; 						// Mask EXTI1 interrupt
	/* Your local variables...  */
	freq = 0.00;
	float period = 0.00;

	if ((EXTI->PR & EXTI_PR_PR1) != 0) {  				// Check if EXTI1 interrupt pending flag is indeed set
		edge++;											/* if entered interupt and edge was detected. Therefore to keep track edge counter is incremented.
														If first interrupt thrown, edge = 1 enter first statment. Else its end of signal edge = 2 and enter second statement*/
		if (edge == 1) {									// Check if this is first ege
			TIM2 ->CNT = (uint32_t) 0x0; 				// CLEAR COUNT REGISTER
			TIM2->CR1 |= TIM_CR1_CEN;					// START THE TIMER

		} else {											// Else (this is the second edge):

			TIM2->CR1 &= ~TIM_CR1_CEN;					//	- Stop timer (TIM2->CR1).
			pulse_count = TIM2 -> CNT;					//	- Read out count register (TIM2->CNT).
			freq = ((float) SystemCoreClock)/pulse_count;	//	- Calculate signal frequency.
			period = 1/freq;							//	- Calculate signal period.


			trace_printf("\nThe Signal Parameters are as follow:\n");
			trace_printf("  Signal Frequency: %f Hz\n  Signal Period:    %f sec/cycle\n",freq,period);	// Print calculated frequency and period.
			//write_Freq(freq);
			//write_Res(freq);
			edge = 0;
		}

		EXTI->PR |= EXTI_PR_PR1;				  		// Clear EXTI1 interrupt pending flag (EXTI->PR).
		EXTI->IMR |= EXTI_IMR_MR1; 						// Unmask EXTI1 Flag
	}
	trace_printf("\n Not Today EXTI 1\n");
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

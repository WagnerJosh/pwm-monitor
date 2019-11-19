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
/******************************************** Includes **************************************************/

#include <stdio.h>
#include <stdlib.h>
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

/******************************************** Pragma's **************************************************/

// Sample pragma's to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/******************************************** Defines **************************************************/

#define myTIM2_PRESCALER ((uint16_t)0x0000) 		/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)	 	/* Maximum possible setting for overflow */

#define myTIM3_PRESCALER ((uint16_t)0x40)

//#define SPI_Direction_1Line_Tx ((uint16_t)0xC000)
//#define SPI_Mode_Master ((uint16_t)0x0104)
//#define SPI_DataSize_8b ((uint16_t)0x0700)
//#define SPI_CPOL_Low ((uint16_t)0x0000)
//#define SPI_CPHA_1Edge ((uint16_t)0x0000)
//#define SPI_NSS_Soft SPI_CR1_SSM
//#define SPI_FirstBit_MSB ((uint16_t)0x0000)
//#define SPI_CR1_SSM ((uint16_t)0x0200)

/******************************************** Initialization Functions (MIGHT NOT BE NEEDED)**************************************************/

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void mySPI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void myLCD_Init(void);
void myEXTI_Init(void);

/******************************************** Functions (MIGHT NOT BE NEEDED) **************************************************/
uint32_t ADC_pot();
void mySPI_SendData(uint8_t data);
void mySPI_sendControl(uint8_t address, uint8_t type);
void write_Freq(uint32_t frequency);
void write_Res(uint32_t resistance);
void set_Address(uint8_t row, uint8_t column);
void Delay(uint32_t time);

/******************************************** Global Variables **************************************************/

unsigned int edge = 0; 								// Edge counter
uint32_t pulse_count = 0;
uint32_t adc_offset = 0;
uint8_t LCD_command = 0x00;
uint8_t LCD_char = 0x40;
uint16_t res =0;

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

	myEXTI_Init();		/* Initialize EXTI, this also includes starting the ADC and DAC. */

	while (1){
		// nothing going on here ...
	}
	return 0;
}
/******************************************** Initialization Code **************************************************/

// Checked
void myGPIOA_Init(){
	trace_printf("\nGPIOA Initializing\n");
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 			// Enable clock for GPIOA peripheral. Relevant register: RCC->AHBENR

	// Configure PA0 for potentiometer
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);		// Configure PA0 as input. Relevant register: GPIOA->MODER. [00]
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0); 		// Ensure no pull-up/pull-down for PA1. Relevant register: GPIOA->PUPDR

	// Configure PA1 for 555
	GPIOA->MODER &= ~(GPIO_MODER_MODER1); 		// Configure PA1 as input. Relevant register: GPIOA->MODER. [00]
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);		// Ensure no pull-up/pull-down for PA0. Relevant register: GPIOA->PUPDR

	// Configure PA4 as an analog mode pin for opto
	GPIOA->MODER &= ~(GPIO_MODER_MODER4);		// Configure PA4 as output. Relevant register: GPIOA->MODER. [11]
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);		// Ensure no pull-up/pull-down for PA4. Relevant register: GPIOA->PUPDR
	trace_printf("\nGPIOA Initialized\n");
}

// Checked
// initialize all pins that are associated with the LCD including LCK.
void myGPIOB_Init(){
	trace_printf("\nGPIOB Initializing\n");

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 			// Enable clock for GPIOB peripheral. Relevant register: RCC->AHBENR
	GPIOB->AFR[0] = 0x00;						// Clearing AF(Alternate Functions) register

	// Configure PB3 as SPI-SCK:M21
	GPIOB->MODER |= (GPIO_MODER_MODER3_1);		// Configure PB3 as alternate function. Relevant register: GPIOB->MODER. [11]
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3); 		// Ensure no pull-up/pull-down for PA1. Relevant register: GPIOB->PUPDR
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR3;

	// Configure PB5 as SPI-MOSI:M17
	GPIOB->MODER |= (GPIO_MODER_MODER5_1); 	// Configure PB3 as alternate function. Relevant register: GPIOB->MODER. [11]
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);		// Ensure no pull-up/pull-down for PA0. Relevant register: GPIOB->PUPDR
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR5;

	// Configure PB4 as LCK:M25
	GPIOB->MODER |= (GPIO_MODER_MODER4_0);		// Configure PA4 as output. Relevant register: GPIOA->MODER. [11]
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);		// Ensure no pull-up/pull-down for PA4. Relevant register: GPIOA->PUPDR
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR4;
	trace_printf("\nGPIOB Initialized\n");
}

// Checked
void myADC_Init(){
	trace_printf("\nADC Initializing\n");

	//Enable ADC RCC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;				//Enable clock for ADC peripheral. Relevant register: RCC->APB2ENR

	ADC1->CR = ADC_CR_ADCAL;					// Enable calibration
	while(ADC1->CR == ADC_CR_ADCAL);					// Wait for calibration to complete
	adc_offset = ((ADC1->DR)& ADC_DR_DATA);		// Store calibration factor [6:0] from ADC_DR


	//Configure ADC: continuous and overrun
	ADC1->CFGR1 |= (ADC_CFGR1_DISCEN | ADC_CFGR1_OVRMOD);				//overrun
	//ADC1->CFGR1 |= ADC_CFGR1_DISCEN;				//continuous

	//Select operating channel (output pin) - ADC_IN0
	ADC1->CHSELR = ADC_CHSELR_CHSEL0;				//Channel IN0 is selected to be converted
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;				//Ensure interrupts are disabled

	ADC1->CR |= ADC_CR_ADEN;						//Enabled ADC

	//trace_printf("\nADC IS IN DA HOUSE\n");
	while(!(ADC1->ISR & ADC_ISR_ADRDY));						// True when not ready for conversion
	//trace_printf("\nADC IS IN DA HOUSE\n");
	trace_printf("\nADC Initialized\n");
}

// Checked
// 12-bit -> stored in a left aligned or right aligned 16-bit data register. can set thresholds Upper and lower.
void myDAC_Init(){
	// Enable clock
	trace_printf("\nDAC Initializing\n");
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;				//Enable clock for DAC peripheral. Relevant register: RCC->APB1ENR

	// Enable Output buffer
	//DAC->CR |= DAC_CR_BOFF1; //<- not sure if this is needed

	// Enable DAC
	DAC->CR |= DAC_CR_EN1;
	trace_printf("\nDAC Initialized\n");
}

// Checked
//Initialize SPI by calling all SPI initialization Functions
void mySPI_Init(){
	trace_printf("\nSPI Initializing\n");

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; 		//Enable SPI1 clock

	//Given in class
	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef * SPI_InitStruct = &SPI_InitStructInfo;

	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;	//Biggest pre-scaler
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, &SPI_InitStructInfo);
	SPI_Cmd(SPI1, ENABLE);

	trace_printf("\nSPI Initialized\n");
}

// Checked
//Initialize TIM2 clock to use internal interrupts every second
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

// Checked
//Initialize TIM3 clock to be used for Delay function for LCD
void myTIM3_Init(){
	//trace_printf("\nTIM 3 Initializing\n");

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;			//Enable clock for TIM3 peripheral. Relevant register: RCC->APB1ENR

	TIM3->CR1 = 0x10; 							// Relevant register: TIM3->CR1
	TIM3->PSC = myTIM3_PRESCALER; 				// Set clock prescaler value
	trace_printf("\nTIM 3 Initialized\n");
}

// Checked
// Initialize  LCD. need to change to 4bit Mode
void myLCD_Init(){
	//trace_printf("\nLCD Initializing\n");
	mySPI_sendControl(0x20, LCD_command); // set to 4-bit mode
	Delay(2);
	mySPI_sendControl(0x28, LCD_command);
	Delay(2);
	mySPI_sendControl(0x0C, LCD_command);
	Delay(2);
	mySPI_sendControl(0x06, LCD_command);
	Delay(2);
	mySPI_sendControl(0x01, LCD_command);

	Delay(1);
	trace_printf("\nLCD Initialized\n");
}

// checked but minor difference
void myEXTI_Init(){									// Initializing EXTI
	trace_printf("\nEXTI 1 Initializing\n");
	// Differences
	SYSCFG->EXTICR[0] = 0x0000FF0F;
	//SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA; 	// Map EXTI1 line to PA1. Relevant register: SYSCFG->EXTICR[0]
	EXTI->RTSR |= EXTI_RTSR_TR1; 					// EXTI1 line interrupts: set rising-edge trigger.Relevant register: EXTI->RTSR
	EXTI->IMR |= EXTI_IMR_MR1; 						// Unmask interrupts from EXTI1 line. Relevant register: EXTI->IMR. unmasked so it is not ignored.

	NVIC_SetPriority(EXTI0_1_IRQn , 0); 			// Assign EXTI1 interrupt priority = 0 in NVIC. Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_EnableIRQ(EXTI0_1_IRQn) ; 					// Enable EXTI1 interrupts in NVIC. Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	trace_printf("\nEXTI 1 Initialized\n");
}


/******************************************** ADC and DAC loading Code **************************************************/
//checked
uint32_t ADC_pot(){
	//drink up bitches and start conversion
	ADC1->DR = 0x00;
	ADC1->CR |= ADC_CR_ADSTART;
	//trace_printf("Conversion Therapy...\n");
	//wait for end of conversion
	while(!(ADC1->ISR & ADC_ISR_EOC));
	//trace_printf("... Complete!\n");
	//reset flag
	ADC1->ISR &= ~(ADC_ISR_EOC);

	//push value
	//uint32_t potential = ((ADC1->DR)& ADC_DR_DATA) - adc_offset;
	//trace_printf("\nADC_DR_DATA value: %u\n",ADC_DR_DATA);
	//trace_printf("\nPotential: %u\n", potential);


	DAC->DHR12R1 =( 1350 + (0.666)*ADC1->DR);// potential;
	//Delay(50);
	//trace_printf("\nPotential: %u\n", DAC->DHR12R1);
	return (ADC1->DR);
	// CONVERSION COMPLETE WELCOME JESUS YOU HAVE UNLOCKED YOUR POTENTIAL
}


/******************************************** SPI Code **************************************************/
//checked
void mySPI_SendData(uint8_t data) {
	//trace_printf("\nSending DATA ..... \n");

	GPIOB->BRR = GPIO_Pin_4;					/* Force you LCK signal to 0 */

	/* Wait until SPI1 is ready (TXE = 1 or BSY = 0) */
	while(((SPI1->SR & SPI_SR_TXE) == 0) & ((SPI1->SR & SPI_SR_BSY) == 1));

	SPI_SendData8(SPI1, data);					/* Assumption: your data holds 8 bits to be sent*/

	while((SPI1->SR & SPI_SR_BSY) == 1);		/* Wait until SPI1 is not busy (BSY = 0) */

	//Delay(1);
	GPIOB->BSRR = GPIO_Pin_4;					/* Force your LCK signal to 1 */
	//Delay(2);
	//trace_printf("\nSend DATA Success.\n");
}

//checked
void mySPI_sendControl(uint8_t location, uint8_t type){ //sends LCD control commands including addressing, clearing,
	//trace_printf("\nSend Control start \n");
	uint8_t lowhalf = (0x0F & location); 		// generate lowhalf address
	uint8_t highhalf = (0x0F & (location >> 4)); 	// generate highhalf address. Shifted 4 to left


	mySPI_SendData((type | highhalf)); 	// disable LCD, push type, highhalf,
	mySPI_SendData(((type | 0x80)| highhalf)); 	// enable LCD, push type, highhalf,
	mySPI_SendData((type | highhalf)); 	// disable LCD, push type, highhalf,

	mySPI_SendData((type | lowhalf)); 	// disable LCD, push type, highhalf,
	mySPI_SendData(((type |0x80) | lowhalf)); 	// enable LCD, push type, highhalf,
	mySPI_SendData((type | lowhalf)); 	// disable LCD, push type, highhalf,

	//trace_printf("\nThe send has been Controlled.\n");
}

/******************************************** LCD Helper Functions **************************************************/
//checked minor differnce in loading freq
void write_Freq(uint32_t frequency){ // ( AKA_ write High Part of LCD) takes in frequency and displays it.

	// ADD STATEMENT FOR FREQ = 0 CASE.
	//trace_printf("\nWriting on the Frequency\n");
	set_Address(0,0);
	char disp[9];
	char freq_buffer[4];
	uint32_t temp =(uint32_t) frequency;

	itoa(temp,freq_buffer,10);		// convert number to string array

	disp[0] =  'F';					// load everything into buffer.
	disp[1] =  ':';
	disp[2] = 	freq_buffer[0];
	disp[3] = 	freq_buffer[1];
	disp[4] = 	freq_buffer[2];
	disp[5] = 	freq_buffer[3];
	disp[6] =  'H';
	disp[7] =  'z';

	for(int i=0; i<8; i++) {
		//trace_printf("\n DISP: %c \n", disp[i]);
		mySPI_sendControl(disp[i], 0x40);
	}

}

// ( AKA_ Write Low part of LCD) takes in resistance and displays it.
//Checked
void write_Res(uint32_t resistance){ // add statement for res=0 case.
	set_Address(1,0);
	char disp[8];
	char res_buffer[4];
	uint32_t res =(uint32_t) resistance;

	itoa(res,res_buffer,10);	// convert number to string array

	disp[0] =  'R';				// load everything into buffer.
	disp[1] =  ':';
	disp[2] = res_buffer[0];
	disp[3] = res_buffer[1];
	disp[4] = res_buffer[2];
	disp[5] = res_buffer[3];
	disp[6] =  'O';
	disp[7] =  'h';

	for(int i=0; i<8; i++) {
		//trace_printf("\n RES: %c ", disp[i]);
		mySPI_sendControl(disp[i], 0x40);
	}


}

//Set the address for where to print on the LCD screen
//checked
void set_Address(uint8_t row, uint8_t column) {

	uint8_t address = ((row*0x40) | (0x80+column));		//Since row is in 0x40 intervals and in order DDRAM address, DB7 must be 1
	mySPI_sendControl(address, LCD_command);
}

//checked
void Delay(uint32_t time){ // delay the system. .... Time is in milliseconds
	//trace_printf("\nI'm just stalling. aka Delaying\n");

	uint16_t clock = time*(750); 					// calculate number of clock cycles that correspond to ms time (12000/16).

	TIM3->CNT = clock;    							//Set clock into counter
	TIM3->CR1 |= TIM_CR1_CEN;     					//Enabled counter

	while ((TIM3->CNT & 0xFFFF) != 0x00); 				//Loop until counter becomes empty

	TIM3->CR1 &= ~TIM_CR1_CEN;		 				// Restart stopped timer.

	//trace_printf("\nYou have been delayed. You are free to go\n");
}

/******************************************** Interrupt Handler Code **************************************************/

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler() {

	if ((TIM2->SR & TIM_SR_UIF) != 0) 	{ 				// Check if update interrupt flag is indeed set
		trace_printf("\n*** Overflow! ***\n");
		TIM2->SR ^= (TIM_SR_UIF);						// Clear update interrupt flag. Relevant register: TIM2->SR. UIF : Update Interrupt flag
		TIM2->CR1 |= 0x01;		 				// Restart stopped timer. Relevant register: TIM2->CR1
	}

}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler(){
	//trace_printf("\n EXTI 1 is Interrupting you\n");
	//EXTI->IMR &= ~EXTI_IMR_MR1; 						// Mask EXTI1 interrupt

	uint32_t freq = 0;
	uint32_t pulse_count = 0;

	if ((EXTI->PR & EXTI_PR_PR1) != 0) {  				// Check if EXTI1 interrupt pending flag is indeed set
														// if entered interupt and edge was detected. Therefore to keep track edge counter is incremented.
														//If first interrupt thrown, edge = 1 enter first statment. Else its end of signal edge = 2 and enter second statement
		if (edge == 0) {									// Check if this is first ege
			TIM2 ->CNT = 0; 				// CLEAR COUNT REGISTER
			TIM2 ->CR1 |= TIM_CR1_CEN;					// START THE TIMER
			edge =1;

		} else {											// Else (this is the second edge):
			edge = 0;
			pulse_count = TIM2 -> CNT;					//	- Read out count register (TIM2->CNT).
			TIM2->CR1 = 0;					//	- Stop timer (TIM2->CR1).
			//trace_printf("  Pulses: %u \n \n",pulse_count);
			freq = (SystemCoreClock)/pulse_count;	//	- Calculate signal frequency.

			//trace_printf("\nThe Signal Parameters are as follow:\n");
			//trace_printf("  Signal Frequency: %u Hz\n \n",freq);	// Print calculated frequency and period.

			write_Freq(freq);
			res = ADC_pot();
			write_Res(res);

		}

		EXTI->PR ^= 0x20;				  		// Clear EXTI1 interrupt pending flag (EXTI->PR).
		//EXTI->IMR |= EXTI_IMR_MR1; 						// Unmask EXTI1 Flag
	}
	//trace_printf("\n YOU WILL DO AMAZING\n");
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

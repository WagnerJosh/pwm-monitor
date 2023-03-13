//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: CENG 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include <stdlib.h>
#include "stm32f0xx_spi.h"


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

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)


// initialization functions

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void mySPI_Init(void);
void myLCD_Init(void);

// Utility Functions

void mySPI_send(uint8_t);		// Latches a single instruction to LCD
void mySPI_send3(uint8_t, uint8_t type);		// sends two half-words to LCD (6 latches)
uint32_t get_ADC(void);			// Reads value of POT from ADC
void myWait(uint32_t ms);		// stalls for (ms) milliseconds using general purpose timer TIM3
void myLCD_set_adr(uint8_t row, uint8_t col);
void myLCD_write_R(uint32_t resistance);
void myLCD_write_f(uint32_t frequency);

// Gloabl Variables:

uint16_t firstedge = 0;
uint16_t potValue = 0;
uint8_t LCD_command = 0x00;
uint8_t LCD_char = 0x40;
#define LCK_PIN GPIO_Pin_4;

// Your global variables...


int main(int argc, char* argv[]){

	trace_printf("This is Part 2 of Introductory Lab...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myGPIOB_Init();		// initialize GPIOB
	myTIM3_Init();
	mySPI_Init();		// initialize SPI
	myADC_Init();		// Initialize ADC1
	myDAC_Init();		// initialize DAC
	myLCD_Init();
	//myEXTI_Init();		/* Initialize EXTI */


	// put placeholder f: and R: on screen.
	myLCD_set_adr(0, 0);
	mySPI_send3('F', LCD_char);
	mySPI_send3(':', LCD_char);
	myLCD_set_adr(0, 6);
	mySPI_send3('H', LCD_char);
	mySPI_send3('z', LCD_char);

	myLCD_set_adr(1, 0);
	mySPI_send3('R', LCD_char);
	mySPI_send3(':', LCD_char);

	myLCD_set_adr(1, 6);
	mySPI_send3('O', LCD_char);
	mySPI_send3('h', LCD_char);

	myLCD_write_R(1234);

	myEXTI_Init();

	while (1)
	{

	}

	return 0;
}

void myGPIOA_Init(){

	// Connections:
	// 555 Output -> PA1
	// POT -> PA0
	// PA4 -> Optocoupler Input

	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR   (manual pg 120)
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA1 as input for signal detection.
	// Relevant register: GPIOA->MODER (manual 159)
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	GPIOA->MODER &= ~(GPIO_PUPDR_PUPDR1);

	// Use PA0 for ADC input
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

	// Use PA4 for DAC output
	// Correct code should be 'GPIOA->MODER &= (GPIO_MODER_MODER4);'
	// However, this seems to cause JTAG failures on launch.
	GPIOA->MODER |= (GPIO_MODER_MODER4);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

	trace_printf("GPIOB initialized\n");

}

void myGPIOB_Init()
{
	// Pins defined here are used for SPI communications.

	// Connections:
	// PB3 -> M21
	// PB4 -> M25
	// PB5 -> M17

	/* Enable clock for GPIOB peripheral */
	// Relevant register: RCC->AHBENR   (manual pg 120)
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	GPIOB->AFR[0] = 0x0000;			// Clear alternate function register

	// Set up PB3 - AF0 - SPI1-SCK
	GPIOB->MODER |= (GPIO_MODER_MODER3_1);  // AF0
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);	// no pu/pd
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR3;

	// Set up PB4 - SPI LCK (Manual Control)
	GPIOB->MODER |= (GPIO_MODER_MODER4_0);  //General output
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);	// no pu/pd
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR5;

	// Set up PB5 - AF0 - SPI1-MOSI
	GPIOB->MODER |= (GPIO_MODER_MODER5_1);  // AF0
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);	// no pu/pd
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR5;



	trace_printf("GPIOB initialized\n");

}

void myADC_Init()
{

	// Enable Clock for ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;    // set ADC Clock Enable Bit High

    ADC1->CR = ADC_CR_ADCAL;
    while (ADC1->CR == ADC_CR_ADCAL) {};

	// Enable ADC continuous mode w/ overrun
	ADC1->CFGR1 |= (ADC_CFGR1_DISCEN | ADC_CFGR1_OVRMOD);

	// Set ADC1 to only use channel 1
	ADC1->CHSELR = ADC_CHSELR_CHSEL0;

	ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN; 	// disable interrupts

	ADC1->CR |= ADC_CR_ADEN; 		// set ADC Enable bit to 1
	while(!(ADC1->ISR & ADC_ISR_ADRDY)){};

	trace_printf("ADC Initialized\n");

}

void myDAC_Init()
{

	// Enables the DAC.
	// After this has been run, any value placed in DAC->DHR12R1
	// will be immeadiately converted to an analog voltage on PA4.

	RCC->APB1ENR |=  RCC_APB1ENR_DACEN;    // set DAC Clock Enable Bit High

	DAC->CR |= DAC_CR_EN1; // Enable DAC1

	trace_printf("DAC Initialized\n");

}

void mySPI_Init()
{

	RCC->APB2ENR |=  RCC_APB2ENR_SPI1EN;    // set SPI Clock Enable Bit High

	SPI_InitTypeDef	SPI_InitStructInfo;
	SPI_InitTypeDef	* SPI_InitStruct = &SPI_InitStructInfo;

	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, &SPI_InitStructInfo);

	SPI_Cmd(SPI1, ENABLE);

	trace_printf("SPI Initialized\n");

}

void myLCD_Init()
{
	// LC Initialization:
	// Set LCD to 4-Bit Mode.

	mySPI_send(0x03);
	mySPI_send(0x83);
	mySPI_send(0x03);

	myWait(6);

	mySPI_send(0x03);
	mySPI_send(0x83);
	mySPI_send(0x03);

	myWait(2);

	mySPI_send(0x03);
	mySPI_send(0x83);
	mySPI_send(0x03);

	mySPI_send(0x02);
	mySPI_send(0x82);
	mySPI_send(0x02);

	//mySPI_send3(0x01, LCD_command);			// 2 lines of 8 characters

	mySPI_send3(0x28, LCD_command);			// 2 lines of 8 characters
	mySPI_send3(0x0C, LCD_command);			// Display On
	mySPI_send3(0x06, LCD_command);			// Auto-increment DRAM
	mySPI_send3(0x01, LCD_command);			// Clear Display

	trace_printf("LCD Initialized\n");

}

void mySPI_send(uint8_t message){

	// Sends a single message to 74HC595 as outlined in interfacing slides.

	// force lck to 0
	GPIOB->BRR = LCK_PIN;	// Set Bit-Set Bit Low

	// wait for SPI to deassert busy
	while(((SPI1->SR & SPI_SR_BSY) == 1) & ((SPI1->SR & SPI_SR_TXE) == 0)){	};

	SPI_SendData8(SPI1, message);

	while((SPI1->SR & SPI_SR_BSY) == 1){}; // wait for SPI to deassert busy

	myWait(1);

	//force lck to 1
	GPIOB->BSRR = LCK_PIN;		// Set bit-set bit high

	myWait(0x2);				// wait 2ms for LCD response

}

void mySPI_send3(uint8_t c, uint8_t type){

	uint8_t lowhalf = (0x0F & c);
	uint8_t highhalf = (0x0F & (c >> 4));

	// execute 3 SPI sends for each half of the character to be sent.

	mySPI_send((type | highhalf));
	mySPI_send(((type | 0x80) | highhalf));
	mySPI_send((type | highhalf));

	mySPI_send((type | lowhalf));
	mySPI_send(((type | 0x80) | lowhalf));
	mySPI_send((type | lowhalf));


}

uint32_t get_ADC(){

	// Returns the Value of the ADC as a 12 bit number (0-4096)
	// This WIL NOT WORK if the grounds are not connected between STM32 and PBMCUSLK.
	// Also loads ADC reading into DAC buffer.

	ADC1->DR = 0x0;  // clear ADC output
	ADC1->CR |= ADC_CR_ADSTART; // Start ADC
	while ((ADC1->ISR & ADC_ISR_EOC) == 0){}; // Wait for end of Conversion
	ADC1->ISR &= ~(ADC_ISR_EOC); // clear end of conversion flag

	// 1350 + 0.6666*(ADC Value) roughly maps output to 1-3.3V for enhanced 4N35 Resolution.

	DAC->DHR12R1 = (1350 + (0.6666 * (ADC1->DR)));	// load into DAC buffer.
	myWait(50);
	return(ADC1->DR);	// return value
}

void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;


	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;

	trace_printf("TIM2 initialized\n");
}

void myTIM3_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	/* Configure TIM2: count down*/
	// Relevant register: TIM2->CR1
	TIM3->CR1 = 0x10;

	/* Set clock prescaler value */
	TIM3->PSC = 0x40; 	// timer frequency = 1000Hz. 1 Cycle = 1ms.

	trace_printf("TIM3 initialized\n");
}

void myWait(uint32_t ms){     // load counter, start it and wait until empty

	uint16_t time = (ms*(12000/16));
	TIM3->CNT = time;			// initialize counter
	TIM3->CR1 |= TIM_CR1_CEN;	// start
	while((TIM3->CNT & 0xFFFF) != 0x000){};   // wait until counter is empty
	TIM3->CR1 &= ~TIM_CR1_CEN;    // disable counter
}

/// LCD Utilities:

void myLCD_set_adr(uint8_t row, uint8_t col){
	uint8_t address = ((row*0x40) | (0x80+col));
	mySPI_send3(address, LCD_command);
}

// myLCD_write_f(): Convert integer to string and print on display row 1

void myLCD_write_f(uint32_t frequency){
	myLCD_set_adr(0, 2);
	char buffer[5];
	itoa(frequency, buffer, 10);
	if(frequency < 1000){
		buffer[3] = ' ';
	}
	uint8_t i = 0;
	while(i < 4){
		mySPI_send3(buffer[i], LCD_char);
		i++;
	}

}

// myLCD_write_R(): Convert integer to string and print on display row 2

void myLCD_write_R(uint32_t resistance){
	myLCD_set_adr(1, 2);
	char buffer[5];
	itoa(resistance, buffer, 10);
	uint8_t i = 0;
	if(resistance < 1000){
		buffer[3] = ' ';
	}
	if(resistance < 100){
		buffer[2] = ' ';
	}
	if(resistance < 10){
		buffer[1] = ' ';
	}

	while(i < 4){
		mySPI_send3(buffer[i], LCD_char);
		i++;
	}

}



void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] &= 0x0000FF0F;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	trace_printf("EXTI initialized\n");
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR ^= TIM_SR_UIF;

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= 0x01;
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	// Your local variables...

	uint32_t frequency = 0;
	uint32_t duration = 0;

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		//
		// 1. If this is the first edge:
		//	- Clear count register (TIM2->CNT).
		//	- Start timer (TIM2->CR1).
		//    Else (this is the second edge):
		//	- Stop timer (TIM2->CR1).
		//	- Read out count register (TIM2->CNT).
		//	- Calculate signal period and frequency.
		//	- Print calculated values to the console.
		//	  NOTE: Function trace_printf does not work
		//	  with floating-point numbers: you must use
		//	  "unsigned int" type to print your signal
		//	  period and frequency.
		//
		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		//

		if(firstedge == 0){
			TIM2->CNT = 0x00000000;
			TIM2->CR1 |= TIM_CR1_CEN;
			firstedge = 1;

		}

		else{
			duration = TIM2->CNT;
			TIM2->CR1 = 0x0000;
			firstedge = 0;
			frequency = (double)SystemCoreClock / (double)duration;

			myLCD_write_f(frequency);
			potValue = get_ADC();
			myLCD_write_R(potValue*1.2);

		}

		EXTI->PR ^= 0x20; // reset flag.

	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

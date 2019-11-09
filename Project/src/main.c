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

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

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


#define myTIM2_PRESCALER ((uint16_t)0x0000) 		/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)	 	/* Maximum possible setting for overflow */
#define myTIM3_PRESCALER ((uint16_t)0x0000)
#define myTIM3_PERIOD ((uint32_t)0xFFFFFFFF)


void myGPIOA_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);		/* Initialize ADC */
void myDAC_Init(void);

/* Your global variables... */
unsigned int edge = 0; 								// Edge counter
uint32_t pulse_count = 0;
uint32_t adc_offset = 0;

//uint32_t ADC_MAX_Value = ;  						//ADC supply requirements: 2.4 V to 3.6 V
//uint32_t ADC_resolution = 4096 ;

//uint32_t DAC_MAX_Value = ;
//uint32_t DAC_resolution = ;

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

int main(int argc, char* argv[]) {

	trace_printf("This is the Final Project...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	myADC_Init();		/* Initialize ADC */
	myDAC_Init();		/* Initialize DAC */

	while (1){

		uint32_t input = ADC_pot();
		trace_printf("\nADC Value: %u\n", input);

		/* Store DAC Value in Output buffer */
		DAC->DHR12R1 = input;
		trace_printf("\nDAC Output buffer: %u\n", DAC->DHR12R1);
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

void myGPIOA_Init(){

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

void myTIM2_Init(){

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
}

//NEED TO ADJUST VALUES
void myTIM3_Init(){

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;			//Enable clock for TIM2 peripheral. Relevant register: RCC->APB1ENR

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	   enable update events, interrupt on overflow only */

	TIM3->CR1 = ((uint16_t)0x008C); 			// Relevant register: TIM2->CR1
	TIM3->PSC = myTIM3_PRESCALER; 				// Set clock prescaler value
	TIM3->ARR = myTIM3_PERIOD; 					// Set auto-reloaded delay
	TIM3->EGR = ((uint16_t)0x0001); 			// Update timer registers.  Relevant register: TIM2->EGR

	NVIC_SetPriority(TIM3_IRQn, 0); 			// Relevant register: NVIC->IP[3], or use NVIC_SetPriority. Assign TIM2 interrupt priority = 0 in NVIC.
	NVIC_EnableIRQ(TIM3_IRQn); 					// Enable TIM2 interrupts in NVIC. Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ.

	TIM3->DIER |= TIM_DIER_UIE; 				// Enable update interrupt generation  Relevant register: TIM2->DIER.

	TIM3->CR1 |= TIM_CR1_CEN; 				//They added a line to activate timer, not sure if necessary
}

//create tim3 initialization
//replicate isr for tim3
// using tim 3 because it is general purpose like tim2
// we need to put custom scalers
	//prescaler and autoreload must be set

void myEXTI_Init(){								// Initializing EXTI

	SYSCFG->EXTICR[0]= SYSCFG_EXTICR1_EXTI1_PA; // Map EXTI1 line to PA1. Relevant register: SYSCFG->EXTICR[0]
	EXTI->RTSR |= EXTI_RTSR_TR1; 				// EXTI1 line interrupts: set rising-edge trigger.Relevant register: EXTI->RTSR
	EXTI->IMR |= EXTI_IMR_MR1; 					// Unmask interrupts from EXTI1 line. Relevant register: EXTI->IMR. unmasked so it is not ignored.

	NVIC_SetPriority(EXTI0_1_IRQn , 0); 		// Assign EXTI1 interrupt priority = 0 in NVIC. Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_EnableIRQ(EXTI0_1_IRQn) ; 				// Enable EXTI1 interrupts in NVIC. Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
}

void myADC_Init(){		// 12-bit -> stored in a left aligned or right aligned 16-bit data register. can set thresholds Upper and lower.
	trace_printf("\nADC knock knock\n");
	//Enable ADC RCC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;				//Enable clock for ADC peripheral. Relevant register: RCC->APB2ENR

	if(ADC_CR_ADEN == 0x00) {						// If calibration is not complete
		trace_printf("\nADC unlock that door\n");
		ADC1->CR |= ADC_CR_ADCAL;					// Enable calibration
		while(ADC_CR_ADCAL == 1);					// Wait for calibration to complete
		adc_offset = ((ADC1->DR)& ADC_DR_DATA);					// Store calibration factor [6:0] from ADC_DR
		trace_printf("\nADC SHE UNLOCKED ENTER AT OWN RISK\n");
	}

	//Configure ADC: continuous and overrun
	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;				//overrun
	ADC1->CFGR1 |= ADC_CFGR1_CONT;					//continuous
	//ADC1->CFGR1 |= ADC_CFGR1_RES;					//set resolution (maybe to 12 = [00]) (I think it is already set there?)

	//Select operating channel (output pin) - ADC_IN0
	ADC1->CHSELR |= ADC_CHSELR_CHSEL0;				//Channel IN0 is selected to be converted
	ADC1->CR |= ADC_CR_ADEN;						//Enabled ADC

	// Waiting for ADC to be ready for conversion
	trace_printf("\nADC TEEEHEE\n");
	while(ADC_ISR_ADRDY == 0);						// True when not ready for conversion
	trace_printf("\nADC IS IN DA HOUSE\n");
}

void myDAC_Init(){
	// Enable clock
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;			//Enable clock for DAC peripheral. Relevant register: RCC->APB1ENR

	// Enable Output buffer
	DAC->CR |= DAC_CR_BOFF1;

	// Enable DAC
	DAC->CR |= DAC_CR_EN1;
}



uint32_t freq_generator(uint32_t value) {
	/*If this function is not operating correctly may want to offset the output range by the diode drop across the
	 * Optocoupler ~ 0.6V for VBE and 0.6V for VBC and 0.229V from AC
	 */
	return value;
}


void TIM2_IRQHandler() {	/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */

	if ((TIM2->SR & TIM_SR_UIF) != 0) 	{ 		// Check if update interrupt flag is indeed set
		trace_printf("\n*** Overflow! ***\n");
		TIM2->SR &= ~(TIM_SR_UIF);				// Clear update interrupt flag. Relevant register: TIM2->SR. UIF : Update Interrupt flag
		TIM2->CR1 |= TIM_CR1_CEN;		 		// Restart stopped timer. Relevant register: TIM2->CR1
	}
}
//TIMER FOR LCD
void TIM3_IRQHandler() {	/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */

	if ((TIM3->SR & TIM_SR_UIF) != 0) 	{ 		// Check if update interrupt flag is indeed set
		trace_printf("\n*** Overflow! ***\n");
		TIM3->SR &= ~(TIM_SR_UIF);				// Clear update interrupt flag. Relevant register: TIM3->SR. UIF : Update Interrupt flag
		TIM3->CR1 |= TIM_CR1_CEN;		 		// Restart stopped timer. Relevant register: TIM3->CR1
	}
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler(){

	EXTI->IMR &= ~EXTI_IMR_MR1; 							// Mask EXTI1 interrupt
	/* Your local variables...  */
	float freq = 0.00;
	float period = 0.00;

	if ((EXTI->PR & EXTI_PR_PR1) != 0) {  				// Check if EXTI1 interrupt pending flag is indeed set
		edge++;											/* if entered interupt and edge was detected. Therefore to keep track edge counter is incremented.
														If first interrupt thrown, edge = 1 enter first statment. Else its end of signal edge = 2 and enter second statement*/
		if(edge == 1){									// Check if this is first ege

			TIM2 ->CNT = (uint32_t) 0x0; 				// CLEAR COUNT REGISTER
			TIM2->CR1 |= TIM_CR1_CEN;					// START THE TIMER

		}else{											// Else (this is the second edge):

			TIM2->CR1 &= ~TIM_CR1_CEN;					//	- Stop timer (TIM2->CR1).
			pulse_count = TIM2 -> CNT;					//	- Read out count register (TIM2->CNT).
			freq = ((float) SystemCoreClock)/pulse_count;	//	- Calculate signal frequency.
			period = 1/freq;							//	- Calculate signal period.

			// PRINT STATMENTS
			// NOTE: Function trace_printf does not work with floating-point numbers: you must use "unsigned int" type
			// to print your signal period and frequency.

			trace_printf("\nThe Signal Parameters are as follow:\n");
			trace_printf("  Signal Frequency: %f Hz\n  Signal Period:    %f sec/cycle\n",freq,period);	// Print calculated frequency and period.

			edge = 0;
		}
			EXTI->PR |= EXTI_PR_PR1;				  	// Clear EXTI1 interrupt pending flag (EXTI->PR).
			EXTI->IMR |= EXTI_IMR_MR1; 					// Unmask EXTI1 Flag
	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

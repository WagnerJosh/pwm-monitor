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

void myGPIOA_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);

/* Your global variables... */
unsigned int edge = 0; 								// Edge counter
uint32_t pulse_count = 0;
uint32_t res = 0;

int main(int argc, char* argv[]) {

	trace_printf("This is Part 2 of Introductory Lab...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */

	while (1){
		// We will be measuring the analog value into Pin 0 here.
		// We could technically do this as a function and just call it here.
		int res = myADC();
				// Nothing is going on here...
	}

	return 0;

}

void myGPIOA_Init(){

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 			// Enable clock for GPIOA peripheral. Relevant register: RCC->AHBENR
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);		// Configure PA0 as input. Relevant register: GPIOA->MODER. [00]
	GPIOA->MODER &= ~(GPIO_MODER_MODER1); 		// Configure PA1 as input. Relevant register: GPIOA->MODER. [00]
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1); 		// Ensure no pull-up/pull-down for PA1. Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);		// Ensure no pull-up/pull-down for PA0. Relevant register: GPIOA->PUPDR
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

void myEXTI_Init(){							// Initializing EXTI

	SYSCFG->EXTICR[0]= SYSCFG_EXTICR1_EXTI1_PA; // Map EXTI1 line to PA1. Relevant register: SYSCFG->EXTICR[0]
	EXTI->RTSR |= EXTI_RTSR_TR1; 				// EXTI1 line interrupts: set rising-edge trigger.Relevant register: EXTI->RTSR
	EXTI->IMR |= EXTI_IMR_MR1; 					// Unmask interrupts from EXTI1 line. Relevant register: EXTI->IMR. unmasked so it is not ignored.

	NVIC_SetPriority(EXTI0_1_IRQn , 0); 		// Assign EXTI1 interrupt priority = 0 in NVIC. Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_EnableIRQ(EXTI0_1_IRQn) ; 				// Enable EXTI1 interrupts in NVIC. Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
}

void myADC_Init(){		// 12-bit -> stored in a left aligned or right aligned 16-bit data register. can set thresholds Upper and lower.
	// Start ADC Clock
	// Calibrate ADC. Removes offset error . need to have ADC disabled ( ADEN =0);
		// wait for ADRDY Flag
	// We get Calibration factor ( stored in ADC_DR from bits [6:0])

	// ADC needs a Stabilization time t_stab before it starts converting accurately. !!! Factor in ( while x flag = X)

	/*
	 * Follow this procedure to enable the ADC:
		1. Set ADEN=1 in the ADC_CR register.
		2. Wait until ADRDY=1 in the ADC_ISR register (ADRDY is set after the ADC startup
			time). This can be handled by interrupt if the interrupt is enabled by setting the
			ADRDYIE bit in the ADC_IER register.

		Follow this procedure to disable the ADC:
		1. Check that ADSTART=0 in the ADC_CR register to ensure that no conversion is
			ongoing. If required, stop any ongoing conversion by writing 1 to the ADSTP bit in the
			the ADC_CR register and waiting until this bit is read at 0.
		2. Set ADDIS=1 in the ADC_CR register.
		3. If required by the application, wait until ADEN=0 in the ADC_CR register, indicating that
			the ADC is fully disabled (ADDIS is automatically reset once ADEN=0)
	 *
	 */


}
void myDAC_Init(){
	// Initialize DAC
}
int myADC(){
	// ONLY DELETE ONCE WE HAVE INITIALIZED PIN 0.

}
void TIM2_IRQHandler() {	/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */

	if ((TIM2->SR & TIM_SR_UIF) != 0) 	{ 		// Check if update interrupt flag is indeed set
		trace_printf("\n*** Overflow! ***\n");
		TIM2->SR &= ~(TIM_SR_UIF);				// Clear update interrupt flag. Relevant register: TIM2->SR. UIF : Update Interrupt flag
		TIM2->CR1 |= TIM_CR1_CEN;		 		// Restart stopped timer. Relevant register: TIM2->CR1
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

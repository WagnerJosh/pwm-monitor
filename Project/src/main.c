//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//
// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------
//
/******************************************** Includes **************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"

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

#define myTIM3_PRESCALER ((uint16_t)0x40)           /* Clock prescaler for TIM3 timer: 64 */

/******************************************** Initialization Functions **************************************************/

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void mySPI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void myLCD_Init(void);
void myEXTI_Init(void);

/******************************************** Functions  **************************************************/

uint32_t ADC_pot();
void mySPI_SendData(uint8_t data);
void mySPI_sendControl(uint8_t address, uint8_t type);
void write_Freq(uint32_t frequency);
void write_Res(uint32_t resistance);
void set_Address(uint8_t row, uint8_t column);
void Delay(uint32_t time);

/******************************************** Global Variables **************************************************/

unsigned int edge = 0; 			// Keeps track of waveform edges
uint32_t pulse_count = 0;       // Keeps track of pulses
uint32_t adc_offset = 0;        // Stores adc calibration offset
uint8_t LCD_command = 0x00;     // Tells mySPI_sendControl function to send command
uint8_t LCD_char = 0x40;        // Tells mySPI_sendControl function to send character
uint16_t res =0;                // Stores resistance
uint32_t freq = 0;              // Stores frequency

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

void myGPIOA_Init(){
    trace_printf("\n GPIOA Initializing \n");

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 			    // Enable clock for GPIOA peripheral. Relevant register: RCC->AHBENR

    // Configure PA0 for potentiometer
    GPIOA->MODER &= ~(GPIO_MODER_MODER0);		    // Configure PA0 as input. Relevant register: GPIOA->MODER. [00]
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0); 		    // Ensure no pull-up/pull-down for PA1. Relevant register: GPIOA->PUPDR

    // Configure PA1 for 555
    GPIOA->MODER &= ~(GPIO_MODER_MODER1); 		    // Configure PA1 as input. Relevant register: GPIOA->MODER. [00]
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);		    // Ensure no pull-up/pull-down for PA0. Relevant register: GPIOA->PUPDR

    // Configure PA4 as an analog mode pin for optocoupler
    GPIOA->MODER &= ~(GPIO_MODER_MODER4);		    // Configure PA4 as output. Relevant register: GPIOA->MODER. [11]
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);		    // Ensure no pull-up/pull-down for PA4. Relevant register: GPIOA->PUPDR

    trace_printf(" GPIOA Initialized\n");
}

// initialize all pins that are associated with the LCD including LCK.
void myGPIOB_Init(){
    trace_printf("\n GPIOB Initializing\n");

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 			    // Enable clock for GPIOB peripheral. Relevant register: RCC->AHBENR
    GPIOB->AFR[0] = 0x00;						    // Clearing AF(Alternate Functions) register

    // Configure PB3 as SPI-SCK:M21
    GPIOB->MODER |= (GPIO_MODER_MODER3_1);		    // Configure PB3 as alternate function. Relevant register: GPIOB->MODER. [11]
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3); 		    // Ensure no pull-up/pull-down for PA1. Relevant register: GPIOB->PUPDR

    // Configure PB5 as SPI-MOSI:M17
    GPIOB->MODER |= (GPIO_MODER_MODER5_1); 	        // Configure PB5 as alternate function. Relevant register: GPIOB->MODER. [11]
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);		    // Ensure no pull-up/pull-down for PA0. Relevant register: GPIOB->PUPDR

    // Configure PB4 as LCK:M25
    GPIOB->MODER |= (GPIO_MODER_MODER4_0);		    // Configure PA4 as output. Relevant register: GPIOA->MODER. [11]
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);		    // Ensure no pull-up/pull-down for PA4. Relevant register: GPIOA->PUPDR
    trace_printf(" GPIOB Initialized\n");
}

void myADC_Init(){
    trace_printf("\n ADC Initializing\n");

    //Enable ADC RCC
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;				// Enable clock for ADC peripheral. Relevant register: RCC->APB2ENR

    ADC1->CR = ADC_CR_ADCAL;						// Enable calibration
    while(ADC1->CR == ADC_CR_ADCAL);				// Wait for calibration to complete
    adc_offset = ((ADC1->DR)& ADC_DR_DATA);			// Store calibration factor [6:0] from ADC_DR


    //Configure ADC: continuous and overrun
    ADC1->CFGR1 |=  ADC_CFGR1_OVRMOD;				// Set overrun
    ADC1->CFGR1 |=  ADC_CFGR1_CONT;				    // Set continuous

    //Select operating channel (output pin) - ADC_IN0
    ADC1->CHSELR |= ADC_CHSELR_CHSEL0;				// Channel IN0 is selected to be converted
    ADC1->CFGR1 &= ~(ADC_CFGR1_EXTEN);				// Ensure interrupts are disabled

    ADC1->CR |= ADC_CR_ADEN;						// Enabled ADC

    while(!(ADC1->ISR & ADC_ISR_ADRDY));			// True when not ready for conversion

    trace_printf(" ADC Initialized \n");
}

void myDAC_Init(){
    trace_printf("\n DAC Initializing\n");

    RCC->APB1ENR |= RCC_APB1ENR_DACEN;				// Enable clock for DAC peripheral. Relevant register: RCC->APB1ENR
    DAC->CR |= DAC_CR_EN1;                          // Enable DAC

    trace_printf(" DAC Initialized\n");
}

void mySPI_Init(){
    trace_printf("\n SPI Initializing\n");

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; 		    // Enable SPI1 clock

    // Code provided by Interface Examples - Our Lab System Lecture Slides
    SPI_InitTypeDef SPI_InitStructInfo;
    SPI_InitTypeDef * SPI_InitStruct = &SPI_InitStructInfo;             // Setup SPI initialization struct

    SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;             // Configure SPI Comm direction
    SPI_InitStruct->SPI_Mode = SPI_Mode_Master;                         // Configure Mode
    SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;                     //  Set  to 8bit data
    SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;                            // Idle Clock
    SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;                          // Active Clock
    SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;                             // Software Slave select
    SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;	// Set to largest pre-scaler
    SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;                    // Most Significant Bit = First Bit
    SPI_InitStruct->SPI_CRCPolynomial = 7;                              // cyclic redundancy

    SPI_Init(SPI1, &SPI_InitStructInfo);                                // Initialize SPI
    SPI_Cmd(SPI1, ENABLE);                                              // Enable SPI

    trace_printf(" SPI Initialized\n");
}

//Initialize TIM2 clock to use internal interrupts every second
void myTIM2_Init(){
    trace_printf("\n TIM 2 Initializing\n");

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;			    //Enable clock for TIM2 peripheral. Relevant register: RCC->APB1ENR

    /* Configure TIM2: buffer auto-reload, count up, stop on overflow,
       enable update events, interrupt on overflow only */

    TIM2->CR1 = ((uint16_t)0x008C); 			    // Relevant register: TIM2->CR1
    TIM2->PSC = myTIM2_PRESCALER; 				    // Set clock prescaler value
    TIM2->ARR = myTIM2_PERIOD; 					    // Set auto-reloaded delay
    TIM2->EGR = ((uint16_t)0x0001); 			    // Update timer registers.  Relevant register: TIM2->EGR

    NVIC_SetPriority(TIM2_IRQn, 0); 			    // Relevant register: NVIC->IP[3], or use NVIC_SetPriority. Assign TIM2 interrupt priority = 0 in NVIC.
    NVIC_EnableIRQ(TIM2_IRQn); 					    // Enable TIM2 interrupts in NVIC. Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ.

    TIM2->DIER |= TIM_DIER_UIE; 				    // Enable update interrupt generation  Relevant register: TIM2->DIER.

    trace_printf(" TIM 2 Initialized\n");
}

//Initialize TIM3 clock to be used for Delay function for LCD
void myTIM3_Init(){
    trace_printf("\n TIM 3 Initializing\n");

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;			    // Enable clock for TIM3 peripheral. Relevant register: RCC->APB1ENR

    TIM3->CR1 = 0x10; 							    // Setting to downcounter
    TIM3->PSC = myTIM3_PRESCALER; 				    // Set clock prescaler value

    trace_printf(" TIM 3 Initialized\n");
}

// Initialize  LCD. need to change to 4bit Mode
void myLCD_Init(){
    trace_printf("\n LCD Initializing\n");

    // Code derived by Interface Examples - Our Lab System Lecture Slides
    mySPI_sendControl(0x20, LCD_command);           // set to 4-bit mode
    Delay(10);                                      // Delay to allow LCD to process
    mySPI_sendControl(0x28, LCD_command);           // Set two 2 lines of 8 characters
    Delay(10);
    mySPI_sendControl(0x0C, LCD_command);           // Display on, No Cursor, Not Blinking
    Delay(10);
    mySPI_sendControl(0x06, LCD_command);           // DRAM set to auto increment, not shifted
    Delay(10);
    mySPI_sendControl(0x01, LCD_command);           // Clear Display
    Delay(10);

    trace_printf(" LCD Initialized\n");
}

void myEXTI_Init(){									// Initializing EXTI
    trace_printf("\n EXTI 1 Initializing\n");

    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA; 	// Map EXTI1 line to PA1. Relevant register: SYSCFG->EXTICR[0]
    EXTI->RTSR |= EXTI_RTSR_TR1; 					// EXTI1 line interrupts: set rising-edge trigger.Relevant register: EXTI->RTSR
    EXTI->IMR |= EXTI_IMR_MR1; 						// Unmask interrupts from EXTI1 line. Relevant register: EXTI->IMR. unmasked so it is not ignored.

    NVIC_SetPriority(EXTI0_1_IRQn , 0); 			// Assign EXTI1 interrupt priority = 0 in NVIC. Relevant register: NVIC->IP[1], or use NVIC_SetPriority
    NVIC_EnableIRQ(EXTI0_1_IRQn) ; 					// Enable EXTI1 interrupts in NVIC. Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
    trace_printf(" EXTI 1 Initialized\n");
}


/******************************************** ADC and DAC loading Code **************************************************/

uint32_t ADC_pot(){

    ADC1->DR = 0x00;                                // Clear ADC
    ADC1->CR |= ADC_CR_ADSTART;			            // Start ADC

    while((ADC1->ISR & ADC_ISR_EOC)==0);            // wait for end of conversion

    ADC1->ISR &= ~(ADC_ISR_EOC);                    // reset ADC flag

    uint32_t potential = (ADC1->DR) - adc_offset;   // apply offset to adc value read

    // Digital to Analog conversion (DAC) Process
    DAC->DHR12R1 = (1350 + (0.666)*potential);	    // loading DAC value in output buffer, offset for Opto coupler drop
    Delay(50);

    return potential;							    // return ADC value
}

/******************************************** SPI Code **************************************************/

void mySPI_SendData(uint8_t data) {

    // Code derived by Interface Examples - Our Lab System Lecture Slides
    GPIOB->BRR = GPIO_Pin_4;					    // Force LCK signal to 0

    /* Wait until SPI1 is ready (TXE = 1 or BSY = 0) */
    while(((SPI1->SR & SPI_SR_TXE) == 0) & ((SPI1->SR & SPI_SR_BSY) == 1));

    SPI_SendData8(SPI1, data);					    // Assumption: your data holds 8 bits to be sent

    while((SPI1->SR & SPI_SR_BSY) == 1);		    // Wait until SPI1 is not busy (BSY = 0)

    Delay(1);
    GPIOB->BSRR = GPIO_Pin_4;					    // Force LCK signal to 1
    Delay(1);

}

void mySPI_sendControl(uint8_t word, uint8_t type){ //Controls LCD. Sends addresses and Characters.

    uint8_t lowhalf = (0x0F & word); 			// generate lowhalf address
    uint8_t highhalf = (0x0F & (word >> 4)); 	// generate highhalf address. Shifted 4 to left


    mySPI_SendData((type | highhalf)); 				// disable LCD, push type, highhalf,
    mySPI_SendData(((type | 0x80)| highhalf)); 		// enable LCD, push type, highhalf,
    mySPI_SendData((type | highhalf)); 				// disable LCD, push type, highhalf,

    mySPI_SendData((type | lowhalf)); 				// disable LCD, push type, lowhalf,
    mySPI_SendData(((type |0x80) | lowhalf));		// enable LCD, push type, lowhalf,
    mySPI_SendData((type | lowhalf)); 				// disable LCD, push type, lowhalf,
}

/******************************************** LCD Helper Functions **************************************************/

void write_Freq(uint32_t frequency){ // ( AKA_ write High Part of LCD) takes in frequency and displays it.

    set_Address(0,0);                               // Set address of LCD to first row, left most block
    char disp[5];                                   // Initialize display buffer
    itoa(frequency, disp, 10);		                // convert number to string array

    if(frequency < 1000){                           // If frequency is less than 1000,  make the thousandth spot be a space
        disp[3] = ' ';
    }

    // Sending frequency characters
    mySPI_sendControl('F', LCD_char);
    mySPI_sendControl(':', LCD_char);

    // Sending individual characters to be displayed
    uint8_t i = 0;
    while(i < 4){
        mySPI_sendControl(disp[i], LCD_char);
        i++;
    }

    // Sending frequency unit characters
    mySPI_sendControl('H', LCD_char);
    mySPI_sendControl('z', LCD_char);
}

void write_Res(uint32_t resistance){

    set_Address(1, 0);                              // Set address of LCD to second row, left most block
    char disp[5];                                   // Initialize display buffer
    itoa(resistance, disp, 10);                     // convert number to string array

    // Determine spaces required to display the resistance
    if(resistance < 1000){                          // If frequency is less than 1000,  make the thousandth spot be a space
        disp[3] = ' ';
    }
    if(resistance < 100){                           // If frequency is less than 100,  make the hundredth spot be a space
        disp[2] = ' ';
    }
    if(resistance < 10){                            // If frequency is less than 10,  make the tenth spot be a space
        disp[1] = ' ';
    }

    // Sending resistance characters
    mySPI_sendControl('R', LCD_char);
    mySPI_sendControl(':', LCD_char);

    // Sending individual characters to be displayed
    uint8_t i = 0;
    while(i < 4){
        mySPI_sendControl(disp[i], LCD_char);
        i++;
    }

    // Sending resistance unit characters
    mySPI_sendControl(' ', LCD_char);
    mySPI_sendControl(0xF4, LCD_char);          //0xF4 corresponds to Ohms symbol
}

//Set the address for where to print on the LCD screen
void set_Address(uint8_t row, uint8_t column) {

    uint8_t address = ((row*0x40) | (column+0x80));		//Since row is in 0x40 intervals and to set DDRAM address, DB7 must be 1
    mySPI_sendControl(address, LCD_command);
}

void Delay(uint32_t time){ // delay the system. .... Time is in milliseconds

    uint16_t clock = time*(750); 					    // calculate number of clock cycles that correspond to ms time (12000/16).

    TIM3->CNT = clock;    							    // Set clock into counter
    TIM3->CR1 |= TIM_CR1_CEN;     					    // Enabled counter

    while ((TIM3->CNT & 0xFFFF) != 0x00); 			    // Loop until counter becomes empty

    TIM3->CR1 &= ~TIM_CR1_CEN;		 				    // Restart stopped timer.
}

/******************************************** Interrupt Handler Code **************************************************/

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler() {

    if ((TIM2->SR & TIM_SR_UIF) != 0) 	{ 				// Check if update interrupt flag is indeed set
        trace_printf("\n*** Overflow! ***\n");
        TIM2->SR &= ~(TIM_SR_UIF);						// Clear update interrupt flag. Relevant register: TIM2->SR. UIF : Update Interrupt flag
        TIM2->CR1 |= TIM_CR1_CEN;		 				// Restart stopped timer. Relevant register: TIM2->CR1
    }
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler(){

    EXTI->IMR &= ~EXTI_IMR_MR1; 						// Mask EXTI1 interrupt

    uint32_t pulse_count = 0;

    if ((EXTI->PR & EXTI_PR_PR1) != 0) {  				// Check if EXTI1 interrupt pending flag is indeed set

        /* If entered interrupt and edge was detected. Therefore to keep track edge counter is incremented.
        If first interrupt thrown, edge = 1 enter first statment. Else its end of signal edge = 2 and enter second statement */

        edge++;

        if (edge == 1) {								// Check if this is first edge
            TIM2 ->CNT = (uint32_t) 0x00; 				// Clear count register
            TIM2 ->CR1 |= TIM_CR1_CEN;					// Start the timer

        } else {										// Else (this is the second edge):
            edge = 0;
            TIM2->CR1 &= ~TIM_CR1_CEN;                  // Stop timer (TIM2->CR1).
            pulse_count = TIM2 -> CNT;					// Read out count register (TIM2->CNT).

            freq = SystemCoreClock/pulse_count;		    // Calculate signal frequency.

            write_Freq(freq);                           // Display frequency on LCD
            Delay(5);
            res = 1.22*ADC_pot(); 						// map the ADC value to the resistance value. 0-> 5K res = 0-> 4096 adc
            write_Res(res);                             // Display resistance on LCD
            Delay(5);
        }

        EXTI->PR |= EXTI_IMR_MR1;				  		// Clear EXTI1 interrupt pending flag (EXTI->PR).
        EXTI->IMR |= EXTI_IMR_MR1; 						// Unmask EXTI1 Flag
    }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

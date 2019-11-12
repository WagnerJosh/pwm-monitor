
/* LCD Code for ECE 355 Project */

#include <stdio.h>

#include "stm32f0xx_rcc.c"
#include "stm32f0xx_gpio.c"
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f0xx_spi.c"


/* Pragma's */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* defines */
#define myTIM14_PRESCALER ((uint16_t)0x0000) 		/* Clock prescaler for TIM14 timer: ADD STUFF */
#define myTIM14_PERIOD ((uint32_t)0xFFFFFFFF)	 	/* Maximum possible setting for overflow */

#define SPI_Direction_1Line_Tx ((uint16_t)0xC000)
#define SPI_Mode_Master ((uint16_t)0x0104)
#define SPI_DataSize_8b ((uint16_t)0x0700)
#define SPI_CPOL_Low ((uint16_t)0x0000)
#define SPI_CPHA_1Edge ((uint16_t)0x0000)
#define SPI_NSS_Soft SPI_CR1_SSM
#define SPI_FirstBit_MSB ((uint16_t)0x0000)

#define SPI_CR1_SSM ((uint16_t)0x0200)

/* Global Variable Declarations */

/* Initialization Code */

void myGPIOB_Init(){
	// initilize all pins that are associated with the LCD including LCK.

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 			// Enable clock for GPIOB peripheral. Relevant register: RCC->AHBENR

    // Configure PB3 as SPI-SCK
    GPIOB->MODER &= ~(GPIO_MODER_MODER3_1);		// Configure PB3 as alternate function. Relevant register: GPIOB->MODER. [11]
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3); 		// Ensure no pull-up/pull-down for PA1. Relevant register: GPIOB->PUPDR

    // Configure PB5 as SPI-MOSI
    GPIOB->MODER &= ~(GPIO_MODER_MODER5_1); 		// Configure PB3 as alternate function. Relevant register: GPIOB->MODER. [11]
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);		// Ensure no pull-up/pull-down for PA0. Relevant register: GPIOB->PUPDR

    // Configure PB4 as LCK
    GPIOB->MODER &= ~(GPIO_MODER_MODER4_0);		// Configure PA4 as output. Relevant register: GPIOA->MODER. [11]
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);		// Ensure no pull-up/pull-down for PA4. Relevant register: GPIOA->PUPDR

}

void mySPI_Init(){
	// call all SPI initialization Functions
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
}

void myLCD_init(){
// Initialize  LCD. need to change to 4bit Mode


}

/* Functions */

void mySPI_SendData(uint8_t data) {
	/* Force you LCK signal to 0 */
	GPIOB->BRR = GPIO_Pin_4;

	/* Wait until SPI1 is ready (TXE = 1 or BSY = 0) */
	while((SPI1->SR & SPI_SR_TXE) == 0x00 || (SPI1->SR & SPI_SR_BSY) != 0x00);

	/* Assumption: your data holds 8 bits to be sent*/
	SPI_SendData(SPI1, data);

	/* Wait until SPI1 is not busy (BSY = 0) */
	while((SPI1->SR & SPI_SR_BSY) != 0x00);

	/* Force your LCK signal to 1 */
	GPIOB->BRR = GPIO_Pin_4;
}


void mySPI_sendcontrol(uint8_t address, uint8_t type){ //sends LCD control commands
  uint8_t

}
void write_Freq(uint32_t frequency){ // ( AKA_ write High Part of LCD) takes in frequency and displays it.
	//take in freq
	//convert to ASCII
	//Write to LCD

  // Set write address location  row* 0x40 column* 0x80
  // initilaize 5 char array
  //load frequency into array
  // send buffer to display
}

void write_Res(uint32_t resistance){ // ( AKA_ Write Low part of LCD) takes in resistance and displays it.
	//take in resistance
	//convert to ASCII
	//Write to LCD

  // set write address

}

void clear_LCD(){
	// Clear the screen to be able to take new values.
  mySPI_send3(0x01, LCD_command);			// Clear Display

}

void Delay_Init() {

    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;			//Enable clock for TIM2 peripheral. Relevant register: RCC->APB1ENR

    /* Configure TIM2: buffer auto-reload, count up, stop on overflow,
       enable update events, interrupt on overflow only */

    TIM14->CR1 = ((uint16_t)0x008C); 			// Relevant register: TIM2->CR1
    TIM14->PSC = myTIM14_PRESCALER; 			// Set clock prescaler value
    TIM14->ARR = myTIM14_PERIOD; 					// Set auto-reloaded delay
    TIM14->EGR = ((uint16_t)0x0001); 			// Update timer registers.  Relevant register: TIM2->EGR

    //MAYBE CHANGE PRIORITY
    NVIC_SetPriority(TIM14_IRQn, 0); 			// Relevant register: NVIC->IP[3], or use NVIC_SetPriority. Assign TIM2 interrupt priority = 0 in NVIC.
    NVIC_EnableIRQ(TIM14_IRQn); 					// Enable TIM2 interrupts in NVIC. Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ.

    TIM14->DIER |= TIM_DIER_UIE; 				// Enable update interrupt generation  Relevant register: TIM14->DIER.

    //They added a line to activate timer, not sure if necessary
}


void Delay(uint32_t time) { // delay the system. .... Time is in milliseconds

  uint16_t clock = time // calculate number of clock cycles that correspond to ms time.

	TIM14->CNT |= 0x00;     //Clear timer
	TIM14->ARR = clock;      //Autoload time into timer
  TIM14->EGR = ((uint16_t)0x0001); // Update timer registers
  TIM14->CR1 |= TIM_CR1_CEN;      //Enabled counter

  while ((TIM14->SR & TIM_SR_UIF) != 0); //Loop until interrupt flag gets set

  TIM14->SR &= ~(TIM_SR_UIF);				// Clear update interrupt flag.
  TIM14->CR1 |= TIM_CR1_CEN;		 		// Restart stopped timer.

}

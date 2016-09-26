/*
 * serial.c
 *
 *  Created on: 26 sep. 2016
 *      Author: Niklas
 */

#include "stm32l0xx.h"
#include "swarm.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Error codes used to make the orange led blinking */
#define ERROR_USART 0x07

#define USART_DATA (0xCA)
#define SPI_DATA (0xDE)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void Configure_Serial(void);
void Configure_GPIO_USART2(void);
void Configure_USART2(void);
void USART_String(const char *s);


void Configure_Serial(void)
{

	Configure_GPIO_USART2();
	Configure_USART2();
}


/***************************************************************************/

void USART_String(const char *s)
{
	USART2->CR1 |= USART_CR1_TE; // Set transmit enable bit

	while(*s)
	{                // Print char's as long as not a null char
		USART2->TDR = *s;      // Put char into the data register

		while(!(USART2->ISR & USART_ISR_TXE));   // xfer to shift register
		s++;                              // Increment ptr in memory
	}
}

/***************************************************************************/

/**
  * Brief   This function :
             - Enables GPIO clock
             - Configures the USART2 pins on GPIO PA2 PA3
  * Param   None
  * Retval  None
  */
void Configure_GPIO_USART2(void)
{
	/* Enable the peripheral clock of GPIOA */
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

	/* GPIO configuration for USART2 signals */
	/* (1) Select AF mode (10) on PA2 and PA3 */
	/* (2) AF4 for PA2 USART2 signal */
	/* (3) AF4 for PA15 USART2 signal */

	GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE2|GPIO_MODER_MODE15))\
				 | (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE15_1); /* (1) */
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0x00000F00))\
				  | (4 << (2 * 4)); /* (2) */
	GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xF0000000))\
		  | (4 << ((15-8) * 4)); /* (3) */

	GPIOA->OTYPER  &= ~GPIO_OTYPER_OT_2;       //Output Push/Pull
	GPIOA->OTYPER  &= ~GPIO_OTYPER_OT_15;       //Output Push/Pull
	GPIOA->OSPEEDR |=  GPIO_OSPEEDER_OSPEED2;  //PA2 high speed
	GPIOA->OSPEEDR |=  GPIO_OSPEEDER_OSPEED15;  //PA15 high speed
	GPIOA->PUPDR   |=  GPIO_PUPDR_PUPD2_0;         //PA2 Pull up
	GPIOA->PUPDR   |=  GPIO_PUPDR_PUPD15_0;         //PA15 Pull up
}

/**
  * Brief   This function configures USART2.
  * Param   None
  * Retval  None
  */
void Configure_USART2(void)
{
	/* Enable the peripheral clock USART2 */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	/* Configure USART2 */
	/* (1) oversampling by 16, 9600 baud */
	/* (2) 8 data bit, 1 start bit, 1 stop bit, no parity */
	USART2->BRR = 160000 / 96; /* (1) */
	USART2->CR1 = USART_CR1_TE | USART_CR1_UE; /* (2) */

	/* polling idle frame Transmission w/o clock */
	while((USART2->ISR & USART_ISR_TC) != USART_ISR_TC)
	{
	/* add time out here for a robust application */
	}
	USART2->ICR = USART_ICR_TCCF;/* clear TC flag */
	USART2->CR1 |= USART_CR1_TCIE;/* enable TC interrupt */

	/* Configure IT */
	/* (4) Set priority for USART1_IRQn */
	/* (5) Enable USART1_IRQn */
	//NVIC_SetPriority(USART2_IRQn, 0); /* (4) */
	//NVIC_EnableIRQ(USART2_IRQn); /* (5) */
}

/******************************************************************************/
/*            Cortex-M0 Plus Processor Exceptions Handlers                    */
/******************************************************************************/

/**
  * Brief   This function handles USART1 interrupt request.
  * Param   None
  * Retval  None
  */
void USART1_IRQHandler(void)
{
  uint8_t USART_Data = 0;

  if((USART2->ISR & USART_ISR_TC) == USART_ISR_TC)
  {
    USART2->ICR = USART_ICR_TCCF;/* clear transfer complete flag */
  }
  else if((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
  {
    USART_Data = USART2->RDR;/* receive data, clear flag */
  }
  else
  {
    //error = ERROR_USART; /* Report an error */
    NVIC_DisableIRQ(USART2_IRQn);/* Disable USART1_IRQn */
  }
}


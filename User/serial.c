/*
 * serial.c
 *
 *  Created on: 26 sep. 2016
 *      Author: Niklas
 */

#include "stm32l0xx.h"
#include "stdio.h"
#include "swarm.h"

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
  while(*s)
  {
    while(USART2->ISR & USART_ISR_TXE); // Wait for TXE to assert
    USART2->TDR = *s++;
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
}

/******************************************************************************/
/*            Cortex-M0 Plus Processor Exceptions Handlers                    */
/******************************************************************************/

/*
 * serial.c
 *
 *  Created on: 26 sep. 2016
 *      Author: Niklas
 */

#include "stm32l0xx.h"
#include "swarm.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Error codes */
#define ERROR_USART 0x07

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char stringtosend[] = "DMA\n";
char stringtoreceive[] = "  ";


/* Private function prototypes -----------------------------------------------*/
void Configure_Serial(void);
void Configure_GPIO_USART2(void);
void Configure_DMA1(void);
void Configure_USART2(void);
void USART_Send(const char *s);


void Configure_Serial(void)
{

	Configure_GPIO_USART2();
	Configure_DMA1();
	Configure_USART2();
}


/***************************************************************************/

void USART_Send(const char *s)
{

	/* start 8-bit transmission with DMA */
	DMA1_Channel7->CCR &=~ DMA_CCR_EN;
	DMA1_Channel7->CNDTR = sizeof(stringtosend);/* Data size */
	DMA1_Channel7->CCR |= DMA_CCR_EN;
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
	/*
	GPIOA->OTYPER  &= ~GPIO_OTYPER_OT_2;       //Output Push/Pull
	GPIOA->OTYPER  &= ~GPIO_OTYPER_OT_15;       //Output Push/Pull
	GPIOA->OSPEEDR |=  GPIO_OSPEEDER_OSPEED2;  //PA2 high speed
	GPIOA->OSPEEDR |=  GPIO_OSPEEDER_OSPEED15;  //PA15 high speed
	GPIOA->PUPDR   |=  GPIO_PUPDR_PUPD2_0;         //PA2 Pull up
	GPIOA->PUPDR   |=  GPIO_PUPDR_PUPD15_0;         //PA15 Pull up
	*/
}

/**
  * Brief   This function configures DMA.
  * Param   None
  * Retval  None
  */
__INLINE void Configure_DMA1(void)
{
  /* Enable the peripheral clock DMA1 */
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  /* DMA1 Channel7 USART2_TX config */
  /* (1)  Map USART2_TX DMA channel */
  /* (2)  Peripheral address */
  /* (3)  Memory address */
  /* (4)  Memory increment */
  /*      Memory to peripheral */
  /*      8-bit transfer */
  /*      Transfer complete IT */
  DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & ~DMA_CSELR_C7S) | (3 << (6 * 4)); /* (1) */
  DMA1_Channel7->CPAR = (uint32_t)&(USART2->TDR); /* (2) */
  DMA1_Channel7->CMAR = (uint32_t)stringtosend; /* (3) */
  DMA1_Channel7->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE; /* (4) */

  /* DMA1 Channel7 USART_RX config */
  /* (5)  Map USART2_RX DMA channel */
  /* (6)  Peripheral address */
  /* (7)  Memory address */
  /* (8)  Data size */
  /* (9)  Memory increment */
  /*      Peripheral to memory*/
  /*      8-bit transfer */
  /*      Transfer complete IT */
  DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & ~DMA_CSELR_C6S) | (3 << (5 * 4)); /* (5) */
  DMA1_Channel6->CPAR = (uint32_t)&(USART2->RDR); /* (6) */
  DMA1_Channel6->CMAR = (uint32_t)stringtoreceive; /* (7) */
  DMA1_Channel6->CNDTR = sizeof(stringtoreceive); /* (8) */
  DMA1_Channel6->CCR = DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_EN; /* (9) */

  /* Configure IT */
  /* (10) Set priority for DMA1_Channel4_5_6_7_IRQn */
  /* (11) Enable DMA1_Channel4_5_6_7_IRQn */
  NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0); /* (10) */
  NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn); /* (11) */
}

/**
  * Brief   This function configures USART2.
  * Param   None
  * Retval  None
  */
__INLINE void Configure_USART2(void)
{
  /* Enable the peripheral clock USART2 */
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

  /* Configure USART2 */
  /* (1) oversampling by 16, 9600 baud */
  /* (2) Enable DMA in reception and transmission */
  /* (3) 8 data bit, 1 start bit, 1 stop bit, no parity, reception and transmission enabled */
  USART2->BRR = 160000 / 96; /* (1) */
  USART2->CR3 = USART_CR3_DMAT | USART_CR3_DMAR; /* (2) */
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; /* (3) */

  while((USART2->ISR & USART_ISR_TC) != USART_ISR_TC)/* polling idle frame Transmission */
  {
    /* add time out here for a robust application */
  }
  USART2->ICR = USART_ICR_TCCF;/* Clear TC flag */
}

/******************************************************************************/
/*            Cortex-M0 Plus Processor Exceptions Handlers                    */
/******************************************************************************/

/**
  * Brief   This function handles DMA1 channel 2 and 3 interrupt request.
  * Param   None
  * Retval  None
  */
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
  if((DMA1->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2)
  {
    DMA1->IFCR = DMA_IFCR_CTCIF2;/* Clear TC flag */
  }
  else if((DMA1->ISR & DMA_ISR_TCIF3) == DMA_ISR_TCIF3)
  {
    DMA1->IFCR = DMA_IFCR_CTCIF3;/* Clear TC flag */

    if(strcmp("OK",(const char *)stringtoreceive) == 0)
    {
      //GPIOB->ODR ^= (1 << 3);/* Toggle Green LED */
    	USART_Send("ah");
    }

    DMA1_Channel6->CCR &=~ DMA_CCR_EN;
    DMA1_Channel6->CNDTR = sizeof(stringtoreceive);/* Data size */
    DMA1_Channel6->CCR |= DMA_CCR_EN;
  }
  else
  {
    //error = ERROR_USART; /* Report an error */
    NVIC_DisableIRQ(DMA1_Channel4_5_6_7_IRQn);/* Disable DMA1_Channel4_5_6_7_IRQn */
  }
}


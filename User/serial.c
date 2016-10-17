/**
 ******************************************************************************
 * File     serial.c
 * Author   Niklas Lidström, Jakob Max
 * Brief	 This code controls serial communication between the MCU and PC

 ===============================================================================
 ##### How to test this example #####
 ===============================================================================
 - Solder SB31 and SB32 on the board
 - Unplug the electronic paper display
 - Connect PA4/PB12, PB3/PB13, PA6/PB14, PA7/PB15.
 - Launch the program
 - Press the button
 - The green LED toggles (message transmit and receive)
 - Else the red LED blink

 *
 ******************************************************************************
 * Attention
 *
 * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/

#include "stm32l0xx.h"
#include "swarm.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

extern volatile uint16_t error;

/* Private typedef -----------------------------------------------------------*/

typedef struct node
{
	struct node *next;
	uint16_t len;
	char cmd[];
} node_t;

/* Private define ------------------------------------------------------------*/

#define RX_BUFFER_LEN 256
#define TX_BUFFER_LEN 256

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

char rx_buffer[RX_BUFFER_LEN];
char tx_buffer[TX_BUFFER_LEN];
uint16_t rx_idx = 0;

node_t *cmd_head = NULL;
node_t *cmd_tail = NULL;

/* Private function prototypes -----------------------------------------------*/

void Configure_GPIO_USART2(void);
void Configure_DMA1(void);
void Configure_USART2(void);

/* Private functions ---------------------------------------------------------*/

/**
 * Brief   This function configures the serial communication
 * Param   None
 * Retval  None
 */
void Configure_Serial(void)
{
	Configure_GPIO_USART2();
	Configure_DMA1();
	Configure_USART2();
}

/**
 * Brief   This function tests the serial communication
 * Param   None
 * Retval  None
 */
void Test_Serial(void)
{
	USART_sendln("\r\nAdding to queue!");
	cmd_queue_add("first");
	cmd_queue_add("second");
	cmd_queue_add("third");
	cmd_queue_add("fourth");

	USART_sendln("\r\nDequeuing:");

	USART_sendln(cmd_queue_process());

	USART_sendln("\r\nPrint first:");
	cmd_queue_print_head();

	USART_sendln("\r\nList queue:");
	cmd_queue_print_all();

	USART_sendln("\r\nAdding to queue!");
	cmd_queue_add("new");

	USART_sendln("\r\nList queue again:");
	cmd_queue_print_all();

	USART_sendln("\r\nQueue finished!");
}

/**
 * Brief   This function sends a string to a serial port
 * Param   char* The string to send
 * Retval  None
 */
void USART_send(const char *msg)
{
	while (DMA1_Channel4->CNDTR)
		; /* Wait until DMA data register is empty */

	size_t len = strlen(msg) + 1;
	memcpy(tx_buffer, msg, len);

	/* start 8-bit transmission with DMA */
	DMA1_Channel4->CCR &= ~ DMA_CCR_EN;
	DMA1_Channel4->CNDTR = len - 1;/* Data size */
	DMA1_Channel4->CCR |= DMA_CCR_EN;
}

/**
 * Brief   This function sends a string to a serial port. The message is terminated
 * 		with a newline character.
 * Param   char* The string to send
 * Retval  None
 */
void USART_sendln(const char *msg)
{
	while (DMA1_Channel4->CNDTR)
		; /* Wait until DMA data register is empty */

	size_t len = strlen(msg);
	memcpy(tx_buffer, msg, len);
	tx_buffer[len] = '\n'; // append newline
	tx_buffer[len + 1] = '\0'; // null terminate

	/* start 8-bit transmission with DMA */
	DMA1_Channel4->CCR &= ~ DMA_CCR_EN;
	DMA1_Channel4->CNDTR = len + 1;/* Data size */
	DMA1_Channel4->CCR |= DMA_CCR_EN;
}

/**
 * Brief   This function adds a command to the command queue
 * Param   char* The command string to add
 * Retval  None
 */
void cmd_queue_add(const char *cmd)
{
	size_t len = strlen(cmd) + 1;
	struct node *link = (struct node*) malloc(
			sizeof(*link) + len * sizeof(char));

	link->len = len;
	link->next = NULL;

	memcpy(link->cmd, cmd, len);

	cmd_tail->next = link;
	cmd_tail = link;

	if (!cmd_head)
	{
		cmd_head = link;
	}
}

/**
 * Brief   This function removes the top command in the command queue
 * Param   None
 * Retval  char* The top command string
 */
char* cmd_queue_process(void)
{
	char *retval;

	node_t *next_node = NULL;

	if (cmd_head == NULL)
	{
		return NULL;
	}

	next_node = cmd_head->next;
	retval = cmd_head->cmd;
	free(cmd_head);
	cmd_head = next_node;

	return retval;
}

/**
 * Brief   This function prints the first command in the command queue to serial
 * Param   None
 * Retval  None
 */
void cmd_queue_print_head(void)
{
	USART_sendln(cmd_head->cmd);
}

/**
 * Brief   This function prints all commands in the command queue to serial
 * Param   None
 * Retval  None
 */
void cmd_queue_print_all(void)
{
	node_t *current = cmd_head;

	char list[255] = "";

	while (current)
	{
		strcat(list, current->cmd);
		strcat(list, "\n");
		current = current->next;
	}

	USART_send(list);
}

/**
 * Brief   This function :
 - Enables GPIO clock
 - Configures the USART2 pins on GPIO PA2 PA15
 * Param   None
 * Retval  None
 */
void Configure_GPIO_USART2(void)
{
	/* Enable the peripheral clock of GPIOA */
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

	/* GPIO configuration for USART2 signals */
	/* (1) Select AF mode (10) on PA2 and PA15 */
	/* (2) AF4 for PA2 USART2 signal */
	/* (3) AF4 for PA15 USART2 signal */

	GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE15))\
			| (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE15_1); /* (1) */

	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0x00000F00))\
			| (4 << (2 * 4)); /* (2) */

	GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xF0000000))\
			| (4 << ((15 - 8) * 4)); /* (3) */

	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_2;       	//Output Push/Pull
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_15;       //Output Push/Pull
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED2;  	//PA2 high speed
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED15;  //PA15 high speed
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD2_0;      //PA2 Pull up
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD15_0;     //PA15 Pull up
}

/**
 * Brief   This function configures DMA.
 * Param   None
 * Retval  None
 */
void Configure_DMA1(void)
{
	/* Enable the peripheral clock DMA1 */
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	/* DMA1 Channel4 USART2_TX config */
	/* (1)  Map USART2_TX DMA channel */
	/* (2)  Peripheral address */
	/* (3)  Memory address */
	/* (4)  Memory increment */
	/*      Memory to peripheral */
	/*      8-bit transfer */
	/*      Transfer complete IT */
	/* (5)  Set DMA interrupt priority */
	/* (6)  Enable DMA interrupt handler */

	DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & ~DMA_CSELR_C4S) | (4 << (3 * 4)); /* (1) */
	DMA1_Channel4->CPAR = (uint32_t) (&(USART2->TDR)); /* (2) */
	DMA1_Channel4->CMAR = (uint32_t) tx_buffer; /* (3) */
	DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE; /* (4) */

	NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0); /* (5) */
	NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn); /* (6) */
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
	/* (2) Enable DMA in reception and transmission */
	/* (3) 8 data bit, 1 start bit, 1 stop bit, no parity, reception and transmission enabled */
	/* (4)  Set DMA interrupt priority */
	/* (5)  Enable DMA interrupt handler */

	USART2->BRR = 160000 / 96; /* (1) */
	USART2->CR3 = USART_CR3_DMAT; /* (2) */
	USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE; /* (3) */

	while ((USART2->ISR & USART_ISR_TC) != USART_ISR_TC)/* polling idle frame Transmission */
	{
		/* add time out here for a robust application */
	}
	USART2->ICR = USART_ICR_TCCF;/* Clear TC flag */

	NVIC_SetPriority(USART2_IRQn, 0); /* (4) */
	NVIC_EnableIRQ(USART2_IRQn); /* (5) */
}

/******************************************************************************/
/*            Cortex-M0 Plus Peripheral Exceptions Handlers                   */
/******************************************************************************/

/**
 * Brief   This function handles USART2 RX interrupt request.
 * Param   None
 * Retval  None
 */
void USART2_IRQHandler(void)
{
	if ((USART2->ISR & USART_ISR_RXNE))
	{
		char data = (char) USART2->RDR;

		if (data == '\n') // Command was registered
		{
			rx_buffer[rx_idx] = '\0';
			cmd_queue_add(rx_buffer);
			rx_idx = 0;
		}
		else
		{
			rx_buffer[rx_idx++] = data;
		}
	}
}

/**
 * Brief   This function handles DMA1 channel 4, 5, 6 and 7 interrupt request.
 * Param   None
 * Retval  None
 */
void DMA1_Channel4_5_6_7_IRQHandler(void)
{

	if ((DMA1->ISR & DMA_ISR_TCIF4) == DMA_ISR_TCIF4)
	{
		DMA1->IFCR = DMA_IFCR_CTCIF4;/* Clear TC flag */
	}
	else
	{
		error = ERROR_DMA; /* Report an error */
		NVIC_DisableIRQ(DMA1_Channel4_5_6_7_IRQn);/* Disable DMA1_Channel4_5_6_7_IRQn */
	}
}


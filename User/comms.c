/**
 ******************************************************************************
 * File     comms.c
 * Author   Niklas Lidström, Jakob Max
 * Brief	This code controls the IR communication between robots and
 * 			global controller

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
/* Private define ------------------------------------------------------------*/

#define ADC_CHANNEL_COUNT 3

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * Brief   This function configures the robot-to-robot communication
 * Param   None
 * Retval  None
 */
void Configure_Comms(void)
{

}


/******************************************************************************/
/*            Cortex-M0 Plus Peripheral Exceptions Handlers                   */
/******************************************************************************/

/**
 * Brief   This function handles DMA1 channel 1 interrupt request.
 * Param   None
 * Retval  None
 */

void DMA1_Channel1_IRQHandler(void)
{
  if ((DMA1->ISR & DMA_ISR_TCIF1) != 0) /* Test if transfer completed on DMA channel 1 */
  {
    DMA1_Channel1->CCR &= (uint32_t)(~DMA_CCR_EN); /* Disable DMA Channel 1 to write in CNDTR*/
    DMA1_Channel1->CNDTR = ADC_CHANNEL_COUNT; /* Reload the number of DMA tranfer to be performs on DMA channel 1 */
    DMA1_Channel1->CCR |= DMA_CCR_EN; /* Enable again DMA Channel 1 */
    DMA1->IFCR |= DMA_IFCR_CTCIF1; /* Clear the flag */
    GPIOB->ODR ^= (1<<4);//toggle green led on PC9
    ADC1->CR |= ADC_CR_ADSTART; /* Restart the sequence conversion */
  }
  else if ((DMA1->ISR & DMA_ISR_TEIF1) != 0) /* Test if transfer error on DMA channel 1 */
  {
    error |= ERROR_DMA_ADC; /* Report an error */
    DMA1->IFCR |= DMA_IFCR_CTEIF1; /* Clear the flag */
  }
  else
  {
    error |= ERROR_DMA_ADC_UNEXPECTED_IRQ; /* report unexpected DMA interrupt occurrence */
  }
}


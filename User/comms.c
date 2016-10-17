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
#define EXTI_BUFFER_LENGTH 64

#define MAIN_TIMER_IR_BASE TIM2_BASE
#define MAIN_TIMER_IR ((TIM_TypeDef *) MAIN_TIMER_IR_BASE)

#define SLAVE_TIMER_IR_BASE TIM21_BASE
#define SLAVE_TIMER_IR ((TIM_TypeDef *) SLAVE_TIMER_IR_BASE)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t adc_buffer[ADC_CHANNEL_COUNT] = {0};


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * Brief   This function configures the robot-to-robot communication
 * Param   None
 * Retval  None
 */
void Configure_Comms(void)
{
	ConfigureTIM_PWM();
}

/**
  * Brief   This function configures the TIMx as PWM mode 1
  *         and enables the peripheral clock on TIMx and on GPIOB.
  *         It configures GPIO PB13 as Alternate function for TIM21_CH1
  *         To use another timer, channel or GPIO, the RCC and GPIO configuration
  *         must be adapted according to the datasheet.
  *         In case of other timer, the interrupt sub-routine must also be renamed
  *         with the right handler and the NVIC configured correctly.
  * Param   None
  * Retval  None
  */
void ConfigureTIM_PWM(void)
{
  /* (1) Enable the peripheral clock of Timer x */
  /* (2) Enable the peripheral clock of GPIOA */
  /* (3) Select alternate function mode on GPIOA pin 0,1,2,3 */
  /* (4) Select AF2 on PA0,1,2,3 in AFRL for TIM2 */
  /* (5) Select AF5 on PA9,10 in AFRH for TIM21 */

  RCC->APB2ENR |= RCC_APB2ENR_TIM21EN; /* (1) */
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN; /* (2) */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3)) \
               | (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1 | GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1); /* (3) */

  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7)) \
                 | (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1 ); /* (3) */

  GPIOA->AFR[0] |= 0x2; /* (4) */
  GPIOA->AFR[0] |= 0x2 << (1 * 4);
  GPIOA->AFR[0] |= 0x2 << (2 * 4);
  GPIOA->AFR[0] |= 0x2 << (3 * 4);

  GPIOA->AFR[1] |= 0x5 << ((9 - 8) * 4); /* (4) */
  GPIOA->AFR[1] |= 0x5 << ((10 - 8) * 4);

  /* (1) Set prescaler to 799, so APBCLK/800 i.e 20KHz */
  /* (2) Set ARR = 1, so the timer will reload at 40KHz (25 us) */
  /* (3) Set CCRx = 1, , the signal will be high during half the timer (12.5 us)  */
  /* (4) Select PWM mode 1 on OC1  (OC1M = 110),
         enable preload register on OC1 (OC1PE = 1) */
  /* (5) Select active high polarity on OC1 (CC1P = 0, reset value),
         enable the output on OC1 (CC1E = 1)*/
  /* (6) Enable output (MOE = 1)*/
  /* (7) Enable counter (CEN = 1)
         select edge aligned mode (CMS = 00, reset value)
         select direction as upcounter (DIR = 0, reset value) */
  /* (8) Force update generation (UG = 1) */

  MAIN_TIMER_IR->PSC = 799; /* (1) */
  MAIN_TIMER_IR->ARR = 1; /* (2) */
  MAIN_TIMER_IR->CCR1 = 1; /* (3) */
  MAIN_TIMER_IR->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE; /* (4) */
  MAIN_TIMER_IR->CCER |= TIM_CCER_CC1E; /* (5) */
  MAIN_TIMER_IR->CR1 |= TIM_CR1_CEN; /* (7) */
  //MAIN_TIMER_IR->EGR |= TIM_EGR_UG; /* (8) */
}

void TIM_PWM_enable(bool enable)
{
	if(enable)
	{
		MAIN_TIMER_IR->CR1 |= TIM_CR1_CEN;
	} else {
		MAIN_TIMER_IR->CR1 &= ~TIM_CR1_CEN;
	}
}



/******************************************************************************/
/*            Cortex-M0 Plus Peripheral Exceptions Handlers                   */
/******************************************************************************/




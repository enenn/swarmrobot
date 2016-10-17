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



/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx.h"
#include "swarm.h"

/* Private define ------------------------------------------------------------*/


#define NUMBER_OF_ADC_CHANNEL 7

/* Private variables ---------------------------------------------------------*/
uint16_t ADC_array[NUMBER_OF_ADC_CHANNEL]; //Array to store the values coming from the ADC
uint32_t CurrentChannel; //index on the ADC_array
int configure_comms(void) /*is this called main?
{
	SysTick->CTRL = 0; /* Disable SysTick */
  ConfigureExternalIT();
  ConfigureGPIO();
  ConfigureGPIOforADC();
  SetClockForADC();
  ConfigureADC();
  CalibrateADC();
  EnableADC();
  CurrentChannel = 0; /* Initializes the CurrentChannel */
  ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversions */

  while (error == 0) /* loop till no unrecoverable error, should never be exited */
  {
    __WFI();
  }
  DisableADC();
  SysTick_Config(16000); /* 1ms config */
  while (1)
}
  /**
    * Brief   This function enables the peripheral clocks on GPIO port A and B,
    *         configures GPIO PB4 in output mode for the Green LED pin,
    *         configures GPIO PA5 in output mode for the Red LED pin,
    * Param   None
    * Retval  None
    */
  __INLINE void ConfigureGPIO(void)
  {
    /* (1) Enable the peripheral clock of GPIOA and GPIOB */
    /* (2) Select output mode (01) on GPIOA pin 5 */
    /* (3) Select output mode (01) on GPIOB pin 4 */
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN; /* (1) */
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE5))
                 | (GPIO_MODER_MODE5_0); /* (2) */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE4))
                 | (GPIO_MODER_MODE4_0); /* (3) */
  }


  __INLINE void ConfigureDMA(void)
  {
    /* (1) Enable the peripheral clock on DMA */
    /* (2) Enable DMA transfer on ADC - DMACFG is kept at 0 for one shot mode */
    /* (3) Configure the peripheral data register address */
    /* (4) Configure the memory address */
    /* (5) Configure the number of DMA tranfer to be performed on DMA channel 1 */
    /* (6) Configure increment, size and interrupts */
    /* (7) Enable DMA Channel 1 */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* (1) */
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN; /* (2) */
    DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR)); /* (3) */
    DMA1_Channel1->CMAR = (uint32_t)(ADC_array); /* (4) */
    DMA1_Channel1->CNDTR = NUMBER_OF_ADC_CHANNEL; /* (5) */
    DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 \
                        | DMA_CCR_TEIE | DMA_CCR_TCIE ; /* (6) */
    DMA1_Channel1->CCR |= DMA_CCR_EN; /* (7) */

    /* Configure NVIC for DMA */
    /* (1) Enable Interrupt on DMA Channel 1  */
    /* (2) Set priority for DMA Channel 1 */
    NVIC_EnableIRQ(DMA1_Channel1_IRQn); /* (1) */
    NVIC_SetPriority(DMA1_Channel1_IRQn,0); /* (2) */
  }
  /**
    * Brief   This function enables the peripheral clocks on GPIO ports A,B
    *         configures PA4 and PB1 in Analog mode.
    *         For portability, some GPIO are again enabled.
    * Param   None
    * Retval  None
    */

  __INLINE void  ConfigureGPIOforADC(void)
  {
    /* (1) Enable the peripheral clock of GPIOA and GPIOB */
    /* (2) Select analog mode for PA4 (reset state) */
    /* (3) Select analog mode for PB1 (reset state) */
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN; /* (1) */
    //GPIOA->MODER |= GPIO_MODER_MODE4; /* (2) */
    //GPIOB->MODER |= GPIO_MODER_MODE1; /* (3) */
  }
  /**
    * Brief   This function enables the clock in the RCC for the ADC
    *        and for the System configuration (mandatory to enable VREFINT)
    * Param   None
    * Retval  None
    */
  __INLINE void SetClockForADC(void)
  {
    /* (1) Enable the peripheral clock of the ADC and SYSCFG */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_SYSCFGEN; /* (1) */
  }
  __INLINE void  CalibrateADC(void)
  {
    /* (1) Ensure that ADEN = 0 */
    /* (2) Clear ADEN */
    /* (3) Set ADCAL=1 */
    /* (4) Wait until EOCAL=1 */
    /* (5) Clear EOCAL */
    if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
    {
      ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);  /* (2) */
    }
    ADC1->CR |= ADC_CR_ADCAL; /* (3) */
    while ((ADC1->ISR & ADC_ISR_EOCAL) == 0) /* (4) */
    {
      /* For robust implementation, add here time-out management */
    }
    ADC1->ISR |= ADC_ISR_EOCAL; /* (5) */
  }
  __INLINE void ConfigureADC(void)
  {
    /* (1) Select HSI16 by writing 00 in CKMODE (reset value) */
    /* (2) Select the continuous mode and scanning direction */
    /* (3) Select CHSEL4, CHSEL9 and CHSEL17 */
    /* (4) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than 5 us */
    /* (5) Enable interrupts on EOC, EOSEQ and overrrun */
    /* (6) Wake-up the VREFINT (only for VLCD, Temp sensor and VRefInt) */
    //ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* (1) */
    ADC1->CFGR1 |= ADC_CFGR1_WAIT |ADC_CFGR1_CONT | ADC_CFGR1_SCANDIR; /* (2) */
    /* TODO add more channels*/
    ADC1->CHSELR = ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL9 \
                 | ADC_CHSELR_CHSEL17; /* (3) */
    ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; /* (4) */
    ADC1->IER = ADC_IER_EOCIE | ADC_IER_EOSEQIE | ADC_IER_OVRIE; /* (5) */
    /*ADC->CCR |= ADC_CCR_VREFEN; /* (6) */

    /* Configure NVIC for ADC */
    /* (1) Enable Interrupt on ADC */
    /* (2) Set priority for ADC */
    NVIC_EnableIRQ(ADC1_COMP_IRQn); /* (1) */
    NVIC_SetPriority(ADC1_COMP_IRQn,0); /* (2) */
  }
  __INLINE void EnableADC(void)
  {
    /* (1) Enable the ADC */
    /* (2) Wait until ADC ready if AUTOFF is not set */
    ADC1->CR |= ADC_CR_ADEN; /* (1) */
    if ((ADC1->CFGR1 &  ADC_CFGR1_AUTOFF) == 0)
    {
      while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (2) */
      {
        /* For robust implementation, add here time-out management */
      }
    }
  }

  __INLINE void DisableADC(void)
  {
    /* (1) Ensure that no conversion on going */
    /* (2) Stop any ongoing conversion */
    /* (3) Wait until ADSTP is reset by hardware i.e. conversion is stopped */
    /* (4) Disable the ADC */
    /* (5) Wait until the ADC is fully disabled */
    if ((ADC1->CR & ADC_CR_ADSTART) != 0) /* (1) */
    {
      ADC1->CR |= ADC_CR_ADSTP; /* (2) */
    }
    while ((ADC1->CR & ADC_CR_ADSTP) != 0) /* (3) */
    {
       /* For robust implementation, add here time-out management */
    }
    ADC1->CR |= ADC_CR_ADDIS; /* (4) */
    while ((ADC1->CR & ADC_CR_ADEN) != 0) /* (5) */
    {
      /* For robust implementation, add here time-out management */
    }
  }
  /* TODO interrupt handlers

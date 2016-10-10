/*
 * comms.c
 *
 *  Created on: 26 sep. 2016
 *      Author: Niklas
 */


/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx.h"
#include "swarm.h"

/* Private define ------------------------------------------------------------*/


#define NUMBER_OF_ADC_CHANNEL 3

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

/*
 * serial.c
 *
 *  Created on: 26 sep. 2016
 *      Author: Niklas
 */

#include "stm32l0xx.h"
#include "swarm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Time-out values */
#define HSI_TIMEOUT_VALUE          ((uint32_t)100)  /* 100 ms */
#define PLL_TIMEOUT_VALUE          ((uint32_t)100)  /* 100 ms */
#define CLOCKSWITCH_TIMEOUT_VALUE  ((uint32_t)5000) /* 5 s    */

/* Delay value : short one is used for the error coding, long one (~1s) in case
   of no error or between two bursts */
#define SHORT_DELAY 200
#define LONG_DELAY 1000

/* Error codes used to make the orange led blinking */
#define ERROR_USART 0x01
#define ERROR_HSI_TIMEOUT 0x02
#define ERROR_PLL_TIMEOUT 0x03
#define ERROR_CLKSWITCH_TIMEOUT 0x04

#define USART_DATA (0xCA)
#define SPI_DATA (0xDE)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t Tick;
volatile uint16_t error = 0;  //initialized at 0 and modified by the functions

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Configure_GPIO_LED(void);
void Configure_GPIO_SPI1(void);
void Configure_SPI1(void);
void Configure_GPIO_USART1(void);
void Configure_USART1(void);
void Configure_GPIO_Button(void);
void Configure_EXTI(void);

/* Private functions ---------------------------------------------------------*/

/**
  * Brief   Main program.
  * Param   None
  * Retval  None
  */

//TODO:remove this and move to main
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32l0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32l0xx.c file
     */
  SysTick_Config(2000); /* 1ms config */
  SystemClock_Config();
  Configure_GPIO_LED();
  if (error != 0)
  {
    while(1) /* endless loop */
    {
    }
  }

  SysTick_Config(16000); /* 1ms config */

  Configure_GPIO_SPI1();
  Configure_SPI1();
  Configure_GPIO_USART1();
  Configure_USART1();
  Configure_GPIO_Button();
  Configure_EXTI();

  /* Start transmission in button IRQ handler */
  while (1) /* Infinite loop */
  {
  }
}


/**
  * Brief   This function configures the system clock @16MHz and voltage scale 1
  *         assuming the registers have their reset value before the call.
  *         POWER SCALE   = RANGE 1
  *         SYSTEM CLOCK  = PLL MUL8 DIV2
  *         PLL SOURCE    = HSI/4
  *         FLASH LATENCY = 0
  * Param   None
  * Retval  None
  */

/**
  * Brief   This function enables the peripheral clocks on GPIO port A and B,
  *         configures GPIO PB4 in output mode for the Green LED pin,
  *         configures GPIO PA5 in output mode for the Red LED pin,
  * Param   None
  * Retval  None
  */
__INLINE void Configure_GPIO_LED(void)
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
  * Brief   This function :
             - Enables GPIO clock
             - Configures the SPI1 pins on GPIO PA6 PA7 PB3
  * Param   None
  * Retval  None
  */
__INLINE void Configure_GPIO_SPI1(void)
{
  /* Enable the peripheral clock of GPIOA and GPIOB */
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

  /* GPIO configuration for SPI1 signals */
  /* (1) Select AF mode (10) on PA6 and PA7 */
  /* (2) AF0 for SPI1 signals */
  /* (3) Select AF mode (10) on PB3 */
  /* (4) AF0 for SPI1 signals */
  GPIOA->MODER = (GPIOA->MODER
                  & ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7))\
                | (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1); /* (1) */
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(( 15 << 6 ) | ( 15 << 7 ))); /* (2) */
  GPIOB->MODER = (GPIOB->MODER
                  & ~(GPIO_MODER_MODE3)) | (GPIO_MODER_MODE3_1); /* (3) */
  GPIOB->AFR[0] = (GPIOA->AFR[0] & ~( 15 << 3 )); /* (4) */
}

/**
  * Brief   This function configures SPI1.
  * Param   None
  * Retval  None
  */
__INLINE void Configure_SPI1(void)
{
  /* Enable the peripheral clock SPI1 */
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  /* (1) Configure SPI1 in slave NSS soft */
  /*     SS at low, slave, CPOL and CPHA at zero (rising first edge) */
  SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SPE; /* (1) */
}

/**
  * Brief   This function :
             - Enables GPIO clock
             - Configures the USART1 pins on GPIO PA8, PA9 and PA10
  * Param   None
  * Retval  None
  */
__INLINE void Configure_GPIO_USART1(void)
{
  /* Enable the peripheral clock of GPIOA */
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  /* GPIO configuration for USART1 signals */
  /* (1) Select AF mode (10) on PA8, PA9 and PA10 */
  /* (2) AF4 for USART1 signals */
  GPIOA->MODER = (GPIOA->MODER \
               & ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10))\
           | (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1)\
                 ; /* (1) */
  GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (0x00000FFF))\
                  | (4 << (0 * 4)) | (4 << (1 * 4)) | (4 << (2 * 4)); /* (2) */
}

/**
  * Brief   This function configures USART1.
  * Param   None
  * Retval  None
  */
__INLINE void Configure_USART1(void)
{
  /* Enable the peripheral clock USART1 */
  RCC->APB2ENR |= RCC_APB1ENR_USART2EN;

  /* Configure USART1 */
  /* (1) oversampling by 16, 9600 baud */
  /* (2) Synchronous mode */
  /*     CPOL and CPHA = 0 => rising first edge */
  /*     Last bit clock pulse */
  /*     Most significant bit first in transmit/receive */
  /* (3) 8 data bit, 1 start bit, 1 stop bit, no parity */
  /*     Transmission enabled, reception enabled */
  USART2->BRR = 160000 / 96; /* (1) */
  USART2->CR2 = USART_CR2_MSBFIRST | USART_CR2_CLKEN | USART_CR2_LBCL; /* (2) */
  USART2->CR1 = USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE; /* (3) */

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
  NVIC_SetPriority(USART2_IRQn, 0); /* (4) */
  NVIC_EnableIRQ(USART2_IRQn); /* (5) */
}

/**
  * Brief   This function :
             - Enables GPIO clock
             - Configures the Push Button GPIO PA0
  * Param   None
  * Retval  None
  */
__INLINE void Configure_GPIO_Button(void)
{
  /* Enable the peripheral clock of GPIOA */
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  /* Select mode */
  /* Select input mode (00) on PA0 */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE0));
}

/**
  * Brief   This function configures EXTI.
  * Param   None
  * Retval  None
  */
__INLINE void Configure_EXTI(void)
{
  /* Configure Syscfg, exti and nvic for pushbutton PA0 */
  /* (1) PA0 as source input */
  /* (2) Unmask port 0 */
  /* (3) Rising edge */
  /* (4) Set priority */
  /* (5) Enable EXTI0_1_IRQn */
  SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0) | SYSCFG_EXTICR1_EXTI0_PA; /* (1) */
  EXTI->IMR |= EXTI_IMR_IM0; /* (2) */
  EXTI->RTSR |= EXTI_RTSR_TR0; /* (3) */
  NVIC_SetPriority(EXTI0_1_IRQn, 0); /* (4) */
  NVIC_EnableIRQ(EXTI0_1_IRQn); /* (5) */
}

/******************************************************************************/
/*            Cortex-M0 Plus Processor Exceptions Handlers                         */
/******************************************************************************/



/******************************************************************************/
/*                 STM32L0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l0xx.s).                                               */
/******************************************************************************/


/**
  * Brief   This function handles EXTI 0 1 interrupt request.
  * Param   None
  * Retval  None
  */
void EXTI0_1_IRQHandler(void)
{
  if((EXTI->PR & EXTI_PR_PR0) == EXTI_PR_PR0)
  {
    /* Clear EXTI 0 flag */
    EXTI->PR = EXTI_PR_PR0;

    /* start 8-bit synchronous transmission */
    //*(uint8_t *)&(SPI1->DR) = SPI_DATA;/* to test USART reception */
    SPI1->DR = SPI_DATA;/* to test USART reception */
    USART2->TDR = USART_DATA;/* will inititiate 8-bit transmission if TXE */
  }
}

/**
  * Brief   This function handles USART1 interrupt request.
  * Param   None
  * Retval  None
  */
void USART1_IRQHandler(void)
{
  uint8_t SPI_Data = 0;
  uint8_t USART_Data = 0;

  if((USART2->ISR & USART_ISR_TC) == USART_ISR_TC)
  {
    USART2->ICR = USART_ICR_TCCF;/* clear transfer complete flag */
  }
  else if((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
  {
    USART_Data = USART2->RDR;/* receive data, clear flag */
    SPI_Data = SPI1->DR;/* Get transmit data to SPI */

    if((USART_Data == SPI_DATA)&&(SPI_Data == USART_DATA))
    {
      GPIOB->ODR ^= (1 << 4);/* Toggle Green LED */
    }
    else
    {
      error = ERROR_USART; /* Report an error */
    }
  }
  else
  {
    error = ERROR_USART; /* Report an error */
    NVIC_DisableIRQ(USART2_IRQn);/* Disable USART1_IRQn */
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


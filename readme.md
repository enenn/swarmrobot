##Swarm robot

###Required software

* [ST-Link V2-1 driver](http://www.st.com/content/st_com/en/products/embedded-software/development-tool-software/stsw-link009.html)

* [Virtual COM port driver](http://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-stm32102.html)

* Some serial terminal viewer. [Termite](http://www.compuphase.com/software_termite.htm), for example.


###Serial communication

Baud rate: 9600

###IR comms

Carrier frequency: 40 KHz = 16 MHz / 400


###Pin list

| Reg # | Pin mode              | Description                      |
| ----- | --------------------- | -------------------------------- |
| PA0   | AF2 (TIM2_CH1)        | PWM for IR out                   |
| PA1   | AF2 (TIM2_CH2)        | PWM for IR out                   |
| PA2   | AF2 (TIM2_CH3)        | PWM for IR out                   |
| PA3   | AF2 (TIM2_CH4)        | PWM for IR out                   |
| PA4   | ADC                   | ADC_IN4                          |
| PA5   | ADC                   | ADC_IN5                          |
| PA6   | ADC                   | ADC_IN6                          |
| PA7   | ADC                   | ADC_IN7                          |
| PA8   |                       |                                  |
| PA9   | AF5 (TIM22_CH1)       | PWM for IR out (synced to TIM2)  |
| PA10  | AF5 (TIM22_CH2)       | PWM for IR out (synced to TIM2)  |
| PA11  |                       |                                  |
| PA12  |                       |                                  |
| PA13  |                       |                                  |
| PA14  |                       |                                  |
| PA15  |                       |                                  |
| PB0   | ADC                   | ADC_IN8                          |
| PB1   | ADC                   | ADC_IN9                          |
| PB2   |                       |                                  |
| PB3   |                       |                                  |
| PB4   |                       |                                  |
| PB5   |                       |                                  |
| PB6   |                       |                                  |
| PB7   |                       |                                  |
| PB8   |                       |                                  |
| PB9   |                       |                                  |
| PB10  |                       |                                  |
| PB11  |                       |                                  |
| PB12  |                       |                                  |
| PB13  |                       |                                  |
| PB14  |                       |                                  |
| PB15  |                       |                                  |
| PB0   |                       |                                  |
| PC0   |                       |                                  |
| PH0   |                       |                                  |
| PH1   |                       |                                  |











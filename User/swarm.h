/*
 * Error codes
 */

#define ERROR_HSI_TIMEOUT 0x01
#define ERROR_PLL_TIMEOUT 0x02
#define ERROR_CLKSWITCH_TIMEOUT 0x03

#define ERROR_USART 0x04
#define ERROR_DMA 0x05

/*
 * Serial communication
 * Logging
 * Debugging
 */

void Configure_Serial(void);
void USART_send(const char *s);
void USART_sendln(const char *s);

void cmd_queue_add(const char *cmd);
char* cmd_queue_process(void);
void cmd_queue_list(void);
void cmd_queue_print_head(void);


/*
 * Behaviour
 */



/*
 * Robot to robot communication
 * Sensors
 */


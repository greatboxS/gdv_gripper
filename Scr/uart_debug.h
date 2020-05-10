#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include <stdint.h>

//void MX_USART3_UART_Init(void);

void uart_pc_init(void);
void send_response_data(uint8_t *str_, uint16_t len);
void Printf_Debug(unsigned char *data,int size);
void uart_debug_init(void);
void send_debugging_data(char *str_);

#endif

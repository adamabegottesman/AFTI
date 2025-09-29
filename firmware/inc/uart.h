
#ifndef INC_UART_H_
#define INC_UART_H_

#include <stdint.h>

#include "stm32f4xx.h"

void uart2_tx_init(void);
char uart2_read(void);

void uart2_rxtx_init(void);
//int uart2_getchar_nonblocking(void);

#endif /* INC_UART_H_ */

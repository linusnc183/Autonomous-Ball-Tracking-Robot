#ifndef UART_CONTROL_H
#define UART_CONTROL_H

void UART_Init(void);
void UART_Update(void);   // call periodically; stops motors if link times out

#endif

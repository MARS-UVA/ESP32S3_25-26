#ifndef POWERLIB_UART
#define POWERLIB_UART

#include "driver/uart.h"
#include "utils.h"
#include "marsRTOS.h"

/* --------------------- Functions ------------------ */

void UART_setup();
void UART_read(ControlPacket *packet);
void UART_write(CurrVoltPacket *packet);

#endif
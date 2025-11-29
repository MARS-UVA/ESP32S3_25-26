#ifndef POWERLIB_CONTROL
#define POWERLIB_CONTROL

#include "can2.h"
#include "uart.h"

extern QueueHandle_t uart_queue;
void initializeTalons();

void directControl(SerialPacket pkt);
void UART_can_task();

#endif
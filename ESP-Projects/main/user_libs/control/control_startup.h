#ifndef POWERLIB_CONTROL
#define POWERLIB_CONTROL

#include "can2.h"
#include "uart.h"
#include "wifi.h"
#include "actuators.h"

extern QueueHandle_t uart_queue;
extern int leftDirection;
extern int rightDirection;

void initializeTalons();

void directControl(SerialPacket pkt);
void UART_can_task();
void moveActuators();
void evaluteActuators();

#endif

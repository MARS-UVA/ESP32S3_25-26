#ifndef MARS_RTOS_H
#define MARS_RTOS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "uart.h"
#include "wifi.h"

QueueHandle_t control_queue; // queue stores the control packet values

void UART_rx_task();
void UART_tx_task();
void UART_can_task();

#endif
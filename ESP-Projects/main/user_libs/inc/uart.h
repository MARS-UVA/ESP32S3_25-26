#pragma once

#include "driver/uart.h"
#include "utils.h"
#include "packets.h"

/* --------------------- Functions ------------------ */

void UART_setup();
void UART_read(ControlPacket_OneRobot *packet);
void UART_write(uint8_t *packet, size_t length);

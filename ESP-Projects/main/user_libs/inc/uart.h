#pragma once

#include "driver/uart.h"
#include "utils.h"
#include "packets.h"

/* --------------------- Functions ------------------ */

void UART_setup();
void UART_read(ControlPacket *packet);
void UART_write(CurrVoltPacket *packet);
void UART_write_position(PositionPacket *packet);

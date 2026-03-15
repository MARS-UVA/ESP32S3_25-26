#pragma once

#include "driver/uart.h"
#include "utils.h"
#include "packets.h"

/* --------------------- Functions ------------------ */

void UART_setup();
void UART_read(ControlPacket_ConstructionRobot *packet);
void UART_write(CurrVoltPacket_ConstructionRobot *packet);

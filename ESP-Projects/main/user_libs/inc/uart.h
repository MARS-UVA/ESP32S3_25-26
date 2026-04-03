#pragma once

#include "driver/uart.h"
#include "utils.h"
#include "packets.h"

#define S3_TX_PIN 43
#define S3_RX_PIN 44

/* --------------------- Functions ------------------ */

void UART_setup();
void UART_read(ControlPacket_OneRobot *packet);
void UART_write(ControlPacket_OneRobot *packet);
//void UART_write(CurrVoltPacket_OneRobot *packet);

void UARTWriteTemperature(TempPacket_OneRobot *packet);

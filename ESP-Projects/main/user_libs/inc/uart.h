#pragma once

#include "driver/uart.h"
#include "utils.h"
#include "packets.h"

#define C5_TX_PIN 23
#define C5_RX_PIN 24
#define C5_UART_PORT UART_NUM_0
#define C5_Clk SOC_MOD_CLK_PLL_F80M

#define S3_TX_PIN 43
#define S3_RX_PIN 44
#define S3_UART_PORT UART_NUM_1
#define S3_CLK UART_SCLK_DEFAULT

/* --------------------- Functions ------------------ */

void UART_setup();
void UART_read(ControlPacket_OneRobot *packet);
void UART_write(CurrVoltPacket_OneRobot *packet);

void UARTWriteTemperature(TempPacket_OneRobot *packet);

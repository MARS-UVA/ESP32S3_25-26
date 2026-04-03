#pragma once

#include "can.h"
#include "uart.h"
#include "wifi.h"
#include "packets.h"

#include "talonFX.h"
#include "talonSRX.h"

extern QueueHandle_t uart_queue;
void initializeTalons();
void directControl(ControlPacket_OneRobot pkt);

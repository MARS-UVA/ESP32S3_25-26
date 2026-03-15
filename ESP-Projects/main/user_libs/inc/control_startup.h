#pragma once

#include "can.h"
#include "uart.h"
#include "wifi.h"
#include "packets.h"

#include "talonFX.h"
#include "talonSRX.h"



extern QueueHandle_t uart_queue;
//void initializeTalons(PDH *pdh);
void directControl(ControlPacket_ConstructionRobot pkt);

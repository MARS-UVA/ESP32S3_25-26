#ifndef POWERLIB_CONTROL
#define POWERLIB_CONTROL

#include "can2.h"
#include "uart.h"

void initializeTalons();

void directControl(SerialPacket pkt);

#endif
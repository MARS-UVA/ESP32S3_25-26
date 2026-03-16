#pragma once

#include "can.h"
#include "uart.h"
#include "packets.h"

#include "talonFX.h"
#include "talonSRX.h"

// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 33 // 38
#define BACK_LEFT_WHEEL_ID 0 // 13
#define FRONT_RIGHT_WHEEL_ID 59 // 36
#define BACK_RIGHT_WHEEL_ID 37
#define BACK_BUCKET_DRUM_ID 25  // needs updating
#define FRONT_BUCKET_DRUM_ID 60
#define FRONT_ACTUATOR_ID 55
#define BACK_ACTUATOR_ID 16 // needs updating

// Define channel IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_CHANNEL_ID 11 // 12
#define BACK_LEFT_WHEEL_CHANNEL_ID 3 // 13
#define FRONT_RIGHT_WHEEL_CHANNEL_ID 10 // 3
#define BACK_RIGHT_WHEEL_CHANNEL_ID 1
#define BACK_BUCKET_DRUM_CHANNEL_ID 0
#define FRONT_BUCKET_DRUM_CHANNEL_ID 2
#define FRONT_ACTUATOR_CHANNEL_ID 15
#define BACK_ACTUATOR_CHANNEL_ID 14

extern QueueHandle_t uart_queue;
void initializeTalons(PDH *pdh);
void directControl(ControlPacket_OneRobot pkt);

#ifndef POWERLIB_CONTROL
#define POWERLIB_CONTROL

#include "can.h"
#include "uart.h"
#include "packets.h"

#include "talonFX.h"
#include "talonSRX.h"

// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 38
#define BACK_LEFT_WHEEL_ID 13
#define FRONT_RIGHT_WHEEL_ID 36
#define BACK_RIGHT_WHEEL_ID 37
#define BACK_BUCKET_DRUM_ID 25  // needs updating
#define FRONT_BUCKET_DRUM_ID 60
#define FRONT_ACTUATOR_ID 55
#define BACK_ACTUATOR_ID 16 // needs updating

// Define channel IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_CHANNEL_ID 12
#define BACK_LEFT_WHEEL_CHANNEL_ID 13
#define FRONT_RIGHT_WHEEL_CHANNEL_ID 3
#define BACK_RIGHT_WHEEL_CHANNEL_ID 1
#define BACK_BUCKET_DRUM_CHANNEL_ID 0
#define FRONT_BUCKET_DRUM_CHANNEL_ID 2
#define FRONT_ACTUATOR_CHANNEL_ID 15
#define BACK_ACTUATOR_CHANNEL_ID 14

extern QueueHandle_t uart_queue;
void initializeTalons();
void directControl(ControlPacket_OneRobot pkt);

#endif
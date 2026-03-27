#pragma once

#include "can.h"
#include "uart.h"
#include "wifi.h"
#include "packets.h"
#include "actuators.h"

#include "talonFX.h"
#include "talonSRX.h"

// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 36
#define BACK_LEFT_WHEEL_ID 37
#define FRONT_RIGHT_WHEEL_ID 38
#define BACK_RIGHT_WHEEL_ID 13
#define BUCKET_DRUM_RIGHT_ID 25
#define BUCKET_DRUM_LEFT_ID 60
#define LEFT_ACTUATOR_ID 16
#define RIGHT_ACTUATOR_ID 55

// Define channel IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_CHANNEL_ID 12
#define BACK_LEFT_WHEEL_CHANNEL_ID 13
#define FRONT_RIGHT_WHEEL_CHANNEL_ID 3
#define BACK_RIGHT_WHEEL_CHANNEL_ID 14
#define BUCKET_DRUM_RIGHT_CHANNEL_ID 4
#define BUCKET_DRUM_LEFT_CHANNEL_ID 2
#define LEFT_ACTUATOR_CHANNEL_ID 0
#define RIGHT_ACTUATOR_CHANNEL_ID 1

#define HALL_PIN_LEFT   16
#define HALL_PIN_RIGHT  17

#define ADC_PIN_LEFT ADC_CHANNEL_3
#define ADC_PIN_RIGHT ADC_CHANNEL_4

extern QueueHandle_t uart_queue;
extern int leftDirection;
extern int rightDirection;
void initializeTalons();
void directControl(ControlPacket pkt);
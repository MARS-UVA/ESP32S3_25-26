#pragma once

#include "can.h"
#include "uart.h"
#include "packets.h"
#include "talonFX.h"
#include "talonSRX.h"
#include "i2c.h"

// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 38 // 38
#define BACK_LEFT_WHEEL_ID 13 // 13
#define FRONT_RIGHT_WHEEL_ID 36 // 36
#define BACK_RIGHT_WHEEL_ID 37
#define BACK_BUCKET_DRUM_ID 25  
#define FRONT_BUCKET_DRUM_ID 60
#define FRONT_ACTUATOR_ID 16
#define BACK_ACTUATOR_ID 55 

// Define channel IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_CHANNEL_ID 12 // 12
#define BACK_LEFT_WHEEL_CHANNEL_ID 13 // 13
#define FRONT_RIGHT_WHEEL_CHANNEL_ID 3 // 3
#define BACK_RIGHT_WHEEL_CHANNEL_ID 1
#define BACK_BUCKET_DRUM_CHANNEL_ID 0
#define FRONT_BUCKET_DRUM_CHANNEL_ID 2
#define FRONT_ACTUATOR_CHANNEL_ID 15
#define BACK_ACTUATOR_CHANNEL_ID 14

// Define INA219
#define INA219_SENSOR_ADDR      65
#define INA219_REG_BUSVOLTAGE   0x02

extern QueueHandle_t uart_queue;
void initializeTalons(void);
void directControl(ControlPacket_OneRobot pkt);
void initAuxVoltageSensor(void);
void updateAuxVoltage(void);
float getAuxVoltage(void);

// TODO: Remove this function after testing
void test_run_motor();

TempPacket_OneRobot getTemperatureOneRobot();

void canSetupTalons();

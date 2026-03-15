#ifndef POWERLIB_CONTROL
#define POWERLIB_CONTROL

#include "can.h"
#include "uart.h"
#include "packets.h"

#include "talonFX.h"
#include "talonSRX.h"

//Excavation robot
// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 38
#define BACK_LEFT_WHEEL_ID 13
#define FRONT_RIGHT_WHEEL_ID 36
#define BACK_RIGHT_WHEEL_ID 37
#define BUCKET_LADDER_ID 25  // needs updating
#define CONVEYOR_BELT_ID 60
#define LEFT_TRACK_ACTUATOR_ID 55
#define RIGHT_TRACK_ACTUATOR_ID 16 // needs updating

// Define channel IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_CHANNEL_ID 12
#define BACK_LEFT_WHEEL_CHANNEL_ID 13
#define FRONT_RIGHT_WHEEL_CHANNEL_ID 3
#define BACK_RIGHT_WHEEL_CHANNEL_ID 1
#define BUCKET_LADDER_CHANNEL_ID 0
#define CONVEYOR_BELT_CHANNEL_ID 2
#define LEFT_TRACK_ACTUATOR_CHANNEL_ID 15
#define RIGHT_TRACK_ACTUATOR_CHANNEL_ID 14

//Construciton Robot
// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 38
#define BACK_LEFT_WHEEL_ID 13
#define FRONT_RIGHT_WHEEL_ID 36
#define BACK_RIGHT_WHEEL_ID 37
#define ACTUATOR_ID 25  // needs updating

// Define channel IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_CHANNEL_ID 12
#define BACK_LEFT_WHEEL_CHANNEL_ID 13
#define FRONT_RIGHT_WHEEL_CHANNEL_ID 3
#define BACK_RIGHT_WHEEL_CHANNEL_ID 1
#define ACTUATOR_CHANNEL_ID 0

extern QueueHandle_t uart_queue;
void initializeExcavationRobotTalons(PDH *pdh);
void initializeConstructionRobotTalons(PDH *pdh);
void directControlExcavationRobot(ControlPacket_ExcavationRobot pkt);
void directControlConstructionRobot(ControlPacket_ConstructionRobot pkt);

#endif
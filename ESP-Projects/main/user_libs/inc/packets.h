#pragma once

#include "utils.h"

//Construction Robot Packets
typedef struct __attribute__((packed))
{
    uint8_t invalid;
    uint8_t header;

    uint8_t front_left_wheel;
    uint8_t back_left_wheel;
    uint8_t front_right_wheel;
    uint8_t back_right_wheel;
    
    uint8_t actuator;
    uint8_t vibrator;
} ControlPacket_ConstructionRobot;
//jayradster hates kittens

typedef struct __attribute__((packed))
{
    uint8_t start;
    uint8_t invalid;
    uint8_t header;
    uint8_t reserved_bit1;
    uint8_t reserved_bit2;

    float front_left_wheel;
    float back_left_wheel;
    float front_right_wheel;
    float back_right_wheel;
    float actuator;
    float main_battery;
} CurrVoltPacket_ConstructionRobot;

typedef struct __attribute__((packed))
{
    uint8_t start;
    uint8_t invalid;
    uint8_t header;
    uint8_t reserved_bit1;
    uint8_t reserved_bit2;
    
    float front_left_wheel_temp;
    float back_left_wheel_temp;
    float front_right_wheel_temp;
    float back_right_wheel_temp;
} TempPacket_ConstructionRobot;

typedef struct __attribute__((packed))
{
    uint8_t start;
    uint8_t invalid;
    uint8_t header;
    uint8_t reserved_bit1;
    uint8_t reserved_bit2;

    float actuator_position;
} PositionPacket_ConstructionRobot;


CurrVoltPacket_ConstructionRobot Init_CurrVolt_Construction_Robot_Packet();
TempPacket_ConstructionRobot Init_Temp_Construction_Robot_Packet();
PositionPacket_ConstructionRobot Init_Position_Construction_Robot_Packet();
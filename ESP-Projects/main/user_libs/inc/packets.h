#pragma once

#include "utils.h"

//Excavation Robot Packets
typedef struct __attribute__((packed))
{
    uint8_t invalid;
    uint8_t header;

    uint8_t front_left_wheel;
    uint8_t back_left_wheel;
    uint8_t front_right_wheel;
    uint8_t back_right_wheel;
    
    uint8_t bucket_ladder;
    uint8_t conveyor_belt;
    uint8_t track_actuator;
} ControlPacket_ExcavationRobot;
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
    float bucket_ladder;
    float conveyor_belt;
    float left_track_actuator;
    float right_track_actuator;
    float main_battery;
    float aux_battery;
} CurrVoltPacket_ExcavationRobot;

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
    float bucket_ladder_temp;
    float conveyor_belt_temp;
} TempPacket_ExcavationRobot;

typedef struct __attribute__((packed))
{
    uint8_t start;
    uint8_t invalid;
    uint8_t header;
    uint8_t reserved_bit1;
    uint8_t reserved_bit2;

    float left_track_actuator_position;
    float right_track_actuator_position;    
} PositionPacket_ExcavationRobot;


CurrVoltPacket_ExcavationRobot Init_CurrVolt_Excavation_Robot_Packet();
TempPacket_ExcavationRobot Init_Temp_Excavation_Robot_Packet();
PositionPacket_ExcavationRobot Init_Position_Excavation_Robot_Packet();
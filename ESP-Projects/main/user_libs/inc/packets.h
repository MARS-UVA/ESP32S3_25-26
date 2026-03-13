#pragma once

#include "utils.h"

typedef struct __attribute__((packed))
{
    uint8_t invalid;
    uint8_t header;

    uint8_t front_left_wheel;
    uint8_t back_left_wheel;
    uint8_t front_right_wheel;
    uint8_t back_right_wheel;
    uint8_t front_bucket_drum;
    uint8_t back_bucket_drum;
    uint8_t front_actuator;
    uint8_t back_actuator;
} ControlPacket_OneRobot;

typedef struct __attribute__((packed))
{
    uint8_t invalid;
    uint8_t header;
    uint8_t reserved_bit1;
    uint8_t reserved_bit2;

    float front_left_wheel;
    float back_left_wheel;
    float front_right_wheel;
    float back_right_wheel;
    float front_drum;
    float back_drum;
    float front_actuator;
    float back_actuator;
    float main_battery;
    float aux_battery;
} CurrVoltPacket_OneRobot;

typedef struct __attribute__((packed))
{
    uint8_t invalid;
    uint8_t header;
    uint8_t reserved_bit1;
    uint8_t reserved_bit2;
    
    float front_left_wheel_temp;
    float back_left_wheel_temp;
    float front_right_wheel_temp;
    float back_right_wheel_temp;
    float front_drum_temp;
    float back_drum_temp;
} TempPacket_OneRobot;

typedef struct __attribute__((packed))
{
    uint8_t invalid;
    uint8_t header;
    uint8_t reserved_bit1;
    uint8_t reserved_bit2;

    float front_actuator_position;
    float back_actuator_position;    
} PositionPacket_OneRobot;


CurrVoltPacket_OneRobot Init_CurrVolt_Packet();
TempPacket_OneRobot Init_Temp_Packet();
PositionPacket_OneRobot Init_Position_Packet();
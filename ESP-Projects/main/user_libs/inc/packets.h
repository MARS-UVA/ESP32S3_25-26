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

    float front_left_wheel;
    float back_left_wheel;
    float front_right_wheel;
    float back_right_wheel;
    float front_bucket;
    float back_bucket;
    float front_actuator;
    float back_actuator;
} CurrVoltPacket_OneRobot;

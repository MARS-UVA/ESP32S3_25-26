#ifndef PACKETS_H
#define PACKETS_H

#include "utils.h"

typedef struct __attribute__((packed))
{
    uint8_t invalid;
    uint8_t header;

    uint8_t top_left_wheel;
    uint8_t back_left_wheel;
    uint8_t top_right_wheel;
    uint8_t back_right_wheel;
    uint8_t drum;
    uint8_t actuator;
} ControlPacket;

typedef struct __attribute__((packed))
{
    uint8_t invalid;
    uint8_t header;

    float top_left_wheel;
    float back_left_wheel;
    float top_right_wheel;
    float back_right_wheel;
    float bucket_left;
    float bucket_right;
    float left_actuator;
    float right_actuator;
} CurrVoltPacket;

#endif
#pragma once

#include "driver/uart.h"
#include <string.h>

typedef struct PIDController
{
    float kp;             // Proportional gain
    float ki;             // Integral gain
    float kd;             // Derivative gain
    float previous_error; // Previous error value
    float integral;       // Integral of the error
} PIDController;

PIDController initPID(float kp, float ki, float kd);

double computePID(PIDController *pid, float setpoint, float measured_value, float dt);
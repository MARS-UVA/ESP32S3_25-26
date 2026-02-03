#ifndef PID_H
#define PID_H

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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

#endif // PID_H

#pragma once

#include "pid.h"
#include "can.h"
#include "adc.h"
#include "pid.h"
#include "packets.h"
#include "talonSRX.h"

#include "esp_timer.h"
#include <math.h>
#include "driver/gpio.h"
#include "utils.h"

typedef struct Actuator
{
    TalonSRX *controller;
    Pot *pot;
    PIDController *pid;
    double prevPosition;
    double prevVelocity;
    double lastTime;
    double velocity;
    int *direction;
} Actuator;

Actuator initActuator(TalonSRX *talonSrx, Pot *pot, PIDController *pid, int *direction);
void hallEffectInit(int pinLeft, int pinRight);
void moveSyncActuatorsToPosition(Actuator *leftActuator, Actuator *rightActuator, double targetPosition);
void moveSyncActuatorsToVelocity(Actuator *leftActuator, Actuator *rightActuator, double targetVelocity);
PositionPacket calculatePulse(Actuator *leftActuator, Actuator *rightActuator);
void evaluatePot(Actuator *leftActuator, Actuator *rightActuator);
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

typedef struct
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
void hallEffectInit(int pinFront, int pinBack);
void moveSyncActuatorsToPosition(Actuator *frontActuator, Actuator *backActuator, double targetPosition);
void moveSyncActuatorsToVelocity(Actuator *frontActuator, Actuator *backActuator, double targetVelocity);
PositionPacket_OneRobot calculatePulse(Actuator *frontActuator, Actuator *backActuator);
void evaluatePot(Actuator *frontActuator, Actuator *backActuator);
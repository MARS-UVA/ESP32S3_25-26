#ifndef SYNCED_SRX_H
#define SYNCED_SRX_H

#include "../can/can2.h"
#include "../ADC/adc2.h"
#include "pid.h"

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
void calculatePulse(Actuator *leftActuator, Actuator *rightActuator);

#endif

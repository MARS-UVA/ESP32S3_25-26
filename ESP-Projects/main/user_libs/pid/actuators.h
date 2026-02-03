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
    uint32_t prevPosition;
    uint32_t prevVelocity;
    uint32_t lastTime;
} Actuator;

Actuator initActuator(TalonSRX *talonSrx, Pot *pot, PIDController *pid);
void moveSyncActuatorsToPosition(Actuator *leftActuator, Actuator *rightActuator, double targetPosition);
void moveSyncActuatorsToVelocity(Actuator *leftActuator, Actuator *rightActuator, double targetVelocity);

#endif

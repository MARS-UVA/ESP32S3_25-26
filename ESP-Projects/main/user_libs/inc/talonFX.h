/**
 * @file talonFX.h
 * @brief Header file for Talon FX motor controller functions.
 * 
 * @author Diana Lin
 * @author Carlos Giron
 * @author Anthony Vu
 * 
 * @copyright Copyright (c) 2026 Mechatronics and Robotics Society
 * @version 1.0
 * @date 2026-02-09
 */

#pragma once

#include "can.h"

typedef struct
{
    uint8_t id;
    float currentLimit;
    float kP;
    float kI;
    float kD;
    bool breakMode;
    uint8_t channel;
    float current;
    int temperature;
} TalonFX;

// Inits
TalonFX talonFXInit(uint8_t n_id, uint8_t c_id);

// FX FUNCS
void setFX(TalonFX *fx, float speed);
void setTargetFX(TalonFX *fx, int velocity);

void canSetupTalonFX(TalonFX **motors, size_t count);

int getTemperatureTalonFX(TalonFX *fx);

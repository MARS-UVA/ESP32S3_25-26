/**
 * @file talonFX.h
 * @brief Header file for Talon FX motor controller functions.
 * 
 * @author Diana Lin <xrc9wg@virginia.edu>
 * @author Carlos Giron <rdb7fq@virginia.edu>
 * @author Anthony Vu <anthonyvu@email.virginia.edu>
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

/**
 * @brief Initializes a TalonFX structure with default values.
 * 
 * @param n_id  CAN ID for the Talon FX (0-63).
 * @param c_id  Channel number for current monitoring (0-23).
 * @return Initialized TalonFX structure.
 */
TalonFX talonFXInit(uint8_t n_id, uint8_t c_id);

/**
 * @brief Set the duty cycle of the Talon FX motor controller.
 * 
 * @param fx    Pointer to the TalonFX structure representing the motor controller to control.
 * @param speed Desired speed as a duty cycle (0.0 to 1.0
 */
void setFX(TalonFX *fx, float speed);

/**
 * @brief Set the target velocity for the Talon FX motor controller using PID control.
 * 
 * @param fx        Pointer to the TalonFX structure representing the motor controller to control.
 * @param velocity  Desired velocity in encoder units per 100ms.
 */
void setTargetFX(TalonFX *fx, int velocity);

/**
 * @brief Set up CAN communication for the Talon FX motor controller, including registering the appropriate CAN RX handler.
 * 
 * @param fx    Pointer to the TalonFX structure representing the motor controller for which to set up CAN communication.
 */
void talonFXCanSetup(TalonFX *fx);
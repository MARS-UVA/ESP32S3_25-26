/**
 * @file TalonSRX.h
 * @brief Header file for Talon SRX motor controller functions.
 * 
 * @author Diana Lin
 * @author Carlos Giron
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
    bool inverted;
    uint8_t channel;
    float current;
} TalonSRX;

TalonSRX talonSRXInit(uint8_t n_id, uint8_t c_id, bool inv);
void setSRX(TalonSRX *srx, double value);

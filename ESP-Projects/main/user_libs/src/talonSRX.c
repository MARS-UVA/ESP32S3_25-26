/**
 * @file talonSRX.c
 * @brief Implements functions for controlling the Talon SRX motor controller.
 * 
 * @author Diana Lin
 * @author Carlos Giron
 * 
 * @copyright Copyright (c) 2026 Mechatronics and Robotics Society
 * @version 1.0
 * @date 2026-02-09
 */

#include "talonSRX.h"

TalonSRX talonSRXInit(uint8_t n_id, uint8_t c_id, bool inv)
{
    return (TalonSRX){
        .id = n_id,
        .inverted = inv,
        .channel = c_id,
        .current = 0.0f,
    };
}

void setInvertedSRX(TalonSRX *srx, bool invert)
{
    srx->inverted = invert;
}

// set the output magnitude of a Talon SRX
void setSRX(TalonSRX *srx, double value)
{
    // set the direction of a Talon SRX to be inverted
    int valueInt = (int)(value * 1023);
    uint8_t buff[] = {(valueInt >> 16) & 255, (valueInt >> 8) & 255, valueInt & 255, 0, 0, 0, 0x0b, srx->inverted ? 0x40 : 0x00};
    sendMsg(CAN_ID_SET_SRX, srx->id, buff, 8);
}

/**
 * @file talonFX.c
 * @brief Implements functions for controlling the Talon FX motor controller.
 * 
 * @author Diana Lin <xrc9wg@virginia.edu>
 * @author Carlos Giron <rdb7fq@virginia.edu>
 * @author Anthony Vu <anthonyvu@email.virginia.edu>
 * 
 * @copyright Copyright (c) 2026 Mechatronics and Robotics Society
 * @version 1.0
 * @date 2026-02-09
 */

#include "talonFX.h"
#include "can.h"
#include "utils.h"

/**
 * @brief Initializes a TalonFX structure with default values.
 * 
 * @param n_id  CAN ID for the Talon FX (0-63).
 * @param c_id  Channel number for current monitoring (0-23).
 * @return Initialized TalonFX structure.
 */
TalonFX talonFXInit(uint8_t n_id, uint8_t c_id)
{
    return (TalonFX){
        .id = n_id,
        .currentLimit = 0.0f,
        .kP = 0.0f,
        .kI = 0.0f,
        .kD = 0.0f,
        .breakMode = false,
        .channel = c_id,
        .current = 0.0f,
        .temperature = 0,
    };
}

void setFX(TalonFX *fx, float speed)
{
    uint8_t buff[] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    short valueInt = (short)(speed * 1024);
    if (valueInt < 0)
    {
        valueInt = 0xfff - (-1 * valueInt);
    }
    writeToBuffInd(buff, (uint8_t *)&valueInt, 6, 2);
    sendMsg(CAN_ID_SET_FX, fx->id, buff, 8);
}

void setTargetFX(TalonFX *fx, int velocity)
{
    if (velocity >= 0)
    {
        velocity *= 16;
    }
    else
    {
        velocity = 0x40000 - (-16 * velocity);
    }
    // Get feedforward value
    float feedforward = 0.1;
    int feedforwardInt = feedforward * 100;
    if (feedforward < 0)
        feedforwardInt = (~(feedforwardInt * -1)) + 1;

    uint8_t buff[] = {0, 1, velocity & 0xff, (velocity >> 8) & 0xff, velocity >> 16 & 0xff, 0, feedforwardInt & 0xff, (feedforwardInt >> 8) & 0xff};
    sendMsg(CAN_ID_SET_TARGET, fx->id, buff, 8);
}

/**
 * @internal
 * @brief Process an incoming CAN message containing Talon FX data and update the TalonFX structure accordingly.
 * 
 * @param fx        Pointer to the TalonFX structure to update.
 * @param rx_frame  Pointer to the received TWAI frame containing the CAN message data.
 * @param recv_buff Pointer to the raw data buffer from the received CAN frame.
 */
static void receiveCANTalonFX(TalonFX *fx, const twai_frame_t *rx_frame, uint64_t *recv_buff)
{
    if (rx_frame->header.id != (0x2044740 | fx->id)) {
        return;
    } 

    fx->temperature = extractBits(*recv_buff, 40, 8);
}

/**
 * @internal
 * @brief CAN RX callback function for Talon FX messages. This function is called by the CAN driver when a message is received.
 * It checks if the message is intended for the given TalonFX instance and updates its state accordingly.
 * 
 * @param context   Pointer to the TalonFX instance that should be updated based on the received CAN message.
 * @param frame     Pointer to the received TWAI frame containing the CAN message data.
 */
static void talonFXCanHandler(void *context, const twai_frame_t *frame)
{
    TalonFX *fx = (TalonFX *)context;
    receiveCANTalonFX(fx, frame, (uint64_t *)frame->buffer);
}

/*
 * @internal
 * @brief CAN RX context for Talon FX messages.
 */
static can_rx_context_t talonfx_rx_ctx = {
    .handler = talonFXCanHandler,
    .context = NULL,
};

void talonFXCanSetup(TalonFX *fx)
{
    talonfx_rx_ctx.context = fx;
    canSetup(&talonfx_rx_ctx);
}
#include "TalonFX.h"

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
    };
}

// FX CAN FUNCS
void setFX(TalonFX *fx, float speed) // set duty cycle for speed
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

void setTargetFX(TalonFX *fx, int velocity) // setting PID velocity
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
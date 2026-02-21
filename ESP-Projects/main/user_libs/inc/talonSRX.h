#ifndef TALONSRX_H
#define TALONSRX_H

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

#endif
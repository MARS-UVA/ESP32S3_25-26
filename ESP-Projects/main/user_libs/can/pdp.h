#ifndef PDP_H
#define PDP_H

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "can2.h"

// defined PDP struct
typedef struct
{
    int identifier;
    uint64_t cache0;
    uint64_t cache40;
    uint64_t cache80;
    short cacheWords[6];
    bool receivedNew0;
    bool receivedNew40;
    bool receivedNew80;
    float busVoltage;
} PDP;

// global pdp struct
extern PDP pdp;
extern twai_event_callbacks_t user_cbs;

// initiate PDP struct
void PDPInit(int identifier);

// request packets
void requestCurrentReadingsPDP();

// get current in amps
float getChannelCurrentPDP(int channelID);

#endif
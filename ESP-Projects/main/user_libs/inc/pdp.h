#pragma once

#include "can.h"
#include "control_startup.h"

// defined PDP struct
typedef struct
{
    int identifier;
    int16_t channelCurrents[18];
    float busVoltage;

    SemaphoreHandle_t sem00;
    SemaphoreHandle_t sem40;
    SemaphoreHandle_t sem80;

    StaticSemaphore_t _buf00;
    StaticSemaphore_t _buf40;
    StaticSemaphore_t _buf80;

    // uint64_t cache0;
    // uint64_t cache40;
    // uint64_t cache80;
    // short cacheWords[6];
    // bool receivedNew0;
    // bool receivedNew40;
    // bool receivedNew80;
    // float busVoltage;
} PDP;

// global pdp struct
// extern twai_event_callbacks_t can_cbs;

extern TalonFX *fxMotors[];
extern TalonSRX *srxMotors[];

// initiate PDP struct
void PDPInit(PDP *pdp, int identifier);

// request packets
void requestCurrentReadingsPDP(PDP *pdp);

// wait for current updates to be received
bool awaitCurrentReadingsPDP(PDP *pdp, int waittime_ms);

// get current in amps
float getChannelCurrentPDP(PDP *pdp, int channelID);

// RX cb functions
bool pdp_twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *pdp);

void current_update_task(PDP *pdp);

void canSetupPDP(PDP *pdp);

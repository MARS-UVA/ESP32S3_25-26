#ifndef PDP_H
#define PDP_H

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "can2.h"
#include "control_startup.h"

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
extern twai_event_callbacks_t can_cbs;

extern TalonFX *fxMotors[];
extern TalonSRX *srxMotors[];

// initiate PDP struct
void PDPInit(int identifier);

// request packets
void requestCurrentReadingsPDP();

// get current in amps
float getChannelCurrentPDP(int channelID);

// RX cb function
bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx);

void current_update_task();

#endif
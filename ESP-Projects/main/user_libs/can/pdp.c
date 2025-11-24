#include "utils.h"
#include "can2.h"
#include "pdp.h"

uint8_t prompt_buff[6] = {0x00, 0x00, 0x00, 0x00, 0x20, 0x00}; // buffer to prompt PDP for current readings
uint32_t current_request_id = 0x8041640;                       // CAN ID to request current readings from PDP

// PDP struct instance
PDP pdp = {
    .identifier = 0,
    .cache0 = 0,
    .cache40 = 0,
    .cache80 = 0,
    .cacheWords = {0},
    .receivedNew0 = false,
    .receivedNew40 = false,
    .receivedNew80 = false,
    .busVoltage = 0,
};

void requestCurrentReadingsPDP() // send CAN packet to PDP to request current readings
{
    sendEn();
    sendMsg(current_request_id, pdp.identifier, prompt_buff, 6); // This line sends a CAN packet to the PDP
}

// decode current values from the CAN packet values stored in the cache
void getSixParamPDP(uint64_t *cache)
{

    pdp.cacheWords[0] = (uint8_t)*cache;

    short *numPtr1 = pdp.cacheWords;
    numPtr1[0] = (short)(numPtr1[0] << 2);

    short *numPtr2 = pdp.cacheWords;
    numPtr2[0] = (short)(numPtr2[0] | ((short)((*cache >> 14) & ((unsigned long)3))));
    pdp.cacheWords[1] = (short)((*cache >> 8) & 0x3f);

    short *numPtr3 = &(pdp.cacheWords[1]);
    numPtr3[0] = (short)(numPtr3[0] << 4);

    short *numPtr4 = &(pdp.cacheWords[1]);
    numPtr4[0] = (short)(numPtr4[0] | ((short)((*cache >> 20) & 15)));
    pdp.cacheWords[2] = (short)((*cache >> 0x10) & 15);

    short *numPtr5 = &(pdp.cacheWords[2]);
    numPtr5[0] = (short)(numPtr5[0] << 6);

    short *numPtr6 = &(pdp.cacheWords[2]);
    numPtr6[0] = (short)(numPtr6[0] | ((short)((*cache >> 0x1a) & 0x3f)));
    pdp.cacheWords[3] = (short)((*cache >> 0x18) & ((unsigned long)3));

    short *numPtr7 = &(pdp.cacheWords[3]);
    numPtr7[0] = (short)(numPtr7[0] << 8);

    short *numPtr8 = &(pdp.cacheWords[3]);
    numPtr8[0] = (short)(numPtr8[0] | ((uint8_t)(*cache >> 0x20)));
    pdp.cacheWords[4] = (*cache >> 40);

    short *numPtr9 = &(pdp.cacheWords[4]);
    numPtr9[0] = (short)(numPtr9[0] << 2);

    short *numPtr10 = &(pdp.cacheWords[4]);
    numPtr10[0] = (short)(numPtr10[0] | ((short)((*cache >> 0x36) & ((unsigned long)3))));
    pdp.cacheWords[5] = (short)((*cache >> 0x30) & 0x3f);

    short *numPtr11 = &(pdp.cacheWords[5]);
    numPtr11[0] = (short)(numPtr11[0] << 4);

    short *numPtr12 = &(pdp.cacheWords[5]);
    numPtr12[0] = (short)(numPtr12[0] | ((short)((*cache >> 60) & 15)));
}

// given PDP channel ID, returns the current in Amps at the channel
float getChannelCurrentPDP(int channelID)
{
    float num = 0;
    if (channelID >= 0 && channelID <= 5)
    {
        getSixParamPDP(&(pdp.cache0));
        num = pdp.cacheWords[channelID] * 0.125;
    }
    else if (channelID >= 6 && channelID <= 11)
    {
        getSixParamPDP(&(pdp.cache40));
        num = pdp.cacheWords[channelID - 6] * 0.125;
    }
    else
    {
        getSixParamPDP(&(pdp.cache80));
        num = pdp.cacheWords[channelID - 12] * 0.125;
    }
    return num;
}

// process CAN packets received from the PDP (either current or voltage readings)
void receiveCANPDP(twai_frame_t *msg, uint64_t *data)
{
    // if not correct pdp id
    if ((msg->header.id & pdp.identifier) != pdp.identifier)
        return;

    if ((msg->header.id & 0x8041400) == 0x8041400) // current readings
    {
        switch (msg->header.id & 0xc0)
        {
        case 0x00:
            pdp.cache0 = *data;
            pdp.receivedNew0 = true;
            break;
        case 0x40:
            pdp.cache40 = *data;
            pdp.receivedNew40 = true;
            break;
        case 0x80:
            pdp.cache80 = *data;
            pdp.receivedNew80 = true;
            break;
        }
    }
}

//
bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff)};

    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame))
    {
        receiveCANPDP(&rx_frame, (uint64_t *)&recv_buff);
    }
    return false;
}

// struct for callback
twai_event_callbacks_t can_cbs = {
    .on_rx_done = twai_rx_cb,
};

// initiate PDP struct
void PDPInit(int identifier)
{
    pdp.identifier = identifier;
}
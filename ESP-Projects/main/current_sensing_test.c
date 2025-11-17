#include <stdio.h>
#include "esp_twai.h"
#include "esp_twai_onchip.h"

twai_node_handle_t node_hdl = NULL;
uint8_t en_buff[2] = {0x01, 0x00};
uint8_t send_buff[8] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
uint8_t prompt_buff[6] = {0x00, 0x00, 0x00, 0x00, 0x20, 0x00};

twai_onchip_node_config_t node_config = {
    .io_cfg.tx = 2,                // TWAI TX GPIO pin
    .io_cfg.rx = 1,                // TWAI RX GPIO pin
    .bit_timing.bitrate = 1000000, // 200 kbps bitrate
    .tx_queue_depth = 32,          // Transmit queue depth set to 16
};

twai_frame_t en_msg = {
    .header.id = 0x401bf,          // Message ID
    .header.ide = true,            // Use 29-bit extended ID format
    .buffer = en_buff,             // Pointer to data to transmit
    .buffer_len = sizeof(en_buff), // Length of data to transmit
};

twai_frame_t forward_msg = {
    .header.id = 0x204b540 | 37,     // Message ID
    .header.ide = true,              // Use 29-bit extended ID format
    .buffer = send_buff,             // Pointer to data to transmit
    .buffer_len = sizeof(send_buff), // Length of data to transmit
};

twai_frame_t promptChannel_msg = {
    .header.id = 0x8041640 | 62,       // Message ID
    .header.ide = true,                // Use 29-bit extended ID format
    .buffer = prompt_buff,             // Pointer to data to transmit
    .buffer_len = sizeof(prompt_buff), // Length of data to transmit
};

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

void requestCurrentReadingsPDP(PDP *pdp)
{
    ESP_ERROR_CHECK(twai_node_transmit(node_hdl, &promptChannel_msg, 100)); // Timeout = 0: returns immediately if queue is full
}

// decode current values from the CAN packet values stored in the cache
void getSixParamPDP(PDP *pdp, uint64_t *cache)
{

    pdp->cacheWords[0] = (uint8_t)*cache;

    short *numPtr1 = pdp->cacheWords;
    numPtr1[0] = (short)(numPtr1[0] << 2);

    short *numPtr2 = pdp->cacheWords;
    numPtr2[0] = (short)(numPtr2[0] | ((short)((*cache >> 14) & ((unsigned long)3))));
    pdp->cacheWords[1] = (short)((*cache >> 8) & 0x3f);

    short *numPtr3 = &(pdp->cacheWords[1]);
    numPtr3[0] = (short)(numPtr3[0] << 4);

    short *numPtr4 = &(pdp->cacheWords[1]);
    numPtr4[0] = (short)(numPtr4[0] | ((short)((*cache >> 20) & 15)));
    pdp->cacheWords[2] = (short)((*cache >> 0x10) & 15);

    short *numPtr5 = &(pdp->cacheWords[2]);
    numPtr5[0] = (short)(numPtr5[0] << 6);

    short *numPtr6 = &(pdp->cacheWords[2]);
    numPtr6[0] = (short)(numPtr6[0] | ((short)((*cache >> 0x1a) & 0x3f)));
    pdp->cacheWords[3] = (short)((*cache >> 0x18) & ((unsigned long)3));

    short *numPtr7 = &(pdp->cacheWords[3]);
    numPtr7[0] = (short)(numPtr7[0] << 8);

    short *numPtr8 = &(pdp->cacheWords[3]);
    numPtr8[0] = (short)(numPtr8[0] | ((uint8_t)(*cache >> 0x20)));
    pdp->cacheWords[4] = (*cache >> 40);

    short *numPtr9 = &(pdp->cacheWords[4]);
    numPtr9[0] = (short)(numPtr9[0] << 2);

    short *numPtr10 = &(pdp->cacheWords[4]);
    numPtr10[0] = (short)(numPtr10[0] | ((short)((*cache >> 0x36) & ((unsigned long)3))));
    pdp->cacheWords[5] = (short)((*cache >> 0x30) & 0x3f);

    short *numPtr11 = &(pdp->cacheWords[5]);
    numPtr11[0] = (short)(numPtr11[0] << 4);

    short *numPtr12 = &(pdp->cacheWords[5]);
    numPtr12[0] = (short)(numPtr12[0] | ((short)((*cache >> 60) & 15)));
}

// given PDP channel ID, returns the current in Amps at the channel
float getChannelCurrentPDP(PDP *pdp, int channelID)
{
    float num = 0;
    if (channelID >= 0 && channelID <= 5)
    {
        getSixParamPDP(pdp, &(pdp->cache0));
        num = pdp->cacheWords[channelID] * 0.125;
    }
    else if (channelID >= 6 && channelID <= 11)
    {
        getSixParamPDP(pdp, &(pdp->cache40));
        num = pdp->cacheWords[channelID - 6] * 0.125;
    }
    else
    {
        getSixParamPDP(pdp, &(pdp->cache80));
        num = pdp->cacheWords[channelID - 12] * 0.125;
    }

    return num;
}

// process CAN packets received from the PDP (either current or voltage readings)
void receiveCANPDP(PDP *pdp, twai_frame_t *msg, uint64_t *data)
{
    // not correct pdp id
    if ((msg->header.id & pdp->identifier) != pdp->identifier)
        return;

    if ((msg->header.id & 0x8041400) == 0x8041400) // current readings
    {
        switch (msg->header.id & 0xc0)
        {
        case 0x00:
            pdp->cache0 = *data;
            pdp->receivedNew0 = true;
            break;
        case 0x40:
            pdp->cache40 = *data;
            pdp->receivedNew40 = true;
            break;
        case 0x80:
            pdp->cache80 = *data;
            pdp->receivedNew80 = true;
            break;
        }
    }
}

PDP PDPInit(int identifier)
{
    PDP pdp = {
        .identifier = identifier,
        .cache0 = 0,
        .cache40 = 0,
        .cache80 = 0,
        .cacheWords = {0},
        .receivedNew0 = false,
        .receivedNew40 = false,
        .receivedNew80 = false,
        .busVoltage = 0,
    };

    return pdp;
}

PDP panel;
static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff)};

    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame))
    {
        receiveCANPDP(&panel, &rx_frame, (uint64_t *)&recv_buff);
    }
    return false;
}

twai_event_callbacks_t user_cbs = {
    .on_rx_done = twai_rx_cb,
};

void app_main()
{
    panel = PDPInit(62);
    float current;
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &user_cbs, NULL));
    ESP_ERROR_CHECK(twai_node_enable(node_hdl));
    for (;;)
    {
        ESP_ERROR_CHECK(twai_node_transmit(node_hdl, &en_msg, -1));            // Timeout = 0: returns immediately if queue is full
        ESP_ERROR_CHECK(twai_node_transmit(node_hdl, &forward_msg, -1));       // Timeout = 0: returns immediately if queue is full
        ESP_ERROR_CHECK(twai_node_transmit(node_hdl, &promptChannel_msg, -1)); // Timeout = 0: returns immediately if queue is full
        current = getChannelCurrentPDP(&panel, 13);
        printf("\nCurrent read at channel %d -\t %.3f", 13, current);
    }
}
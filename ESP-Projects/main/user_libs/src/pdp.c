#include "utils.h"
#include "can2.h"
#include "pdp.h"
#include "control_startup.h"

void requestCurrentReadingsPDP(PDP *pdp) // send CAN packet to PDP to request current readings
{
    uint8_t prompt_buff[6] = {0x00, 0x00, 0x00, 0x00, 0x20, 0x00}; // buffer to prompt PDP for current readings
    uint32_t current_request_id = 0x8041640;                       // CAN ID to request current readings from PDP
    sendEn();
    sendMsg(current_request_id, pdp->identifier, prompt_buff, 6); // This line sends a CAN packet to the PDP
}

bool awaitCurrentReadingsPDP(PDP *pdp, int waittime_ms)
{
    // TODO: do this better
    if (xSemaphoreTake(pdp->sem00, waittime_ms / portTICK_PERIOD_MS) == pdFALSE)
        return false;
    if (xSemaphoreTake(pdp->sem40, waittime_ms / portTICK_PERIOD_MS) == pdFALSE)
        return false;
    if (xSemaphoreTake(pdp->sem80, waittime_ms / portTICK_PERIOD_MS) == pdFALSE)
        return false;
    return true;
}

void getSixParamPDP(short *cache_words, uint64_t *packet_data)
{
    cache_words[0] = (uint8_t)*packet_data;

    short *numPtr1 = cache_words;
    numPtr1[0] = (short)(numPtr1[0] << 2);

    short *numPtr2 = cache_words;
    numPtr2[0] = (short)(numPtr2[0] | ((short)((*packet_data >> 14) & ((unsigned long)3))));
    cache_words[1] = (short)((*packet_data >> 8) & 0x3f);

    short *numPtr3 = &(cache_words[1]);
    numPtr3[0] = (short)(numPtr3[0] << 4);

    short *numPtr4 = &(cache_words[1]);
    numPtr4[0] = (short)(numPtr4[0] | ((short)((*packet_data >> 20) & 15)));
    cache_words[2] = (short)((*packet_data >> 0x10) & 15);

    short *numPtr5 = &(cache_words[2]);
    numPtr5[0] = (short)(numPtr5[0] << 6);

    short *numPtr6 = &(cache_words[2]);
    numPtr6[0] = (short)(numPtr6[0] | ((short)((*packet_data >> 0x1a) & 0x3f)));
    cache_words[3] = (short)((*packet_data >> 0x18) & ((unsigned long)3));

    short *numPtr7 = &(cache_words[3]);
    numPtr7[0] = (short)(numPtr7[0] << 8);

    short *numPtr8 = &(cache_words[3]);
    numPtr8[0] = (short)(numPtr8[0] | ((uint8_t)(*packet_data >> 0x20)));
    cache_words[4] = (*packet_data >> 40);

    short *numPtr9 = &(cache_words[4]);
    numPtr9[0] = (short)(numPtr9[0] << 2);

    short *numPtr10 = &(cache_words[4]);
    numPtr10[0] = (short)(numPtr10[0] | ((short)((*packet_data >> 0x36) & ((unsigned long)3))));
    cache_words[5] = (short)((*packet_data >> 0x30) & 0x3f);

    short *numPtr11 = &(cache_words[5]);
    numPtr11[0] = (short)(numPtr11[0] << 4);

    short *numPtr12 = &(cache_words[5]);
    numPtr12[0] = (short)(numPtr12[0] | ((short)((*packet_data >> 60) & 15)));
}

// given PDP channel ID, returns the current in Amps at the channel
float getChannelCurrentPDP(PDP *pdp, int channelID)
{
    return pdp->channelCurrents[channelID] * 0.125f;
}

// process CAN packets received from the PDP (either current or voltage readings)
void receiveCANPDP(PDP *pdp, twai_frame_t *msg, uint64_t *data)
{
    // if not correct pdp id
    if ((msg->header.id & pdp->identifier) != pdp->identifier)
        return;

    // not current readings
    if ((msg->header.id & 0x8041400) != 0x8041400)
        return;

    SemaphoreHandle_t sem;
    int16_t *cache;

    switch (msg->header.id & 0xc0)
    {
    case 0x00:
        cache = &(pdp->channelCurrents[0]);
        sem = pdp->sem00;
        break;
    case 0x40:
        cache = &(pdp->channelCurrents[6]);
        sem = pdp->sem40;
        break;
    case 0x80:
        cache = &(pdp->channelCurrents[12]);
        sem = pdp->sem80;
        break;
    default:
        return;
    }

    getSixParamPDP(cache, data);
    xSemaphoreGive(sem);
}

bool pdp_twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *pdp)
{
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff)};

    if (ESP_OK != twai_node_receive_from_isr(handle, &rx_frame))
    {
        return false;
    }

    receiveCANPDP((PDP *)pdp, &rx_frame, (uint64_t *)&recv_buff);
    return false;
}

// initiate PDP struct
void PDPInit(PDP *pdp, int identifier)
{
    pdp->identifier = identifier;
    pdp->busVoltage = 0;
    for (int i = 0; i < 18; i++)
    {
        pdp->channelCurrents[i] = 0;
    }

    pdp->sem00 = xSemaphoreCreateBinaryStatic(&pdp->_buf00);
    pdp->sem40 = xSemaphoreCreateBinaryStatic(&pdp->_buf40);
    pdp->sem80 = xSemaphoreCreateBinaryStatic(&pdp->_buf80);
}

void canSetupPDP(PDP *pdp)
{
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TX_GPIO_NUM,            // TWAI TX GPIO pin
        .io_cfg.rx = RX_GPIO_NUM,            // TWAI RX GPIO pin
        .bit_timing.bitrate = ROBOT_BITRATE, // 1Mbps bitrate
        .tx_queue_depth = 32,                // Transmit queue depth set to 32
    };
    twai_event_callbacks_t can_cbs = {
        .on_rx_done = pdp_twai_rx_cb,
    };
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &g_node_hdl));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(g_node_hdl, &can_cbs, pdp));
    ESP_ERROR_CHECK(twai_node_enable(g_node_hdl));
}
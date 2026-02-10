/**
 * @file PDH.c
 * @brief Interprets information from the Rev Power Distribution Hub (PDH).
 *
 * @author Carlos Giron <rdb7fq@virginia.edu>
 * @author Cole Luba <meq2fg@virginia.edu>
 * @author Anthony Vu <hsh6ff@email.virginia.edu>
 * @author Aedan Stewart <aedan@email.virginia.edu>
 * @copyright Copyright (c) 2026 Mechatronics and Robotics Society
 * @note Derivative work based on 'PDP.c' authored by Diana Lin.
 * @note Assisted by AI (GPT-5 & Gemini 3 Flash) for decoding logic and comments.
 * @version 1.1
 * @date 2026-02-09
 */

#include "utils.h"
#include "can2.h"
#include "pdh.h"
#include "control_startup.h"

/**
 * @brief Extract a contiguous bit-field from a 64-bit value.
 *
 * Bits are numbered starting from bit 0 (least-significant bit).
 *
 * @param data      Source 64-bit value.
 * @param startBit  Index of first bit to extract.
 * @param bitLength Number of bits to extract.
 * @return Extracted value, right-aligned.
 */
static inline uint32_t extractBits(uint64_t data,
                                   uint8_t startBit,
                                   uint8_t bitLength)
{
    if (bitLength == 0 || bitLength > 32 || startBit >= 64)
        return 0;

    uint64_t mask = (1ULL << bitLength) - 1ULL;
    return (uint32_t)((data >> startBit) & mask);
}

/**
 * @brief Decode a PDH current-data CAN payload.
 *
 * Each payload contains up to 6 channels, each encoded as a 10-bit value.
 * A 2-bit gap exists between the third and fourth channels.
 *
 * @param payload      Raw 64-bit CAN payload.
 * @param out          Destination array for decoded channel values.
 * @param channelCount Number of channels to decode.
 */
void decodePDHFrame(uint64_t payload,
                    uint16_t *out,
                    int channelCount)
{
    uint8_t bitIndex = 0;

    for (int i = 0; i < channelCount; i++)
    {
        out[i] = (uint16_t)extractBits(payload,
                                       bitIndex,
                                       PDH_BITS_PER_CHANNEL);

        // Skip the inter-channel gap after the third channel
        bitIndex += PDH_BITS_PER_CHANNEL + ((bitIndex == 30) ? PDH_CHANNEL_GAP_BITS : 0);
    }
}

float getChannelCurrentPDH(PDH *pdh, int channel)
{
    // erm...
    if (channel < 0)
        return 0.0f;

    // TODO: Low-current channels not implemented (?)
    if (channel >= 20)
        return 0.0f;

    return pdh->channelCurrents[channel] * PDH_HIGH_CURRENT_LSB_A;
}

/**
 * @brief Process an incoming CAN message intended for the PDH.
 *
 * Extended ID format:
 *   0x80518XY
 *     └──┬──┘└─ channel group selector
 *        └──── PDH device ID
 *
 * @param msg  CAN Rx header.
 * @param data Pointer to 64-bit payload.
 */
void receiveCANPDH(PDH *pdh, twai_frame_t *msg, uint64_t *data)
{
    // Match PDH device ID (upper bits of the extended ID)
    if ((msg->header.id & 0xFFFFF00) != 0x8051800) // 0x805183E
        return;

    uint32_t channelGroup = (msg->header.id & PDH_CHANNEL_GROUP_MASK);

    // SemaphoreHandle_t sem;
    uint16_t *current_arr_ptr;

    if (channelGroup == (PDH_GROUP_BASE_0_TO_5 | pdh->identifier))
    {
        current_arr_ptr = &(pdh->channelCurrents[0]);
        // sem = pdh->sem_0_5;
    }
    else if (channelGroup == (PDH_GROUP_BASE_6_TO_11 | pdh->identifier))
    {
        current_arr_ptr = &(pdh->channelCurrents[6]);
        // sem = pdh->sem_6_11;
    }
    else if (channelGroup == (PDH_GROUP_BASE_12_TO_17 | pdh->identifier))
    {
        current_arr_ptr = &(pdh->channelCurrents[12]);
        // sem = pdh->sem_12_17;
    }
    else if (channelGroup == (PDH_GROUP_BASE_18_TO_24 | pdh->identifier))
    {
        current_arr_ptr = &(pdh->channelCurrents[18]);
        // sem = pdh->sem_18_24;
    }
    else
    {
        return;
    }

    decodePDHFrame(*data, current_arr_ptr, 6);
    // xSemaphoreGive(sem);
}

bool pdh_twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *pdh)
{
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff)};

    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame))
    {
        receiveCANPDH((PDH *)pdh, &rx_frame, (uint64_t *)&recv_buff);
    }
    return false;
}

void canSetupPDH(PDH *pdh)
{
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TX_GPIO_NUM,            // TWAI TX GPIO pin
        .io_cfg.rx = RX_GPIO_NUM,            // TWAI RX GPIO pin
        .bit_timing.bitrate = ROBOT_BITRATE, // 1Mbps bitrate
        .tx_queue_depth = 32,                // Transmit queue depth set to 32
    };
    twai_event_callbacks_t can_cbs = {
        .on_rx_done = pdh_twai_rx_cb,
    };
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &g_node_hdl));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(g_node_hdl, &can_cbs, pdh));
    ESP_ERROR_CHECK(twai_node_enable(g_node_hdl));
}

void current_update_task(PDH *pdh)
{

    while (1)
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            fxMotors[i]->current = getChannelCurrentPDH(pdh, fxMotors[i]->channel);
            // fxMotors[i]->current = 1.0;
            // vTaskDelay(1);
        }
        for (uint8_t i = 0; i < 2; i++)
        {
            srxMotors[i]->current = getChannelCurrentPDH(pdh, srxMotors[i]->channel);
            // srxMotors[i]->current = 2.0;
            // vTaskDelay(1);
        }
        vTaskDelay(1000);
        // ESP_LOGI("CURRENT TEST", "Current:\t%.3f\n", fxMotors[0]->current);
    }
}

// initiate PDP struct
void PDHInit(PDH *pdh, int identifier)
{
    pdh->identifier = identifier;

    for (int i = 0; i < 25; i++)
    {
        pdh->channelCurrents[i] = 0;
    }
}

// void printstuff(void)
// {
//     showData((uint8_t *)&(pdh.cacheChannels6to11), 8);
// }
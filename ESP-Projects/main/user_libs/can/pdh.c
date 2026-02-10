/**
 * @file PDH.c
 * @brief Interprets information from the Rev Power Distribution Hub (PDH).
 *
 * @author Carlos Giron <rdb7fq@virginia.edu>
 * @author Cole Luba <meq2fg@virginia.edu>
 * @author Anthony Vu <hsh6ff@email.virginia.edu>
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

// PDH struct instance
PDH pdh = {
    .identifier = 0,
    .cacheChannels0to5 = 0,
    .cacheChannels6to11 = 0,
    .cacheChannels12to17 = 0,
    .cacheChannels18to24 = 0,
    .receivedNew0to5 = false,
    .receivedNew6to11 = false,
    .receivedNew12to17 = false,
    .receivedNew18to24 = false,
};

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

float getChannelCurrentPDH(int channel)
{
    uint16_t currents[6];

    if (channel < 0)
        return 0.0f;

    if (channel < 6)
    {
        decodePDHFrame(pdh.cacheChannels0to5, currents, 6);
        return currents[channel] * PDH_HIGH_CURRENT_LSB_A;
    }
    else if (channel < 12)
    {
        decodePDHFrame(pdh.cacheChannels6to11, currents, 6);
        return currents[channel - 6] * PDH_HIGH_CURRENT_LSB_A;
    }
    else if (channel < 18)
    {
        decodePDHFrame(pdh.cacheChannels12to17, currents, 6);
        return currents[channel - 12] * PDH_HIGH_CURRENT_LSB_A;
    }
    else if (channel < 20)
    {
        decodePDHFrame(pdh.cacheChannels18to24, currents, 6);
        return currents[channel - 18] * PDH_HIGH_CURRENT_LSB_A;
    }
    // TODO: Implement low-current channel decoding
    return 0.0f;
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
void receiveCANPDH(twai_frame_t *msg, uint64_t *data)
{
    // Match PDH device ID (upper bits of the extended ID)
    if ((msg->header.id & 0xFFFFF00) != 0x8051800)  // 0x805183E
        return;
    // Decode which channel group this packet contains
    uint32_t channelGroup = (msg->header.id & PDH_CHANNEL_GROUP_MASK);
    {
    if (channelGroup == (PDH_GROUP_BASE_0_TO_5 | pdh.identifier)) {
        pdh.cacheChannels0to5 = *data;
        pdh.receivedNew0to5 = true;
    }
    else if (channelGroup == (PDH_GROUP_BASE_6_TO_11 | pdh.identifier)) {
        pdh.cacheChannels6to11 = *data;
        pdh.receivedNew6to11 = true;
    }
    else if(channelGroup == (PDH_GROUP_BASE_12_TO_17 | pdh.identifier)) {
        pdh.cacheChannels12to17 = *data;
        pdh.receivedNew12to17 = true;
    }
    else if(channelGroup == (PDH_GROUP_BASE_18_TO_24 | pdh.identifier)) {
        pdh.cacheChannels18to24 = *data;
        pdh.receivedNew18to24 = true;
    }       
     // Different PDH or unknown packet type — ignore
}}

bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff)};

    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame))
    {
        receiveCANPDH(&rx_frame, (uint64_t *)&recv_buff);
    }
    return false;
}

// struct for callback
twai_event_callbacks_t can_cbs = {
    .on_rx_done = twai_rx_cb,
};

void current_update_task()
{

    while (1)
    {
        requestCurrentReadingsPDH();
        for (uint8_t i = 0; i < 6; i++)
        {
            // requestCurrentReadingsPDP();
            fxMotors[i]->current = getChannelCurrentPDH(fxMotors[i]->channel);
            // fxMotors[i]->current = 1.0;
            // vTaskDelay(1);
        }
        for (uint8_t i = 0; i < 2; i++)
        {
            // requestCurrentReadingsPDP();
            srxMotors[i]->current = getChannelCurrentPDH(srxMotors[i]->channel);
            // srxMotors[i]->current = 2.0;
            // vTaskDelay(1);
        }
        vTaskDelay(1000);
        // ESP_LOGI("CURRENT TEST", "Current:\t%.3f\n", fxMotors[0]->current);
    }
}

// initiate PDP struct
void PDHInit(int identifier)
{
    pdh.identifier = identifier;
}

void printstuff(void)
{
    showData((uint8_t *)&(pdh.cacheChannels6to11), 8);
}
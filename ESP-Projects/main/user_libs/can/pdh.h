/**
 * @file PDH.h
 * @brief Defines the structure for the Rev Power Distribution Hub (PDH).
 *
 * @author Carlos Giron <rdb7fq@virginia.edu>
 * @author Anthony Vu <hsh6ff@virginia.edu>
 * @copyright Copyright (c) 2026 Mechatronics and Robotics Society
 * @note Derivative work based on 'PDH.h' authored by Diana Lin.
 * @note Assisted by AI (GPT-5 & Gemini 3 Flash) for decoding logic and comments.
 * @version 1.1
 * @date 2026-02-09
 */

#ifndef PDH_H
#define PDH_H

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "can2.h"
#include "control_startup.h"

// CAN ID Definitions

// Mask for the lower nibble selecting which channel group is present
#define PDH_CHANNEL_GROUP_MASK 0xFF

// Values of the channel-group nibble
#define PDH_GROUP_BASE_0_TO_5 0x00
#define PDH_GROUP_BASE_6_TO_11 0x40
#define PDH_GROUP_BASE_12_TO_17 0x80
#define PDH_GROUP_BASE_18_TO_24 0xC0

// Each PDH current channel is encoded as a 10-bit value
#define PDH_BITS_PER_CHANNEL 10
#define PDH_CHANNEL_GAP_BITS 2

// High-current channel resolution (Amps per LSB)
#define PDH_HIGH_CURRENT_LSB_A 0.125f

/**
 * @brief Power Distribution Hub (PDH) driver state.
 *
 * This structure stores cached CAN payloads from the PDH as well as
 * function pointers for decoding current measurements.
 */
typedef struct
{
    int identifier;

    // Cached CAN payloads containing channel current data
    uint64_t cacheChannels0to5;
    uint64_t cacheChannels6to11;
    uint64_t cacheChannels12to17;
    uint64_t cacheChannels18to24;

    // Flags indicating new data has been received for each cache
    bool receivedNew0to5;
    bool receivedNew6to11;
    bool receivedNew12to17;
    bool receivedNew18to24;

} PDH;

// global pdp struct
extern twai_event_callbacks_t can_cbs;

extern TalonFX *fxMotors[];
extern TalonSRX *srxMotors[];

// initiate PDP struct
void PDHInit(int identifier);

// request packets
void requestCurrentReadingsPDH();

/**
 * @brief Get the current (in Amps) for a specific PDH channel.
 *
 * @param pdh     PDH instance.
 * @param channel Channel index (0–19).
 * @return Channel current in Amps, or 0.0f if unsupported.
 */
float getChannelCurrentPDH(int channelID);

// RX cb function
bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx);

void current_update_task();

#endif
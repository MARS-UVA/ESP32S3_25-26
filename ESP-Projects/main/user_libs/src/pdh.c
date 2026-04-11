/**
 * @file PDH.c
 * @brief Interprets information from the Rev Power Distribution Hub (PDH).
 *
 * @author Carlos Giron <rdb7fq@virginia.edu>
 * @author Cole Luba <meq2fg@virginia.edu>
 * @author Anthony Vu <hsh6ff@email.virginia.edu>
 * @author Aedan Stewart <aedan@email.virginia.edu>
 * @note Derivative work based on 'PDP.c' authored by Diana Lin <xrc9wg@virginia.edu>.
 * @note Assisted by AI (GPT-5 & Gemini 3 Flash) for decoding logic and comments.
 * @version 1.1
 * @date 2026-02-09
 * @copyright Copyright (c) 2026 Mechatronics and Robotics Society
 */

#include <semaphore.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "pdh.h"
#include "can.h"
#include "utils.h"

/**
 * @internal
 * @brief Decode a PDH current-data CAN payload.
 *
 * Each payload contains up to 6 channels, each encoded as a 10-bit value.
 * A 2-bit gap exists between the third and fourth channels.
 *
 * @param payload      Raw 64-bit CAN payload.
 * @param out          Destination array for decoded channel values.
 * @param channelCount Number of channels to decode.
 */
static void decodePDHFrame(uint64_t payload,
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
        bitIndex += PDH_BITS_PER_CHANNEL + ((bitIndex == 20) ? PDH_CHANNEL_GAP_BITS : 0);
    }
}

float getChannelCurrentPDH(PDH *pdh, uint8_t channel)
{
    // TODO: Implement low-current channels decoding

    // Return 0.0f for current channels that do not exist
    if (channel >= 20)
        return 0.0f;

    return pdh->channelCurrents[channel] * PDH_HIGH_CURRENT_LSB_A;
}

/**
 * @brief Get the input voltage (in volts) of the PDH
 *
 * @param pdh   PDH structure.
 */
float getInputVoltagePDH(PDH *pdh)
{
    return pdh->totalVoltage;
}

// TODO: Replace the if-else logic in the CAN RX callback with a more efficient dispatch mechanism
/**
 * @internal
 * @brief Process an incoming CAN message including current information.
 *
 * @param pdh   PDH structure.
 * @param msg   CAN Rx header.
 * @param data  Pointer to a 64-bit payload.
 */
static void receiveCurrentPDH(PDH *pdh, twai_frame_t *msg, uint64_t *data)
{
    // Match PDH device ID (upper bits of the extended ID)
    if ((msg->header.id & 0xFFFFF00) != 0x8051800)
        return;

    uint32_t channelGroup = (msg->header.id & PDH_CHANNEL_GROUP_MASK);

    uint16_t *current_arr_ptr;

    if (channelGroup == (PDH_GROUP_BASE_0_TO_5 | pdh->identifier))
    {
        current_arr_ptr = &(pdh->channelCurrents[0]);
    }
    else if (channelGroup == (PDH_GROUP_BASE_6_TO_11 | pdh->identifier))
    {
        current_arr_ptr = &(pdh->channelCurrents[6]);
    }
    else if (channelGroup == (PDH_GROUP_BASE_12_TO_17 | pdh->identifier))
    {
        current_arr_ptr = &(pdh->channelCurrents[12]);
    }
    else if (channelGroup == (PDH_GROUP_BASE_18_TO_24 | pdh->identifier))
    {
        current_arr_ptr = &(pdh->channelCurrents[18]);
    }
    else
    {
        return;
    }

    decodePDHFrame(*data, current_arr_ptr, 6);
    // xSemaphoreGive(sem);
}

/**
 * @internal
 * @brief Process an incoming CAN message including voltage data.
 *
 * @param pdh   PDH structure.
 * @param msg   CAN Rx header.
 * @param data  Pointer to a 64-bit payload.
 */
static void receiveVoltagePDH(PDH *pdh, twai_frame_t *msg, uint64_t *data)
{
    // Match PDH device ID
    if ((msg->header.id) != (0x8051900 | pdh->identifier))
        return;

    pdh->totalVoltage = extractBits(*data, 0, 12) * PDH_VOLTAGE_RESOLUTION;
}

/**
 * @internal
 * @brief Process an incoming CAN message intended for the PDH.
 *
 * Extended ID format:
 *   0x80518XY
 *     └──┬┘└─ channel group | CAN ID
 *        └──── PDH device ID
 *
 * @param msg  CAN Rx header.
 * @param data Pointer to 64-bit payload.
 */
void receiveCANPDH(PDH *pdh, twai_frame_t *msg, uint64_t *data)
{
    receiveVoltagePDH(pdh, msg, data);
    receiveCurrentPDH(pdh, msg, data);
}

// Replace this with abstracted version in can.c that takes a callback argument
/**
 * @brief Handles CAN interrupts
 *
 * @param handle    TWAI node handle.
 * @param edata     TWAI "RX done" event data.
 * @param pdh       PDH structure.
 */
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

/**
 * @brief Setups CAN for the PDH.
 *
 * @param pdh   PDH structure.
 */
void canSetupPDH(PDH *pdh)
{
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TX_GPIO_NUM,            // TWAI TX GPIO pin
        .io_cfg.rx = RX_GPIO_NUM,            // TWAI RX GPIO pin
        .io_cfg.quanta_clk_out = -1,         // FIX: Disable clock out (prevents GPIO 0 conflict)
        .io_cfg.bus_off_indicator = -1,      // FIX: Disable bus-off indicator
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

/**
 * @brief Initalizes PDP structure.
 *
 * @param pdh           PDH structure.
 * @param identifier    CAN identifier.
 */
void PDHInit(PDH *pdh, int identifier)
{
    pdh->identifier = identifier;

    for (int i = 0; i < 25; i++)
    {
        pdh->channelCurrents[i] = 0;
    }
}

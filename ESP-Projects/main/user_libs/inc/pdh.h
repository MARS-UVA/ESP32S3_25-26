/**
 * @file PDH.h
 * @brief Defines the structure for the Rev Power Distribution Hub (PDH).
 *
 * @author Carlos Giron <rdb7fq@virginia.edu>
 * @author Anthony Vu <hsh6ff@virginia.edu>
 * @copyright Copyright (c) 2026 Mechatronics and Robotics Society
 * @note Derivative work based on 'PDH.h' authored by Diana Lin <xrc9wg@virginia.edu>.
 * @note Assisted by AI (GPT-5 & Gemini 3 Flash) for decoding logic and comments.
 * @version 1.1
 * @date 2026-02-09
 */

#pragma once

#include <stdint.h>
#include "can.h"

// CAN ID Definitions

// Mask for the lower nibble selecting which channel group is present
#define PDH_CHANNEL_GROUP_MASK 0xFF

// Values of the channel-group nibble
#define PDH_GROUP_BASE_0_TO_5 0x00
#define PDH_GROUP_BASE_6_TO_11 0x40
#define PDH_GROUP_BASE_12_TO_17 0x80
#define PDH_GROUP_BASE_18_TO_24 0xC0

// Bits allocated for each channel in the current data frames
#define PDH_BITS_PER_CHANNEL 10
#define PDH_CHANNEL_GAP_BITS 2

// High-current channel resolution (Amps per LSB)
#define PDH_HIGH_CURRENT_LSB_A 0.125

// Input voltage measurement resolution (Volts per LSB)
#define PDH_VOLTAGE_RESOLUTION 0.00781

/**
 * @brief Power Distribution Hub (PDH) driver state.
 */
typedef struct
{
    int identifier;
    uint16_t channelCurrents[25];
    double totalVoltage;
} PDH;

/**
 * @brief Initialize the PDH structure.
 *
 * @param pdh       PDH instance.
 * @param identifier Unique identifier for the PDH.
 */
void PDHInit(PDH *pdh, int identifier);

/**
 * @brief Get the input voltage (in Volts) for the PDH.
 *
 * @param pdh PDH instance.
 * @return Input voltage in Volts.
 */
float getInputVoltagePDH(PDH *pdh);

/**
 * @brief Setups CAN communication for the PDH, including registering the appropriate CAN RX handler.
 *
 * @param pdh   PDH structure.
 */
void canSetupPDH(PDH *pdh);

/**
 * @brief Get the current (in Amps) for a specific PDH channel.
 *
 * @param pdh     PDH instance.
 * @param channel Channel index (0–19).
 * @return Channel current in Amps, or 0.0f if unsupported.
 */
float getChannelCurrentPDH(PDH *pdh, uint8_t channelID);
void receiveCANPDH(PDH *pdh, twai_frame_t *msg, uint64_t *data);

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

#include <semaphore.h>
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "can.h"
#include "talonFX.h"
#include "talonSRX.h"
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

    // SemaphoreHandle_t sem_0_5;
    // SemaphoreHandle_t sem_6_11;
    // SemaphoreHandle_t sem_12_17;
    // SemaphoreHandle_t sem_18_24;
     
    // StaticSemaphore_t _buf_0_5;
    // StaticSemaphore_t _buf_6_11;
    // StaticSemaphore_t _buf_12_17;
    // StaticSemaphore_t _buf_18_24;
} PDH;

extern TalonFX *fxMotors[];
extern TalonSRX *srxMotors[];

// initiate PDP struct
void PDHInit(PDH *pdh, int identifier);

double getInputVoltagePDH(PDH *pdh);

// initialize CAN to interpret packets into the PDH struct
void canSetupPDH(PDH *pdh);

/**
 * @brief Get the current (in Amps) for a specific PDH channel.
 *
 * @param pdh     PDH instance.
 * @param channel Channel index (0–19).
 * @return Channel current in Amps, or 0.0f if unsupported.
 */
float getChannelCurrentPDH(PDH* pdh, uint8_t channelID);

void current_update_task(PDH* pdh);



#endif
#pragma once

#include "utils.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <semaphore.h>

// LIBRARY CONSTANTS
#define CAN_LOG "CAN_LOG"
#define RX_GPIO_NUM GPIO_NUM_1
#define TX_GPIO_NUM GPIO_NUM_2
#define ROBOT_BITRATE 1000000
#define TIMEOUT -1

// ENUMS & STRUCTS
typedef enum
{
    CAN_ID_SET_FX = 0x204b540,
    CAN_ID_SET_TARGET = 0x2043700,
    CAN_ID_PID = 0x2047c00,
    CAN_ID_CURRENT_LIMIT = 0x2047c00,
    CAN_ID_NEUTRAL_MODE = 0x2047c00,
    CAN_ID_SET_SRX = 0x2040200,
} can_id_t;

// INITS
extern twai_node_handle_t g_node_hdl;

// GENERAL CAN FUNCS
void canSetup(bool (*twai_rx_cb)(twai_node_handle_t, const twai_rx_done_event_data_t *, void *), void *rx_cb_data);
void sendEn();
void sendMsg(can_id_t msg_id, uint8_t d_id, uint8_t *data_buff, size_t len);

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
uint32_t extractBits(uint64_t data, uint8_t startBit, uint8_t bitLength);
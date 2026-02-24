/**
 * @file can.h
 * @brief CAN communication library for ESP32S3.
 * 
 * This library provides functions to set up CAN communication, send messages, and handle received messages using the ESP32's TWAI (CAN) controller.
 * 
 * @author Carlos Giron <rdb7fq@virginia.edu>
 * @author Anthony Vu <anthonyvu@email.virginia.edu>
 * @version 1.0
 * @date 2026-02-09
 * @copyright Copyright (c) 2026 Mechatronics and Robotics Society
 */

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

/**
 * @brief Type definition for a CAN frame handler function.
 * 
 * @param context   Pointer to user-defined context data that will be passed to the handler when a CAN frame is received.
 * @param frame     Pointer to the received TWAI frame containing the CAN message data.
 */
typedef void (*can_frame_handler_t)(
    void *context,
    const twai_frame_t *frame
);

/**
 * @brief Structure for CAN RX callback context, containing the handler function and its associated context data.
 */
typedef struct {
    can_frame_handler_t handler;
    void *context;
} can_rx_context_t;


// INITS
extern twai_node_handle_t g_node_hdl;

// GENERAL CAN FUNCS

/**
 * @brief Handles CAN interrupts by receiving the message and invoking the appropriate handler.
 *
 * @param[in] handle    TWAI node handle.
 * @param[in] edata     TWAI "RX done" event data.
 * @param[in] arg       Pointer to a can_rx_context_t struct containing the handler and context for processing the received frame.
 * @return          false (indicates that the interrupt has been handled)
 */
bool twaiRxCallback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *arg);

/**
 * @brief Set up CAN communication by configuring the TWAI node and registering the RX callback.
 * 
 * @param[in] twai_rx_cb   Pointer to the function that will handle received CAN frames.
 * @param[in] rx_cb_data   Pointer to the context data that will be passed to the RX callback function when a CAN frame is received.
 */
void canSetup(can_rx_context_t *rx_ctx);

// TODO: Write documentation for the following functions

/**
 * 
 */
void sendEn();

/**
 * 
 */
void sendMsg(can_id_t msg_id, uint8_t d_id, uint8_t *data_buff, size_t len);

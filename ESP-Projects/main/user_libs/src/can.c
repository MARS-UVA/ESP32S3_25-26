/**
 * @file    can.c
 * @brief   CAN communication functions for the robot.
 * 
 * @author      Carlos Giron <rdb7fq@virginia.edu>
 * @author      Anthony Vu <anthonyvu@email.virginia.edu>
 * @note        Assisted by AI (GPT-5) for TWAI abstraction.
 * @version     1.0
 * @date        2026-02-09
 * @copyright   Copyright (c) 2026 Mechatronics and Robotics Society
 */

#include "can.h"

// Global TWAI node handle
twai_node_handle_t g_node_hdl = NULL;

bool twaiRxCallback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *arg)
{
    //  Cast the argument to our context struct
    can_rx_context_t *ctx = (can_rx_context_t *)arg;

    // Receive the CAN frame from the TWAI node
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff)
    };

    // If a frame was successfully received, call the handler with the context and the received frame
    if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK)
    {
        ctx->handler(ctx->context, &rx_frame);
    }

    // Return false to indicate that the interrupt has been handled
    return false;
}

void canSetup(can_rx_context_t *rx_ctx)
{
    // Configure the TWAI node with the specified GPIO pins and bitrate
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TX_GPIO_NUM,            // TWAI TX GPIO pin
        .io_cfg.rx = RX_GPIO_NUM,            // TWAI RX GPIO pin
        .bit_timing.bitrate = ROBOT_BITRATE, // 1Mbps bitrate
        .tx_queue_depth = 32,                // Transmit queue depth set to 32
    };

    // Set up the TWAI event callbacks, using the provided RX callback function and context
    twai_event_callbacks_t can_cbs = {
        .on_rx_done = twaiRxCallback,
    };

    // Create a new TWAI node with the specified configuration
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &g_node_hdl));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(g_node_hdl, &can_cbs, rx_ctx));
    ESP_ERROR_CHECK(twai_node_enable(g_node_hdl));
}

// ENABLE CAN FRAME
uint8_t en_buff[] = {0x01, 0x00}; // Enable message data buffer
twai_frame_t en_msg = {
    .header.id = 0x401bf, // Message ID
    .header.ide = true,   // Use 29-bit extended ID format
    .buffer = en_buff,    // Pointer to data to transmit
    .buffer_len = 2,      // Length of data to transmit
};

// GENERAL CAN FUNCS
void sendEn()
{
    ESP_ERROR_CHECK(twai_node_transmit(g_node_hdl, &en_msg, TIMEOUT)); // Timeout = 0: returns immediately if queue is full
    vTaskDelay(1);                                                     // without delay watchdog timers are triggered for tasks
}

void sendMsg(can_id_t msg_id, uint8_t d_id, uint8_t *data_buff, size_t len)
{
    twai_frame_t msg = {
        .header.id = msg_id | d_id,
        .header.ide = true,
        .buffer = data_buff,
        .buffer_len = len,
    };
    sendEn();
    ESP_ERROR_CHECK(twai_node_transmit(g_node_hdl, &msg, TIMEOUT));
}

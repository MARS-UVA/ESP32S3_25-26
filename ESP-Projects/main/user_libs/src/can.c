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

void canSetup()
{
    // Configure the TWAI node with the specified GPIO pins and bitrate
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TX_GPIO_NUM,            // TWAI TX GPIO pin
        .io_cfg.rx = RX_GPIO_NUM,            // TWAI RX GPIO pin
        .bit_timing.bitrate = ROBOT_BITRATE, // 1Mbps bitrate
        .tx_queue_depth = 32,                // Transmit queue depth set to 32
    };


    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &g_node_hdl));
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

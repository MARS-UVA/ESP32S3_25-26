#include "can2.h"

twai_node_handle_t node_hdl = NULL;
twai_onchip_node_config_t node_config = {
    .io_cfg.tx = TX_GPIO_NUM,            // TWAI TX GPIO pin
    .io_cfg.rx = RX_GPIO_NUM,            // TWAI RX GPIO pin
    .bit_timing.bitrate = ROBOT_BITRATE, // 1Mbps bitrate
    .tx_queue_depth = 16,                // Transmit queue depth set to 16
};

// buffers
uint8_t en_buff[2] = {0x01, 0x00};
uint8_t prompt_buff[6] = {0x00, 0x00, 0x00, 0x00, 0x20, 0x00};
uint8_t send_buff[8] = {0};

// twai frames
twai_frame_t en_msg = {
    .header.id = 0x401bf, // Message ID
    .header.ide = true,   // Use 29-bit extended ID format
    .buffer = en_buff,    // Pointer to data to transmit
    .buffer_len = 2,      // Length of data to transmit
};
twai_frame_t setFX_msg = {
    .header.id = 0x204b540,
    .header.ide = true,
    .buffer = send_buff,
    .buffer_len = 8,
};
twai_frame_t setTarget_msg = {
    .header.id = 0x2043700,
    .header.ide = true,
    .buffer = send_buff,
    .buffer_len = 8,
};
twai_frame_t PID_msg = {
    .header.id = 0x2047c00,
    .header.ide = true,
    .buffer = send_buff,
    .buffer_len = 8,
};
twai_frame_t currentLimit_msg = {
    .header.id = 0x2047c00,
    .header.ide = true,
    .buffer = send_buff,
    .buffer_len = 8,
};

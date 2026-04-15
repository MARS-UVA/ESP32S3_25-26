#include "can.h"

// inits
twai_node_handle_t g_node_hdl = NULL;

void canSetup(bool (*twai_rx_cb)(twai_node_handle_t, const twai_rx_done_event_data_t *, void *), void *rx_cb_data)
{
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TX_GPIO_NUM,            // TWAI TX GPIO pin
        .io_cfg.rx = RX_GPIO_NUM,            // TWAI RX GPIO pin
        .bit_timing.bitrate = ROBOT_BITRATE, // 1Mbps bitrate
        .tx_queue_depth = 32,                // Transmit queue depth set to 32
    };
    twai_event_callbacks_t can_cbs = {
        .on_rx_done = twai_rx_cb,
    };
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &g_node_hdl));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(g_node_hdl, &can_cbs, rx_cb_data));
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
    ESP_ERROR_CHECK(twai_node_transmit(g_node_hdl, &msg, TIMEOUT));
    ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(g_node_hdl, TIMEOUT)); // DO NOT REMOVE OR I WILL SLIME YOU OUT
}

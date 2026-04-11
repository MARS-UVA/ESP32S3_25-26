/**
 * @file talonFX.c
 * @brief Implements functions for controlling the Talon FX motor controller.
 * 
 * @author Diana Lin
 * @author Carlos Giron
 * @author Anthony Vu
 * 
 * @copyright Copyright (c) 2026 Mechatronics and Robotics Society
 * @version 1.0
 * @date 2026-02-09
 */

#include "talonFX.h"

#include <stdbool.h>

#include "can.h"

/**
 * @brief Initializes a TalonFX structure with default values.
 * 
 * @param n_id  CAN ID for the Talon FX (0-63).
 * @param c_id  Channel number for current monitoring (0-23).
 * @return Initialized TalonFX structure.
 */
TalonFX talonFXInit(uint8_t n_id, uint8_t c_id)
{
    return (TalonFX){
        .id = n_id,
        .currentLimit = 0.0f,
        .kP = 0.0f,
        .kI = 0.0f,
        .kD = 0.0f,
        .breakMode = false,
        .channel = c_id,
        .current = 0.0f,
        .temperature = 0,
    };
}

// FX CAN FUNCS
void setFX(TalonFX *fx, float speed) // set duty cycle for speed
{
    uint8_t buff[] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    short valueInt = (short)(speed * 1024);
    if (valueInt < 0)
    {
        valueInt = 0xfff - (-1 * valueInt);
    }
    writeToBuffInd(buff, (uint8_t *)&valueInt, 6, 2);
    sendMsg(CAN_ID_SET_FX, fx->id, buff, 8);
}

void setTargetFX(TalonFX *fx, int velocity) // setting PID velocity
{
    if (velocity >= 0)
    {
        velocity *= 16;
    }
    else
    {
        velocity = 0x40000 - (-16 * velocity);
    }
    // Get feedforward value
    float feedforward = 0.1;
    int feedforwardInt = feedforward * 100;
    if (feedforward < 0)
        feedforwardInt = (~(feedforwardInt * -1)) + 1;
    uint8_t buff[] = {0, 1, velocity & 0xff, (velocity >> 8) & 0xff, velocity >> 16 & 0xff, 0, feedforwardInt & 0xff, (feedforwardInt >> 8) & 0xff};
    sendMsg(CAN_ID_SET_TARGET, fx->id, buff, 8);
}

/**
 * 
 */
void receiveCANTalonFX(TalonFX *fx, twai_frame_t *rx_frame, uint64_t *recv_buff)
{
    if (rx_frame->header.id != (0x2044740 | fx->id)) {
        return;
    } 
// 0x2044761 | fx->id
    fx->temperature = extractBits(*recv_buff, 40, 8);
}

/**
 * @brief 
 *
 * @param handle    TWAI node handle.
 * @param edata     TWAI "RX done" event data.
 * @param fx        TalonFX structure.
 */
bool talonfx_twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff)
    };

    if (ESP_OK != twai_node_receive_from_isr(handle, &rx_frame)) {
        return false;
    }

    TalonFXRegistry *reg = (TalonFXRegistry *)user_ctx;
    for (size_t i = 0; i < reg->count; i++) {
        receiveCANTalonFX(reg->motors[i], &rx_frame, (uint64_t *)recv_buff);
    }
    //receiveCANPDH((PDH *)pdh, &rx_frame, (uint64_t *)&recv_buff);
    return false;
}

/**
 * @brief Setups CAN for the Talon FX.
 *
 * @param fx    TalonFX structure.
 */
void canSetupTalonFX(TalonFX **motors, size_t count)
{
    static bool can_initialized = false;
    static TalonFXRegistry registry;
    if (!can_initialized) {
        registry.motors = motors;
        registry.count = count;
    
        twai_onchip_node_config_t node_config = {
            .io_cfg.tx = TX_GPIO_NUM,            // TWAI TX GPIO pin
            .io_cfg.rx = RX_GPIO_NUM,            // TWAI RX GPIO pin
            .bit_timing.bitrate = ROBOT_BITRATE, // 1Mbps bitrate
            .tx_queue_depth = 32,                // Transmit queue depth set to 32
        };

        twai_event_callbacks_t can_cbs = {
            .on_rx_done = talonfx_twai_rx_cb,
        };
    
        //ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &g_node_hdl));
        ESP_ERROR_CHECK(twai_node_register_event_callbacks(g_node_hdl, &can_cbs, &registry));
        //ESP_ERROR_CHECK(twai_node_enable(g_node_hdl));
        can_initialized = true;
    }
}

float getTemperatureTalonFX(TalonFX *fx)
{
    return fx->temperature;
}

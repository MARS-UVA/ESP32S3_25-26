#include "utils.h"
#include "can2.h"
#include "pdp.h"

// inits
twai_node_handle_t g_node_hdl = NULL;
void canSetup()
{
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = TX_GPIO_NUM,            // TWAI TX GPIO pin
        .io_cfg.rx = RX_GPIO_NUM,            // TWAI RX GPIO pin
        .bit_timing.bitrate = ROBOT_BITRATE, // 1Mbps bitrate
        .tx_queue_depth = 32,                // Transmit queue depth set to 32
    };
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &g_node_hdl));
    //ESP_ERROR_CHECK(twai_node_register_event_callbacks(g_node_hdl, &user_cbs, NULL));
    ESP_ERROR_CHECK(twai_node_enable(g_node_hdl));
}

TalonFX talonFXInit(uint8_t n_id)
{
    return (TalonFX){
        .id = n_id,
        .currentLimit = 0.0f,
        .kP = 0.0f,
        .kI = 0.0f,
        .kD = 0.0f,
        .breakMode = false,
    };
}

TalonSRX talonSRXInit(uint8_t n_id, bool inv)
{
    return (TalonSRX){
        .id = n_id,
        .inverted = inv,
    };
}

// ENABLE CAN FRAME
uint8_t en_buff[] = {0x01, 0x00};       // Enable message data buffer
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

// FX CAN FUNCS
void setFX(TalonFX *fx, float speed)
{
    uint8_t buff[] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

    short valueInt = (short)(speed * 1024);
    if (valueInt < 0)
    {
        valueInt = 0xfff - (-1 * valueInt);
    }
    // writeToBuffInd(buff, (uint8_t *)&valueInt, 6, 2);
    sendMsg(CAN_ID_SET_FX, fx->id, buff, 8);
}

void setTargetFX(TalonFX *fx, int velocity)
{
    // Get velocity value (3 bytes)
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

// SRX CAN FUNCS
void setInvertedSRX(TalonSRX *srx, bool invert)
{
    srx->inverted = invert;
}

// set the output magnitude of a Talon SRX
void setSRX(TalonSRX *srx, double value)
{
    // set the direction of a Talon SRX to be inverted
    int valueInt = (int)(value * 1023);
    uint8_t buff[] = {(valueInt >> 16) & 255, (valueInt >> 8) & 255, valueInt & 255, 0, 0, 0, 0x0b, srx->inverted ? 0x40 : 0x00};
    sendMsg(CAN_ID_SET_SRX, srx->id, buff, 8);
}
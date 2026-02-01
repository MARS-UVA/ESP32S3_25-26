#ifndef CAN2_H
#define CAN2_H

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

typedef struct
{
    uint8_t id;
    float currentLimit;
    float kP;
    float kI;
    float kD;
    bool breakMode;
    uint8_t channel;
    float current;
} TalonFX;

typedef struct
{
    uint8_t id;
    bool inverted;
    uint8_t channel;
    float current;
} TalonSRX;

// INITS
extern twai_node_handle_t g_node_hdl;
TalonFX talonFXInit(uint8_t n_id, uint8_t c_id);
TalonSRX talonSRXInit(uint8_t n_id, uint8_t c_id, bool inv);

// GENERAL CAN FUNCS
void canSetup(bool (*twai_rx_cb)(twai_node_handle_t, const twai_rx_done_event_data_t *, void *), void* rx_cb_data);
void sendEn();
void sendMsg(can_id_t msg_id, uint8_t d_id, uint8_t *data_buff, size_t len);

// FX FUNCS
void setFX(TalonFX *fx, float speed);
void setTargetFX(TalonFX *fx, int velocity);

// SRX FUNCS
void setSRX(TalonSRX *srx, double value);

#endif
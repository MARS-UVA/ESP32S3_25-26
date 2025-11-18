#ifndef CAN2_H
#define CAN2_H

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include <stdint.h>
#include <stdbool.h>

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
} TalonFX;

typedef struct
{
    uint8_t id;
    bool inverted;
} TalonSRX;

// INITS
TalonFX talonFXInit(uint8_t id);
TalonSRX TalonSRXInit(uint8_t id, bool inv);

// FX FUNCS
void setFX(twai_node_handle_t *node_hdl, TalonFX *fx, float speed);
void setTargetFX(twai_node_handle_t *node_hdl, TalonFX *fx, int velocity);

// SRX FUNCS
void setSRX(twai_node_handle_t *node_hdl, TalonSRX *srx, double value);

// CAN FUNCS
void canSetup(twai_node_handle_t *node_hdl);
void sendEn(twai_node_handle_t *node_hdl);
void sendMsg(twai_node_handle_t *node_hdl, can_id_t msg_id, uint8_t d_id, uint8_t *data_buff, uint8_t len);

#endif
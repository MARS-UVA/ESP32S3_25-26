#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include <stdint.h>
#include <stdbool.h>

// LIBRARY CONSTANTS
#define CAN_LOG "CAN_LOG"
#define RX_GPIO_NUM GPIO_NUM_1
#define TX_GPIO_NUM GPIO_NUM_2

// STRUCTS
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

// FUNCTIONS
TalonFX talonFXInit(uint8_t id);
TalonSRX TalonSRXInit(uint8_t id);
void talonPercentOut(int16_t speed);
void setPIDValues();
void setTargetVelocity(int velocity);
void canSetup();
void canStop();
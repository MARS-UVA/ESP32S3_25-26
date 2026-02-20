#include "can2.h"

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

// Inits
TalonFX talonFXInit(uint8_t n_id, uint8_t c_id);

// FX FUNCS
void setFX(TalonFX *fx, float speed);
void setTargetFX(TalonFX *fx, int velocity);
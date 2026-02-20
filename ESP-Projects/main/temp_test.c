#include "can2.h"
#include "pdp.h"
#include <freertos/FreeRTOS.h>
#include <stdio.h>

TalonFX motor;

void motor_switch_directions()
{
    motor = talonFXInit(21, 7);
    while (1)
    {
        for(int i = 0; i < 1000; i ++){
        setFX(&motor, 0.5);
        }
        for(int i = 0; i < 1000; i ++){
        setFX(&motor, -0.5);
        }
    }
}

void app_main()
{
    xTaskCreate(motor_switch_directions, "mtr", 4096, NULL, 8, NULL);
}
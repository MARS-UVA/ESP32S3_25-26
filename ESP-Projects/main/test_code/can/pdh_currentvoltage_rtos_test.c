#include "can2.h"
#include "pdh.h"
#include "utils.h"
#include <freertos/FreeRTOS.h>

#include <stdio.h>

TalonFX motor;
PDH pdh;
float current;
float voltage;

void motor_task()
{
    motor = talonFXInit(59, 7);
    while (1)
    {
        setFX(&motor, 0.5);
    }
}

void current_sensing()
{
    
}
void app_main()
{
    PDHInit(&pdh, 62);
    canSetupPDH(&pdh);
    xTaskCreate(current_sensing, "curr", 4096, NULL, 9, NULL);
    xTaskCreate(motor_task, "mtr", 4096, NULL, 8, NULL);
}
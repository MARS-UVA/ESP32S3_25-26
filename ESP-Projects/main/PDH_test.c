#include "can2.h"
#include "pdp.h"
#include <freertos/FreeRTOS.h>

#include <stdio.h>

TalonFX motor;
PDP pdp;

void motor_task()
{
    motor = talonFXInit(1, 7);
    while (1)
    {
        setFX(&motor, 0.5);
    }
}

void current_sensing()
{
    float cur;
    PDPInit(&pdp, 10);
    canSetupPDP(&pdp);
    while (1)
    {
        requestCurrentReadingsPDP(&pdp, 10);
        cur = getChannelCurrentPDP(&pdp, 7);
        printf("Current at channel %d is:\t %.3f\n", pdp.identifier, cur);
    }
}
void app_main()
{
    xTaskCreate(current_sensing, "curr", 4096, NULL, 9, NULL);
    xTaskCreate(motor_task, "mtr", 4096, NULL, 8, NULL);
}
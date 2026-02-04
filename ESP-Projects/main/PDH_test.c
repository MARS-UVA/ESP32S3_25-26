#include "can2.h"
#include "pdp.h"
#include <freertos/FreeRTOS.h>

#include <stdio.h>

TalonFX motor;
PDP pdp;

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
    int pdp_id = 10;
    int ch_id = 7;
    int waittime_ms = 100;
    float cur;

    PDPInit(&pdp, pdp_id);
    canSetupPDP(&pdp);

    while (1)
    {
        requestCurrentReadingsPDP(&pdp);
        bool r = awaitCurrentReadingsPDP(&pdp, waittime_ms);
        if (!r)
            printf("Semaphores not acquired\n");

        cur = getChannelCurrentPDP(&pdp, ch_id);
        printf("Current at channel %d is:\t %.3f\n", ch_id, cur);
    }
}
void app_main()
{
    xTaskCreate(current_sensing, "curr", 4096, NULL, 9, NULL);
    xTaskCreate(motor_task, "mtr", 4096, NULL, 8, NULL);
}
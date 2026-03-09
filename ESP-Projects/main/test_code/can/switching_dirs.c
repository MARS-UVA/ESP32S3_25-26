#include "utils.h"
#include "can.h"
#include "talonFX.h"
#include "pdh.h"

#define WAIT_TIME 50
#define MOTOR_SPEED 0.2

void rtos_talon()
{
    TalonFX motor = talonFXInit(33, 3);
    printf("Motor will run and switch directions in 1s\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1)
    {
        for (int i = 0; i < WAIT_TIME; i++)
        {
            setFX(&motor, MOTOR_SPEED);
        }
        for (int i = 0; i < WAIT_TIME; i++)
        {
            setFX(&motor, -MOTOR_SPEED);
        }
    }
}

void app_main()
{
    PDH pdh;
    PDHInit(&pdh, 62);
    canSetupPDH(&pdh);
    xTaskCreate(rtos_talon, "mtr", 4096, NULL, 9, NULL);
}
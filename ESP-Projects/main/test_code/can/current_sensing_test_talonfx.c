/**
 * @file current_sensing_test_talonfx.c
 * @brief Functionality test for Talon FX current sensing via CAN communication with the PDH.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "talonFX.h"
#include "pdh.h"

TalonFX motor;
PDH pdh;

void motor_task()
{
    while (1)
    {
        setFX(&motor, 0.1);
        vTaskDelay(20);
    }
}

void current_task()
{
    while (1)
    {
        printf("Current at channel 3:\t%f\n", talonFXGetCurrent(&motor));
        vTaskDelay(30);
    }
}

void app_main()
{
    canSetupPDH(&pdh);
    PDHInit(&pdh, 62);
    motor = talonFXInit(0, 3, &pdh);

    xTaskCreate(motor_task, "mtr", 4096, NULL, 8, NULL);
    xTaskCreate(current_task, "cur_fx", 4096, NULL, 7, NULL);
}

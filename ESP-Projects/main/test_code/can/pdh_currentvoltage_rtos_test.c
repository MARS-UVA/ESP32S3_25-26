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
    while (1)
    {
        // setFX(&motor, 0.4);
        vTaskDelay(1);
    }
}

void current_sensing()
{
    float current;
    while(1) {
        current = getChannelCurrentPDH(&pdh, 3);
        voltage = getInputVoltagePDH(&pdh);
        printf("voltage: %.03f\n", voltage);
        printf("current: %.03f\n", current);
        vTaskDelay(10);
    }
    
}

void app_main()
{
    PDHInit(&pdh, 62);
    canSetupPDH(&pdh);
    motor = talonFXInit(33, 3);
    xTaskCreate(current_sensing, "curr", 4096, NULL, 9, NULL);
    xTaskCreate(motor_task, "mtr", 4096, NULL, 8, NULL);
}
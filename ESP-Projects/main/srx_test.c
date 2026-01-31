#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <uart.h>
#include "can2.h"

TalonSRX motor;
TalonSRX motor2;

void runSRX() 
{
    while (1) 
    {
        setSRX(&motor, 0.25);
        setSRC(&motor2, 0.25);
    }
}

void app_main()
{
    canSetup(g_node_hdl);
    initalizePots();
    motor = talonSRXInit(12, 0, false);
    motor2 = talonSRXInit(12, 0, true);

    xTaskCreate(runSRX, "runSRX", 4096, NULL, 8, NULL);
    xTaskCreate(readPot, "readPot", 4096, NULL, 7, NULL);
}
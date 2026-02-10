#include "can2.h"
#include <freertos/FreeRTOS.h>

#include <stdio.h>

void app_main()
{
    TalonFX motor = talonFXInit(33, 0);
    canSetup();

    for (int i = 0; i < 60; i++)
    {
        printf("%d\n", i);
        setTargetFX(&motor, 100);
    }
}
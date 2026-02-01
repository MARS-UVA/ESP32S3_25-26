#include "can2.h"
#include "pdp.h"
#include <freertos/FreeRTOS.h>

#include <stdio.h>

void app_main()
{
    TalonFX motor = talonFXInit(36, 0);
    PDP pdp;
    uint32_t pdp_id = 0;
    canSetupPDP(&pdp);
    

    for (int i = 0; i < 60; i++)
    {
        printf("%d\n", i);
        setTargetFX(&motor, 100);
    }
}
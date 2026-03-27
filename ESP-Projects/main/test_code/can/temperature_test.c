#include "can.h"
#include "talonFX.h"
#include "pdh.h"

void temperature_task()
{
    PDH pdh;
    PDHInit(&pdh, 62);
    TalonFX motor = talonFXInit(33, 2);
    canSetupTalonFX(&motor);
    while (1)
    {
        printf("Temperature:\t%d\n", motor.temperature);
        vTaskDelay(100);
    }
}

void app_main()
{
    // xTaskCreate(motor_task, "mtr", 4096, NULL, 8, NULL);
    xTaskCreate(temperature_task, "cur", 4096, NULL, 8, NULL);
}

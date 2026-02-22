#include "can.h"
#include "talonFX.h"

void temperature_task()
{
    TalonFX motor = talonFXInit(33, 3);
    while (1)
    {
        printf("Temperature:\t%d\n", motor.temperature);
        vTaskDelay(1000);
    }
}
void app_main()
{
    canSetupTalonFX(&motor);
    // xTaskCreate(motor_task, "mtr", 4096, NULL, 8, NULL);
    xTaskCreate(temperature_task, "cur", 4096, NULL, 8, NULL);
}
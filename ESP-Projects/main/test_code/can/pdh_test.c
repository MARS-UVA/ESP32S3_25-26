#include "utils.h"
#include "can.h"
#include "talonFX.h"
#include "pdh.h"

TalonFX motor;
PDH pdh;

void motor_task()
{
    motor = talonFXInit(0, 3, NULL);
    while (1)
    {
        setFX(&motor, 0.1);
        vTaskDelay(20);
    }
}

void current_task()
{
    float current;
    double voltage;

    PDHInit(&pdh, 62);
    printf("Can Setup\n");
    printf("Can Setup Done\n");

    for (;;)
    {
        current = getChannelCurrentPDH(&pdh, 3);
        voltage = getInputVoltagePDH(&pdh);
        printf("Current at channel 3:\t%f\n", current);
        printf("Voltage reading is %.02lf\n", voltage);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main()
{
    canSetupPDH(&pdh);
    xTaskCreate(motor_task, "mtr", 4096, NULL, 8, NULL);
    xTaskCreate(current_task, "cur", 4096, NULL, 7, NULL);
}

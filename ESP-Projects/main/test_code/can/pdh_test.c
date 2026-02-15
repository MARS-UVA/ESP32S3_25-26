#include "utils.h"
#include "can2.h"
#include "pdh.h"


// TalonFX motor;
// void motor_task()
// {
//     motor = talonFXInit(33, 1);
//     while (1)
//     {
//         setFX(&motor, 0.1);
//         vTaskDelay(pdMS_TO_TICKS(250));
//         setFX(&motor, -0.1);
//         vTaskDelay(pdMS_TO_TICKS(250));
//     }
// }

void app_main()
{
    PDH pdh;
    PDHInit(&pdh, 62);

    // float current = -1;
    double voltage;

    printf("Can Setup\n");
    canSetupPDH(&pdh);
    printf("Can Setup Done\n");

    // xTaskCreate(motor_task, "mtr", 4096, NULL, 8, NULL);

    for (;;)
    {
        // for (int i = 1; i <= 1; i++)
        // {
        //     current = getChannelCurrentPDH(&pdh, i);
        //     printf("Current at channel %d:\t%f\n", i, current);
        // }
        
        voltage = getInputVoltagePDH(&pdh);
        printf("Voltage reading is %.02lf\n", voltage);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

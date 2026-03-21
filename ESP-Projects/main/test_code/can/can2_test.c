#include "utils.h"
#include "can.h"
#include "talonFX.h"
#include "pdh.h"
#include <freertos/FreeRTOS.h>

/*
void app_main()
{
    TalonFX fx = talonFXInit(27);
    //TalonSRX srx = talonSRXInit(4, false);
    printf("Can Setup\n");
    canSetup();
    // ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &user_cbs, NULL));
    printf("Can Setup Done\n");
    while (1)
    {
        setTargetFX(&fx, 100);
        // setSRX(&srx, .5);
    }
}
*/

void app_main() {
    PDH pdh;
    PDHInit(&pdh, 62);
    TalonFX motor = talonFXInit(33, 2, &pdh);
    canSetupPDH(&pdh);
    printf("setup done!\n");

    while(1) {
        printf("running...\n");
        setTargetFX(&motor, 100);
        vTaskDelay(1);
    }
}
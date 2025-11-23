#include "utils.h"
#include "can2.h"
#include "freertos/FreeRTOS.h"
#include "pdp.h"
#include <stdio.h>
#include "esp_log.h"

void app_main(void) {

    canSetup();
    PDPInit(10);
    TalonFX motor = talonFXInit(27);
    float current;

    while(1) {
        setTargetFX(&motor, 20);
     //   requestCurrentReadingsPDP();
        current = 0.1f;
        printf("\nCurrent Reading from PDP Channel 10: %.3f Amps", current);
        vTaskDelay(1);
    }
}
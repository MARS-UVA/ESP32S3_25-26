#include "utils.h"
#include "can.h"
#include "talonFX.h"

void rtos_talon() {
    TalonFX motor = TalonFXInit(33, 7);
    printf("Motor will run and switch directions in 1s\n");
    vTaskDelay(PD_MS_TO_TICKS(1000))
    while(1) {
        for (int i=0; i<1000; i++) {
            setFX(&motor, 0.5);
        }
        for (int i=0; i<1000; i++) {
            setFX(&motor, -0.5);
        }
    }
}

void app_main() {
    xTaskCreate(rtos_talon, "mtr", NULL, 9, NULL);
}
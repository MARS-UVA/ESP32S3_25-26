#include "LO_CAN.h"



void app_main(){

    canSetUp();
    
    while (1) {
        talonPercentOut(112); // %50 of motor output
        vTaskDelay(5);
    }

}
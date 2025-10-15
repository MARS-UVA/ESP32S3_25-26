//#include "LO_CAN.h"
#include "CL_CAN.h"



void app_main(){

    canSetup();
    setPIDValues();
    
    while (1) {
        talonPercentOut(112); // %50 of motor output
        vTaskDelay(5); // watchdog -> way to know if microcontroller still works fine by feeding it code
            // if we don't feed the watchdog for a certain amount of time, the code crashes
    }

}
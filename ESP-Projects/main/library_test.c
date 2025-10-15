//#include "LO_CAN.h"
#include "CL_CAN.h"



void app_main(){

    canSetup();
    setPIDValues();
    
    while (1) {
        setTargetVelocity(512); // Set target velocity to 100%
        vTaskDelay(5);
    }

}
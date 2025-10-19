#include "LO_CAN.h"


void app_main(){

    canSetUp();
    supplyCurrentLimit(.5);
    talonPercentOut(20);

}
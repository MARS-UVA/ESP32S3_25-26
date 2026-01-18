#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <uart.h>
#include "control_startup.c"
#include "adc.h"



void app_main()
{
    PotInit(ADC_CHANNEL_1);
    ledSetup();
    
    xTaskCreate(readPot, "readPot", 4096, NULL, 7, NULL);
    //xTaskCreate(, "", 4096, NULL, 6, NULL);
    //
}
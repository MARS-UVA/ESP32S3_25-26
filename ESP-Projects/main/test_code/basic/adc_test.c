#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <uart.h>
#include "control_startup.c"
#include "adc.h"

void app_main()
{
    PotInit(ADC_UNIT_1, ADC_CHANNEL_1);
    printf("Going into readPot...\n");
    xTaskCreate(readPot, "readPot", 4096, NULL, 7, NULL);
}
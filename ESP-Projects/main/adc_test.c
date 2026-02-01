#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <uart.h>
#include "adc.h"

void app_main()
{
    potInit(ADC_UNIT_1, ADC_CHANNEL_4, 0, 0);
    printf("Going into readPot...\n");
    xTaskCreate(readPot, "readPot", 4096, NULL, 7, NULL);
}
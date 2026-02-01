#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <uart.h>
#include "control_startup.c"
#include "can2.h"
#include "./user_libs/ADC/adc.h"

TaskHandle_t readTaskHandle = NULL;

void app_main()
{
    printf("Starting Main Task...\n");
    PotInit(ADC_UNIT_1, ADC_CHANNEL_3);
    // PotInit(ADC_UNIT_1, ADC_CHANNEL_4);
    // initializeTalons();
    // UART_setup();
    // canSetup(g_node_hdl);
    // SerialPacket packet = {0, 0x0, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F};
    // packet.invalid = 1;
    // for (;;)
    // {
    //     // UART_read(&packet);
    //     directControl(packet);
    // }
    return;
}

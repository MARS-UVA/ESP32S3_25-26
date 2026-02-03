#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <uart.h>
#include "control_startup.c"
#include "can2.h"
#include "user_libs/ADC/adc2.h"

TaskHandle_t readTaskHandle = NULL;

extern Pot leftActuatorPot;
extern Pot rightActuatorPot;

void readPots()
{
    while (1)
    {
        readPot(&leftActuatorPot);
        readPot(&rightActuatorPot);
    }
}

void app_main()
{
    initializeTalons();
    // UART_setup();
    canSetup(g_node_hdl);
    SerialPacket packet = {0, 0x0, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F};
    packet.invalid = 1;

    xTaskCreate(readPots, "readPot", 4096, NULL, 9, NULL);

    for (;;)
    {
        // UART_read(&packet);
        directControl(packet);
    }

    return;
}

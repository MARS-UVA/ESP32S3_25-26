#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <uart.h>
#include "control_startup.h"
#include "can.h"

TaskHandle_t readTaskHandle = NULL;

void app_main()
{
    initializeTalons();
    UART_setup();
    canSetup(g_node_hdl);
    ControlPacket_OneRobot packet = {0, 0x0, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F};
    packet.invalid = 1;
    for (;;)
    {
        UART_read(&packet);
        directControl(packet);
    }
    return;
}
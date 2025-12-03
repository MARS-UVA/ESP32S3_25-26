#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <uart.h>
#include "control_startup.c"
#include "can2.h"

void app_main()
{
    initializeTalons();
    UART_setup();
    canSetup(g_node_hdl);

    xTaskCreate(UART_rx_task, "uart_rx", 4096, NULL, 7, NULL);
    xTaskCreate(UART_can_task, "uart_can", 4096, NULL, 8, NULL);
}
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <uart.h>
#include "control_startup.h"
#include "pdp.h"
#include "wifi.h"

TaskHandle_t debug_task_handle = NULL;

void debug_task()
{
    while (1)
    {
        // ESP_LOGI("DEBUG", "Printing");
        UART_write(NULL);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main()
{
    // initializeTalons();
    UART_setup();
    // canSetup();
    // PDPInit(62);
    //  setupWifi();
    //  vTaskDelay(50);
    //  print_IP();

    // xTaskCreate((void *)(udp_receive_task), "udp_receive", 4096, NULL, 7, NULL);
    // xTaskCreate((void *)(UART_rx_task), "uart_rx", 4096, NULL, 7, NULL);
    // xTaskCreate((void *)(UART_can_task), "uart_can", 4096, NULL, 8, NULL);
    // xTaskCreate((void *)(current_update_task), "current_update", 4096, NULL, 8, NULL);
    xTaskCreate(UART_tx_task, "uart_tx", 4096, NULL, 9, NULL);

    // xTaskCreate(debug_task, "debug", 4096, NULL, 9, NULL);
}

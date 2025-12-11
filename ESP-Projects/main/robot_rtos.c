#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <uart.h>
#include "control_startup.h"
#include "can2.h"
#include "pdp.h"

TaskHandle_t debug_task_handle = NULL;

void debug_task()
{
    while (1)
    {
        ESP_LOGI("DEBUG", "Printing");
        vTaskDelay(10);
    }
}

void app_main()
{
    initializeTalons();
    UART_setup();
    canSetup();
    PDPInit(62);

    xTaskCreate(UART_rx_task, "uart_rx", 4096, NULL, 7, NULL);
    xTaskCreate(UART_can_task, "uart_can", 4096, NULL, 8, NULL);
    xTaskCreate(UART_tx_task, "uart_tx", 4096, NULL, 5, NULL);
    xTaskCreate(current_update_task, "current_update", 4096, NULL, 6, NULL);
    // xTaskCreate(debug_task, "debug", 4096, NULL, 9, NULL);
}

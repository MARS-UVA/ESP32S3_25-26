#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "uart.h"
#include "control_startup.h"
#include "can.h"
#include "pdp.h"
#include "ExcavationRobot.h"
#include "tasks.h"
#include "pdh.h"
#include "wifi.h"

// can_rx_context_t can;
PDP pdp;
PDH pdh;

TaskHandle_t current_update_handle = NULL;
TaskHandle_t control_can_handle = NULL;
TaskHandle_t uart_rx_handle = NULL;
TaskHandle_t uart_tx_handle = NULL;

void app_main()
{
    UART_setup();
    // canSetup(&can);
    // PDPInit(&pdp, 62);
    PDHInit(&pdh, 62);
    canSetupRobot(&pdh, &fxMotors[0], 6);
    initializeTalons(&pdh);
    setupWifi();

    vTaskDelay(50);

    xTaskCreate((void *)(write_feedback_task), "uart_can", 4096, NULL, 9, &control_can_handle);
    xTaskCreate((void *)(current_voltage_update_task), "current_update", 4096, &pdh, 9, &current_update_handle);
    xTaskCreate((void *)(temperature_update_task), "temperature_update", 4096, &pdh, 9, &current_update_handle);
    xTaskCreate((void *)(excavation_robot_control_can_task), "uart_can", 4096, NULL, configMAX_PRIORITIES-1, &control_can_handle);
    // axTaskCreate((void *)(UART_rx_task), "uart_rx", 4096, NULL, 7, &uart_rx_handle);
    // xTaskCreate((void *)(UART_tx_task), "uart_tx", 4096, NULL, 10, &uart_tx_handle);
}

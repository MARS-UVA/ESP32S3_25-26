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


//can_rx_context_t can;
PDP pdp;
PDH pdh;


TaskHandle_t current_update_handle = NULL;
TaskHandle_t control_can_handle = NULL;
TaskHandle_t uart_rx_handle = NULL;
TaskHandle_t uart_tx_handle = NULL;

void app_main()
{
    UART_setup();
    //canSetup(&can);
    //PDPInit(&pdp, 62);
    PDHInit(&pdh, 63);
    canSetupPDH(&pdh);
    initializeTalons(&pdh);
    setupWifi();

    vTaskDelay(50);

    // xTaskCreate((void *)(current_update_task), "current_update", 4096, &pdh, 9, &current_update_handle);
    xTaskCreate((void *)(excavation_robot_control_can_task), "uart_can", 4096, NULL, 8, &control_can_handle);
    // xTaskCreate((void *)(UART_rx_task), "uart_rx", 4096, NULL, 7, &uart_rx_handle);
    // xTaskCreate((void *)(UART_tx_task), "uart_tx", 4096, NULL, 10, &uart_tx_handle);
    xTaskCreate((void *)(udp_receive_task), "wifi_recieve", 4096, NULL, 8, &uart_tx_handle);
}

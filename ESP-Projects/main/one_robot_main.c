#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "uart.h"
#include "control_startup.h"
#include "can.h"
#include "talonFX.h"
#include "OneRobot.h"
#include "tasks.h"
#include "pdh.h"

// can_rx_context_t can;
PDH pdh;

TaskHandle_t control_can_handle = NULL;
TaskHandle_t temperature_update_handle = NULL;
TaskHandle_t current_voltage_update_handle = NULL;
TaskHandle_t position_update_handle = NULL;
TaskHandle_t uart_rx_handle = NULL;
TaskHandle_t uart_tx_handle = NULL;
TaskHandle_t enable_handle = NULL;
TaskHandle_t can_enable_handle = NULL;

void app_main()
{
    UART_setup();
    PDHInit(&pdh, 62);
    initializeTalons();
    canSetupRobot(&pdh, &fxMotors[0], 6);
    initAuxVoltageSensor();

    xTaskCreatePinnedToCore((void *)(one_robot_control_can_task), "uart_can", 4096, NULL, configMAX_PRIORITIES - 1, &control_can_handle, 0);
    //xTaskCreatePinnedToCore((void *)(temperature_update_task), "temperature_update", 4096, NULL, 7, &temperature_update_handle, 1);
    //xTaskCreatePinnedToCore((void *)(current_voltage_update_task), "current_voltage_update", 4096, &pdh, 7, &current_voltage_update_handle, 1);
    //xTaskCreatePinnedToCore((void *)(position_update_task), "position_update", 4096, NULL, 7, &position_update_handle, 1);
    xTaskCreatePinnedToCore((void *)(UART_rx_task), "uart_rx", 4096, NULL, 9, &uart_rx_handle, 0);
    xTaskCreatePinnedToCore((void *)(UART_tx_task), "uart_tx", 4096, &pdh, 7, &uart_tx_handle, 1);
    xTaskCreatePinnedToCore((void *)(CAN_enable_task), "can_enable", 4096, NULL, configMAX_PRIORITIES - 2, &can_enable_handle, 1);
}

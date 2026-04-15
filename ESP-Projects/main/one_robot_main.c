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


//can_rx_context_t can;
PDH pdh;

TaskHandle_t control_can_handle = NULL;
TaskHandle_t temperature_update_handle = NULL;
TaskHandle_t uart_rx_handle = NULL;
TaskHandle_t uart_tx_handle = NULL;
TaskHandle_t enable_handle = NULL;
TaskHandle_t motor_task_handle = NULL;

void app_main()
{
    UART_setup();
    PDHInit(&pdh, 62);
    initializeTalons();
    canSetupRobot(&pdh, &fxMotors[0],6);
    //initAuxVoltageSensor();


    xTaskCreate((void *)(one_robot_control_can_task), "uart_can", 4096, NULL, 10, &control_can_handle);
    xTaskCreate((void *)(temperature_update_task), "temperature_update", 4096, NULL, 7, &temperature_update_handle);
    xTaskCreate((void *)(UART_rx_task), "uart_rx", 4096, NULL, 7, &uart_rx_handle);
    xTaskCreate((void *)(UART_tx_task), "uart_tx", 4096, &pdh, 8, &uart_tx_handle);
    xTaskCreate((void *)(enable_task), "enable", 4096, NULL, 9, &enable_handle);
    //xTaskCreate((void *)(motor_task), "motor_task", 4096, NULL, 9, &motor_task_handle);
}

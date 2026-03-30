#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "uart.h"
#include "control_startup.h"
#include "can.h"
#include "pdp.h"
#include "ConstructionRobot.h"
#include "tasks.h"
#include "pdh.h"


// can_rx_context_t can;
// PDP pdp;
PDH pdh;
extern TalonFX backLeft;


// TaskHandle_t current_update_handle = NULL;
TaskHandle_t control_can_handle = NULL;
TaskHandle_t uart_rx_handle = NULL;
// TaskHandle_t uart_tx_handle = NULL;

void app_main()
{
    UART_setup();
    PDHInit(&pdh, 63);
    canSetupPDH(&pdh);
    //PDPInit(&pdp, 62);
    initializeTalons(&pdh);

    vTaskDelay(50);

    // xTaskCreate((void *)(current_update_task), "current_update", 4096, &pdh, 8, &current_update_handle);
    xTaskCreate((void *)(construction_robot_control_can_task), "uart_can", 4096, NULL, 8, &control_can_handle);
    xTaskCreate((void *)(UART_rx_task), "uart_rx", 4096, NULL, 7, &uart_rx_handle);
    // xTaskCreate((void *)(UART_tx_task), "uart_tx", 4096, NULL, 9, &uart_tx_handle);

    // while(1){
    // setTargetFX(&backLeft, 124);
    // vTaskDelay(1);    
    // }
}

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "uart.h"
#include "control_startup.h"
#include "can.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
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
    // UART_setup();
    // initializeTalons();

    // PDHInit(&pdh, 62); 
    // TalonFX testmotor = talonFXInit(33, 3);
    // TalonFX* motors[1] = {&testmotor};
    // canSetupTalonFX(&motors[0], 1);    
    // canSetupRobot(&pdh, &motors[0], 1);
    
    // setFX(&testmotor, 0.5);
    
    // int x = 0;

    // MANUALLY initialize CAN
    // twai_node_handle_t node_hdl = NULL;
    // twai_onchip_node_config_t node_config = {
    //     .io_cfg.tx = 2,             // TWAI TX GPIO pin
    //     .io_cfg.rx = 1,             // TWAI RX GPIO pin
    //     .bit_timing.bitrate = 1000000,  // 200 kbps bitrate
    //     .tx_queue_depth = 32,        // Transmit queue depth set to 5
    // };


    // // Create a new TWAI controller driver instance
    // ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));
    // // Start the TWAI controller
    // ESP_ERROR_CHECK(twai_node_enable(node_hdl));

    // //
    // uint8_t send_buff[8] = {0, 0x01, 0, 0, 0, 0, 0x00, 0x04};
    // uint8_t gefbuff[8] = {0, 0, 0, 0, 0, 0, 1, 0};


    // twai_frame_t setmotor = {
    //     .header.id = CAN_ID_SET_FX | 33,           // Message ID
    //     .header.ide = true,         // Use 29-bit extended ID format
    //     .buffer = send_buff,        // Pointer to data to transmit
    //     .buffer_len = 8,  // Length of data to transmit
    // };

    // twai_frame_t init = {
    //     .header.id = 0x401BF,
    //     .header.ide = true,
    //     .buffer = gefbuff,
    //     .buffer_len = 8,
    // };

    
    // ESP_ERROR_CHECK(twai_node_transmit(node_hdl, &init, 0));  // Timeout = 0: returns immediately if queue is full
    // ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(node_hdl, -1));  // Wait for transmission to finish
    

    // int n = 1;    
    // for (;;) {
    //     printf("running %d\n", n);
    //     ESP_ERROR_CHECK(twai_node_transmit(node_hdl, &setmotor, 0));  // Timeout = 0: returns immediately if queue is full
    //     ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(node_hdl, -1));  // Wait for transmission to finish
    //     vTaskDelay(1);
    //     n++;
    // }

    // initAuxVoltageSensor();

    // xTaskCreatePinnedToCore((void *)(one_robot_control_can_task), "uart_can", 4096, NULL, configMAX_PRIORITIES - 1, &control_can_handle, 0);
    // //xTaskCreatePinnedToCore((void *)(temperature_update_task), "temperature_update", 4096, NULL, 7, &temperature_update_handle, 1);
    // //xTaskCreatePinnedToCore((void *)(current_voltage_update_task), "current_voltage_update", 4096, &pdh, 7, &current_voltage_update_handle, 1);
    // //xTaskCreatePinnedToCore((void *)(position_update_task), "position_update", 4096, NULL, 7, &position_update_handle, 1);
    // xTaskCreatePinnedToCore((void *)(UART_rx_task), "uart_rx", 4096, NULL, 9, &uart_rx_handle, 0);
    // xTaskCreatePinnedToCore((void *)(UART_tx_task), "uart_tx", 4096, &pdh, 7, &uart_tx_handle, 1);
    // xTaskCreatePinnedToCore((void *)(CAN_enable_task), "can_enable", 4096, NULL, configMAX_PRIORITIES - 2, &can_enable_handle, 1);


    TalonFX testmotor = talonFXInit(33, 3);
    TalonFX* motors[1] = {&testmotor};
    
    // PDHInit(&pdh, 62);
    // canSetupRobot(&pdh, &motors[0], 1);
    
    canSetupTalonFX(&motors[0], 1);

    vTaskDelay(pdMS_TO_TICKS(10));

    for(;;) {
        // vTaskDelay(pdMS_TO_TICKS(10));
        sendEn();
        setFX(&testmotor, 0.5);

        printf("hi\n");
        
    }
    
}

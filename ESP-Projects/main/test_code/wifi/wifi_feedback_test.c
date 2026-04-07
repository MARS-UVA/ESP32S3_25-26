#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "uart.h"
// #include "control_startup.h"
#include "can.h"
#include "talonFX.h"
#include "ExcavationRobot.h"
#include "tasks.h"
#include "pdh.h"
#include "wifi.h"

TaskHandle_t control_can_handle = NULL;

void app_main()
{
    // Commented out since we are only testing Wi-Fi right now.
    // initializeTalons(&pdh); 
    UART_setup();
    setupWifi();

    CurrVoltPacket_ExcavationRobot packet = Init_CurrVolt_Excavation_Robot_Packet();
    packet.front_left_wheel = 1.23f;
    packet.back_left_wheel = 4.56f;
    packet.front_right_wheel = 7.89f;
    packet.back_right_wheel = 0.12f;
    packet.bucket_ladder = 3.45f;
    packet.conveyor_belt = 6.78f;
    packet.left_track_actuator = 9.01f;
    packet.right_track_actuator = 2.34f;
    packet.main_battery = 6.78f;
    packet.aux_battery = 9.01f;



    for(;;)
    {
        wifi_write(&packet, sizeof(packet));
        //xTaskCreate((void *)(temperature_update_task), "temperature_update", 4096, NULL, 7, &temperature_update_handle);
        //xTaskCreate((void *)(UART_rx_task), "uart_rx", 4096, NULL, 7, &uart_rx_handle);
        //xTaskCreate((void *)(UART_tx_task), "uart_tx", 4096, &pdh, 9, &uart_tx_handle);
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Limit to 10 packets per second!
    }

}
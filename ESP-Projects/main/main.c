#include "control_startup.c"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <uart.h>

TaskHandle_t readTaskHandle = NULL;


void app_main()
{
    initializeTalons();
    UART_setup();
    canSetup(g_node_hdl);

    SerialPacket packet;



    xTaskCreateTaskPinnedToCore(Read_Task, "Read_Task", 4096, NULL, uart_priority, &readTaskHandle, 1);

    while (true) {   
        packet = UART_read(); 

        directControl(packet);

        //setTargetFX(g_node_hdl, &frontLeft, 200);
    }
}

void Read_Task(void *arg)
{
    SerialPacket packet;
    while (true) {   
        packet = uart_event_task(NULL);
        if (!packet) {
            packet = {0};
            packet.invalid = 1;
        }   
    }
}
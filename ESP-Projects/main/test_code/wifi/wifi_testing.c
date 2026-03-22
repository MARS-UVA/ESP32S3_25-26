#include "wifi.h"
#include "uart.h"
#include "packets.h"

extern QueueHandle_t wifi_queue;

//ControlPacket_OneRobot packet = Init_contr

void wifi_testing() {
    setupWifi();
    while (1) {
        
    }
}
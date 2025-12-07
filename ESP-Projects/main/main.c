#include "uart.h"
#include "wifi.h"
#include "nvs_flash.h"

void app_main()
{     
        setupWifi();
        vTaskDelay(500);
        print_IP();
        vTaskDelay(500);
        sendWifiPacket(NULL);
    
}
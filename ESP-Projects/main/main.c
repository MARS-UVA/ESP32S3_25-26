#include "uart.h"
#include "wifi.h"
#include "nvs_flash.h"

void app_main()
{     
        setupWifi();
        print_IP();
    
}
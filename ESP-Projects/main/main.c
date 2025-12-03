#include "uart.h"
#include "wifi.h"
#include "nvs_flash.h"

void app_main()
{       nvs_flash_init();
    
        setupWifi();
    
}
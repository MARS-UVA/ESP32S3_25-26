#pragma once

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

/* --------------------- Functions ------------------ */
void setupWifi(void);
void print_IP(void);
void sendWifiPacket(void *pvParameters);
void udp_receive_task(void *pvParameters);

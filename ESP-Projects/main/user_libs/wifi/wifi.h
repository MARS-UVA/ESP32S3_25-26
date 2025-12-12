#ifndef WIFI_H
#define WIFI_H

#include "esp_wifi.h"

/* --------------------- Functions ------------------ */
void setupWifi(void);
void print_IP(void);
void sendWifiPacket(void *pvParameters);
void udp_receive_task(void *pvParameters);
#endif // WIFI_H
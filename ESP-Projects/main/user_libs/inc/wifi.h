#pragma once

#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "uart.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "control_startup.h"
#include "packets.h"
#include <stddef.h>

/* --------------------- Functions ------------------ */
void setupWifi(void);
void get_IP(void);
void sendWifiPacket(void *pvParameters);
void udp_receive_task(void *pvParameters);
void wifi_write(void *packet, size_t size);
void write_feedback_task(void);

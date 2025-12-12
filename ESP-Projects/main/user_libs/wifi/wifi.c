#include "./wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "control_startup.h"
#include "uart.h"
#include "freertos/queue.h"

static const char *TAG = "wifi";

#define PORT 25000
#define HOST_IP_ADDR "172.20.10.13"

extern QueueHandle_t uart_queue;

/*void udp_server_task_setup(void *pvParameters) {
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    while (1) {
        ESP_LOGI(TAG, "Waiting for data");
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            break;
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes from:", len);
            ESP_LOGI(TAG, "%s", rx_buffer);
        }
    }

    if (sock != -1) {
        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        close(sock);
    }
    vTaskDelete(NULL);
}*/

void udp_receive_task(void *pvParameters) 
{
    char RxBuffer[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);


    SerialPacket pkt[8] = {0};
    while (1) {
        ESP_LOGI(TAG, "Waiting for data");
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, RxBuffer, sizeof(RxBuffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            break;
        } else 
        {
            ESP_LOGI("Wifi", "Got packet");
            RxBuffer[len] = 0;  // Null-terminate whatever is received and treat it like a string
            ESP_LOGI("Wifi", "%x\n", RxBuffer[2]);
            pkt->invalid = 0;
            pkt->header = RxBuffer[1];
            pkt->top_left_wheel = RxBuffer[2];
            pkt->back_left_wheel = RxBuffer[3];
            pkt->top_right_wheel = RxBuffer[4];
            pkt->back_right_wheel = RxBuffer[5];
            pkt->drum = RxBuffer[6];
            pkt->actuator = RxBuffer[7];
            xQueueOverwrite(uart_queue, &pkt);
        }
    }

    if (sock != -1) {
        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        close(sock);
    }
    vTaskDelete(NULL);
}

void sendWifiPacket(void *pvParameters) 
{
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) 
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelete(NULL);
            return;
        }
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);

    
    while (1) 
    {
        // Implement sending logic here
        const char *payload = "Hello from ESP32";
        int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) 
        {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        }
        else 
        {
            ESP_LOGI(TAG, "Message sent");
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Send every 2
        
    }
}

void setupWifi() {
    /* ------------- Wfi Configs -------------*/
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_mode_t mode = WIFI_MODE_STA;
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Team_02",
            .password = "marsuva!",
            //.ssid = "Team_02",
            //.password = "marsuva!",
        }
    };


    /* ------------- Initialize network + event loop (required) -------------*/
    nvs_flash_init();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();


    /* ------------- Initialize WiFi -------------*/
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(mode));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    //ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(receiveWifiPacket));
    //ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
    //Make sure to make this a variable later
    //ESP_ERROR_CHECK(esp_wifi_set_channel())
    ESP_ERROR_CHECK(esp_wifi_start());

    /* ------------- Connect to Wfi -------------*/


    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_connect failed: %s (0x%X)", esp_err_to_name(err), err);
    } else {
        ESP_LOGI(TAG, "esp_wifi_connect ok");
    }

    vTaskDelay(50);}
void print_IP(void)
{
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_get_ip_info(netif, &ip_info);
    ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&ip_info.ip));
}
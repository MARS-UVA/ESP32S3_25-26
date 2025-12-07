#include "./wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "wifi";


void receiveWifiPacket(void *buf, wifi_promiscuous_pkt_type_t type) {
    // Process the received packet
    const wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;

    ESP_LOGI(TAG, "Packet received! Type: %d, Len: %d, RSSI: %d", 
             type, 
             pkt->rx_ctrl.sig_len, 
             pkt->rx_ctrl.rssi);
}

void setupWifi() {
    /* ------------- Wfi Configs -------------*/
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_mode_t mode = WIFI_MODE_STA;
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "UVA Guest",
            .password = "",
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
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(receiveWifiPacket));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
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
}
void print_IP(void)
{
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_get_ip_info(netif, &ip_info);
    ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&ip_info.ip));
}
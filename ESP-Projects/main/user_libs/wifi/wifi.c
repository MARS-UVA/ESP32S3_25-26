#include "esp_wifi.h"


void setupWifi() {
    /* ------------- Wfi Configs -------------*/
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_mode_t mode = WIFI_MODE_STA;
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "YOUR_SSID",
            .password = "YOUR_PASSWORD",
        }
    };

    /* ------------- Initalize Wfi -------------*/


    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(mode));
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* ------------- Connect to Wfi -------------*/

    ESP_ERROR_CHECK(esp_wifi_connect());
}
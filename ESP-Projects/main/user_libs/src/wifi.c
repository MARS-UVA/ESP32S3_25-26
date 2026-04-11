#include "wifi.h"
#include "packets.h"

// DEBUG/LOGGING TAG
static const char *TAG = "WIFI_APP";

/// Wi-Fi configuration
#define PORT 2001
#define TARGET_IP_ADDR "192.168.50.101" // <-- CHANGE THIS TO JETSON/LAPTOP IP!

/// Wi-Fi credentials
#define WIFI_SSID "Team_39"
#define WIFI_PASS "lunabotics#1"

// Maximum size of a Wi-Fi packet, based on the largest packet structure we expect to send/receive
#define MAX_WIFI_PACKET_SIZE sizeof(CurrVoltPacket_ExcavationRobot)
typedef struct
{
    uint8_t data[MAX_WIFI_PACKET_SIZE];
    size_t size;
} WifiQueueItem;

// All the RTOS queues used
extern QueueHandle_t control_queue;
QueueHandle_t wifi_queue;
QueueHandle_t currvolt_queue;
QueueHandle_t temperature_queue;
QueueHandle_t position_queue;

/**
 * @brief Establishes a UDP connection and continuously listens for packets sent to the specified port. When a packet is received, it is parsed and added to the appropriate FreeRTOS queue based on its header value. DO NOT CALL THIS FUNCTION DIRECTLY, INSTEAD USE setupWifi() TO INITIALIZE THE WIFI AND START THIS TASK.
 *
 * @param pvParameters FreeRTOS task parameters (not used).
 */
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
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int flag = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag));
    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    ControlPacket_ExcavationRobot control_pkt;
    CurrVoltPacket_ExcavationRobot curr_volt_pkt;
    TempPacket_ExcavationRobot temp_pkt;
    PositionPacket_ExcavationRobot position_pkt;

    while (1)
    {
        // ESP_LOGI(TAG, "Waiting for data");
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, RxBuffer, sizeof(RxBuffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len < 0)
        {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            break;
        }
        else
        {
            switch (RxBuffer[1])
            {
            case 0x00:
                // Control packet
                // ESP_LOGI(TAG, "Got Control packet");
                if (len >= sizeof(ControlPacket_ExcavationRobot))
                {
                    memcpy(&control_pkt, RxBuffer, sizeof(ControlPacket_ExcavationRobot));
                    xQueueOverwrite(control_queue, &control_pkt);
                }
                break;
            case 0x01:
                // Curr/Volt packet
                ESP_LOGI(TAG, "Got Curr/Volt packet");
                if (len >= sizeof(CurrVoltPacket_ExcavationRobot))
                {
                    memcpy(&curr_volt_pkt, RxBuffer, sizeof(CurrVoltPacket_ExcavationRobot));
                    xQueueOverwrite(currvolt_queue, &curr_volt_pkt);
                }
                break;
            case 0x02:
                // Temp packet
                ESP_LOGI(TAG, "Got Temp packet");
                if (len >= sizeof(TempPacket_ExcavationRobot))
                {
                    memcpy(&temp_pkt, RxBuffer, sizeof(TempPacket_ExcavationRobot));
                    xQueueOverwrite(temperature_queue, &temp_pkt);
                }
                break;
            case 0x03:
                // Position packet
                ESP_LOGI(TAG, "Got Position packet");
                if (len >= sizeof(PositionPacket_ExcavationRobot))
                {
                    memcpy(&position_pkt, RxBuffer, sizeof(PositionPacket_ExcavationRobot));
                    xQueueOverwrite(position_queue, &position_pkt);
                }
                break;
            default:
                break;
            }
        }
    }

    if (sock != -1)
    {
        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        close(sock);
    }
    vTaskDelete(NULL);
}

void write_feedback_task(void)
{

    while (1)
    {
        {
            TempPacket_ExcavationRobot new_data;
            if (xQueueReceive(temperature_queue, &new_data, 0) == pdTRUE)
            {
                wifi_write(&new_data, sizeof(TempPacket_ExcavationRobot));
            }
        }

        {
            PositionPacket_ExcavationRobot new_data;
            if (xQueueReceive(position_queue, &new_data, 0) == pdTRUE)
            {
                wifi_write(&new_data, sizeof(PositionPacket_ExcavationRobot));
            }
        }

        {
            CurrVoltPacket_ExcavationRobot new_data;
            if (xQueueReceive(currvolt_queue, &new_data, 0) == pdTRUE)
            {
                wifi_write(&new_data, sizeof(CurrVoltPacket_ExcavationRobot));
            }
        }

        vTaskDelay(1);
    }
}

/**
 * @brief Establishes a UDP connection and continuously sends packets from the Wi-Fi transmit queue to the specified target IP address and port. DO NOT USE THIS FUNCTION, INSTEAD USE wifi_write() TO ADD PACKETS TO THE TRANSMIT QUEUE.
 *
 * @param pvParameters FreeRTOS task parameters (not used).
 */
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
    dest_addr.sin_addr.s_addr = inet_addr(TARGET_IP_ADDR);

    WifiQueueItem item;

    while (1)
    {
        // Wait indefinitely until ANY packet is added to the wifi queue
        if (wifi_queue != NULL && xQueueReceive(wifi_queue, &item, portMAX_DELAY) == pdTRUE)
        {
            int err = sendto(sock, item.data, item.size, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0)
            {
                ESP_LOGE(TAG, "Error occurred during sending packet: errno %d", errno);
                vTaskDelay(pdMS_TO_TICKS(50)); // If fails, wait to try again
            }
        }
    }
}

static void wifiEventHandler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        // Attempt to reconnect
        esp_wifi_connect();
        ESP_LOGI("WIFI", "Trying to reconnect...");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("WIFI", "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

/**
 * @brief Sets up all the wifi configurations and starts the Wi-Fi connection. Also initializes the FreeRTOS queues and tasks for Wi-Fi communication.
 */
void setupWifi()
{
    /* ------------- Wfi Configs -------------*/
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_mode_t mode = WIFI_MODE_STA;
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        }};

    /* ------------- Initialize network + event loop (required) -------------*/
    nvs_flash_init();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    /* ------------- Suppress Internal Wi-Fi Logs -------------*/
    esp_log_level_set("wifi", ESP_LOG_ERROR);

    /* ------------- Initialize WiFi -------------*/
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(mode));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, NULL, NULL));
    // ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(receiveWifiPacket));
    // ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
    // Make sure to make this a variable later
    // ESP_ERROR_CHECK(esp_wifi_set_channel())
    ESP_ERROR_CHECK(esp_wifi_start());

    /* ------------- Connect to Wfi -------------*/

    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_connect failed: %s (0x%X)", esp_err_to_name(err), err);
    }
    else
    {
        ESP_LOGI(TAG, "esp_wifi_connect ok");
    }

    /* ------------- Setupt Queues -------------*/
    wifi_queue = xQueueCreate(10, sizeof(WifiQueueItem));
    currvolt_queue = xQueueCreate(1, sizeof(CurrVoltPacket_ExcavationRobot));
    temperature_queue = xQueueCreate(1, sizeof(TempPacket_ExcavationRobot));
    position_queue = xQueueCreate(1, sizeof(PositionPacket_ExcavationRobot));

    // get_IP();

    /* ------------- FreeRTOS Tasks -------------*/
    xTaskCreate(udp_receive_task, "udp_rx_task", 4096, NULL, 7, NULL);
    xTaskCreate(sendWifiPacket, "wifi_tx_task", 4096, NULL, 8, NULL);
}

/**
 * @brief Gets the IP address of the ESP32 and logs it in the console. This function blocks until a valid IP address is obtained.
 */
void get_IP()
{
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    // Wait until we get a non zero IP address
    ESP_LOGI(TAG, "Waiting for IP address...");
    do
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_netif_get_ip_info(netif, &ip_info);
    } while (ip_info.ip.addr == 0);

    ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&ip_info.ip));
}

/**
 * @brief Sends a packet over Wi-Fi by adding it to the Wi-Fi transmit queue.
 *
 * @param packet   The data packet you want to send.
 * @param size   The size of the data packet.
 */
void wifi_write(void *packet, size_t size)
{
    if (wifi_queue == NULL || size > MAX_WIFI_PACKET_SIZE)
        return;

    WifiQueueItem item;
    item.size = size;
    memcpy(item.data, packet, size);

    xQueueSend(wifi_queue, &item, 0); // Send to queue without blocking
}

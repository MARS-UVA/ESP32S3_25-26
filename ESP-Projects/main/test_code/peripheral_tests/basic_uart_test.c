#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_err.h"

#define UART_PORT       UART_NUM_1
#define UART_TX_PIN     17
#define UART_RX_PIN     18
#define UART_BAUD_RATE  115200

void app_main(void)
{
    // Configure UART
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        UART_NUM_1, 1024, 0, 0, NULL, 0
    ));

    ESP_ERROR_CHECK(uart_param_config(
        UART_NUM_1, &uart_config
    ));

    ESP_ERROR_CHECK(uart_set_pin(
        UART_NUM_1,
        17,
        18,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    // Transmit repeatedly
    while (1) {
        const char *message = "Hello from ESP32-S3!\r\n";

        uart_write_bytes(
            UART_NUM_1,
            message,
            strlen(message)
        );

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
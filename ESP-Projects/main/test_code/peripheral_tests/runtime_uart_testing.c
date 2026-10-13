#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

/*
This is a generalized testing script to test UART transmission on the custom ESP32-Dev Kit

For the Custom Devkit, the 

*/


void app_main(void)
{
    // 1. Configure UART0
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 2. Install driver (only if not already installed)
    ESP_ERROR_CHECK(uart_driver_install(
        UART_NUM_0, 2048, 2048, 0, NULL, 0));

    // 3. Apply configuration
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    // 4. Assign UART0 TX/RX pins
    ESP_ERROR_CHECK(uart_set_pin(
        UART_NUM_0, 36, 37,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 5. Define test packet
    uint8_t packet[] = {127, 127, 5, 72, 21, 104, 127, 62, 45, 127};

    // 6. Transmit repeatedly
    while (1) {
        int bytes_sent = uart_write_bytes(
            UART_NUM_0, (const char *)packet, sizeof(packet));

        ESP_LOGI("UART_TEST", "Sent %d bytes", bytes_sent);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

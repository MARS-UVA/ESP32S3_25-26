/* Luke Oliver, Shawn Mitchell, Carlos Giron 9/24/25
CAN set Output example based off Template for starting twai drivers
*/

#include <stdio.h>
#include <stdlib.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* --------------------- Definitions and static variables ------------------ */

// setting rx and tx pins into can transciever
#define RX_GPIO_NUM GPIO_NUM_1
#define TX_GPIO_NUM GPIO_NUM_2
#define EXAMPLE_TAG "TWAI Set Output"

// sets conifigs for our twai driver to send messages
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();     // CAN bus timing on our talon fx
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // Accept all incoming messages for the driver
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

twai_message_t enable_msg = {
    // Message type and format settings
    .extd = 1,         // Standard Format message (29-bit ID)
    .rtr = 0,          // Send a data frame if 0
    .ss = 0,           // Not single shot
    .self = 0,         // Message is not a self reception request (loopback)
    .dlc_non_comp = 0, // DLC is less than 8

    // Message ID and payload for CAN enable frame
    .identifier = 0x401bf,
    .data_length_code = 2,
    .data = {0x01, 0x00},
};

twai_message_t drive_msg = {
    // Message type and format settings
    .extd = 1,         // Standard Format message (29-bit ID)
    .rtr = 0,          // Send a data frame
    .ss = 0,           // Not single shot
    .self = 0,         // Message is not a self reception request (loopback)
    .dlc_non_comp = 0, // DLC is less than 8

    // Message ID and payload for CAN set Output frame
    .identifier = 0x204b540 | 0x1b,
    .data_length_code = 8,
    .data = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};

twai_message_t neutral_mode_msg = {
    // Message type and format settings
    .extd = 1,         // Standard Format message (29-bit ID)
    .rtr = 0,          // Send a data frame
    .ss = 0,           // Not single shot
    .self = 0,         // Message is not a self reception request (loopback)
    .dlc_non_comp = 0, // DLC is less than 8

    // Message ID and payload for CAN set Output frame
    .identifier = 0x2047c00 | 0x1b,
    .data_length_code = 8,
    .data = {0x21, 0x6E, 0x08, 0x00, 0x00, 0x00, 0x00, 0xAA},

};

// function to drive a motor at speed/1024 percent output
void talonPercentOut(int16_t speed)
{
    ESP_ERROR_CHECK(twai_transmit(&enable_msg, portMAX_DELAY));

    uint8_t spBytes[2];
    spBytes[0] = (speed >> 8) & 0x0f; // high byte
    spBytes[1] = (speed & 0xff);      // low byte

    drive_msg.data[7] = spBytes[0]; // flipping endianness
    drive_msg.data[6] = spBytes[1];

    printf("Percent Output:\t%.2f%%\n", (double)speed / 10.24);
    printf("Data[0]:\t%x\n", spBytes[0]);
    printf("Data[1]:\t%x\n\n", spBytes[1]);

    ESP_ERROR_CHECK(twai_transmit(&drive_msg, portMAX_DELAY));
}

void rampUpDownSpeed()
{
    for (int16_t i = 0; i < 256; i++)
    {
        talonPercentOut(i);
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    for (int16_t i = 255; i > -256; i--)
    {
        talonPercentOut(i);
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    for (int16_t i = -255; i < 0; i++)
    {
        talonPercentOut(i);
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void app_main(void)
{
    // Install and start TWAI driver for CAN
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    ESP_ERROR_CHECK(twai_transmit(&enable_msg, portMAX_DELAY));
    rampUpDownSpeed();

    // stop then uninstall twai driver
    ESP_ERROR_CHECK(twai_stop());
    ESP_LOGI(EXAMPLE_TAG, "Driver stopped");

    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");
}

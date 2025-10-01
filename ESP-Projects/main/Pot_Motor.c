#include <esp_adc/adc_continuous.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/twai.h"

/*
Authors: Carlos Giron

USED PIN 4 & 5 FOR CAN RX, TX
USED PIN 2 FOR ADC INPUT!!!
*/
// setting rx and tx pins into can transciever
#define RX_GPIO_NUM GPIO_NUM_4
#define TX_GPIO_NUM GPIO_NUM_5
#define EXAMPLE_TAG "TWAI Set Output"

// sets conifigs for our twai driver to send messages
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();     // CAN bus timing on our talon fx
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // Accept all incoming messages for the driver
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

static TaskHandle_t s_task_handle;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle;

    // ADC continuous mode driver initial configurations.
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024, // set the maximum size of the pool in bytes, and the driver saves ADC conversion result into the pool.
        .conv_frame_size = 256,     // conversion frame size, in bytes
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    // ADC continuous mode driver configurations. kind of stupid. why couldn't adc_config and digi be combined?
    adc_continuous_config_t digi_config = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    // ADC digital controller pattern configuration
    adc_digi_pattern_config_t adc_pattern = {
        .unit = ADC_UNIT_1,
        .channel = ADC_CHANNEL_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH, // Use SOC constant
        .atten = ADC_ATTEN_DB_12,
    };
    digi_config.pattern_num = 1;
    digi_config.adc_pattern = &adc_pattern;

    ESP_ERROR_CHECK(adc_continuous_config(handle, &digi_config));
    *out_handle = handle;
}

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

void app_main()
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[256] = {0};
    memset(result, 0xcc, 256); // set all elements at results buffer to value 0xcc

    s_task_handle = xTaskGetCurrentTaskHandle(); // FIX: Set the task handle before initializing ADC

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(&handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    // Install and start TWAI driver for CAN
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    uint32_t data = 0;
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for conversion done callback

        while (1)
        {
            ret = adc_continuous_read(handle, result, 256, &ret_num, 0);
            if (ret == ESP_OK)
            {
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
                    data = p->type2.data;
                    printf("Value:\t%ld\n", data);
                }
                talonPercentOut((uint16_t)data / 8);
                vTaskDelay(pdMS_TO_TICKS(25)); // Add delay between reads
            }
            else if (ret == ESP_ERR_INVALID_STATE)
            {
                break; // No more data available, wait for next conversion
            }
        }
    }
}
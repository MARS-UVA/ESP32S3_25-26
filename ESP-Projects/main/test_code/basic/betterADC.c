#include <esp_adc/adc_continuous.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"

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
                    uint32_t data = p->type2.data;
                    printf("Value:\t%ld\n", data);
                }
                vTaskDelay(pdMS_TO_TICKS(100)); // Add delay between reads
            }
            else if (ret == ESP_ERR_INVALID_STATE)
            {
                break; // No more data available, wait for next conversion
            }
        }
    }
}
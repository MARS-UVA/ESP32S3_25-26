#include "adc2.h"
#include "utils.h"
#include <esp_adc/adc_continuous.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

adc_continuous_handle_t g_adc_hdl = NULL;

TaskHandle_t s_task_handle;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = 1024,
    .conv_frame_size = 256,
};

adc_continuous_config_t digi_config = {
    .sample_freq_hz = 20 * 1000,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
};

adc_continuous_evt_cbs_t cbs = {
    .on_conv_done = s_conv_done_cb,
};

// read a decimal value between 0 and 1 indicating position of potentiometer
void readPot(Pot *pot)
{
    esp_err_t ret;
    uint32_t ret_num = 0;      // this variable will hold the number of bytes read from the ADC
    uint8_t result[256] = {0}; // this line makes a new array called 'result' of size 256 bytes
    memset(result, 0xcc, 256); // set all elements at results buffer to value 0xcc

    uint32_t data_samples = 0;
    uint32_t chan_num = 0;
    uint32_t sample_num = 0;
    uint32_t sample_sum = 0;

    ret = adc_continuous_read(g_adc_hdl, result, 256, &ret_num, 0);
    if (ret == ESP_OK)
    {
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
        {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
            chan_num = p->type2.channel;
            data_samples = p->type2.data;

            if (chan_num == pot->channel)
            {
                // printf("channel: \t%u, min: \t%u, max: \t%u", pot->channel, pot->minPos, pot->maxPos);
                sample_num += 1;
                sample_sum += data_samples;

                // ESP_LOGI("ADC", "Channel: %lu, Value: %lu, Mapped: %lf", chan_num, data_samples, pot->pos);
                // printf("%lu\t%lu\t%lu\n", chan_num, data_samples, xTaskGetTickCount());

                // vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }
        }
    }
    pot->pos = (double)map(pot->minPos, pot->maxPos, (double)sample_sum / sample_num);
    // pot->pos = (double)sample_sum / sample_num;
    vTaskDelay(1);
}

// this function initializes the Potentiometer struct and its ADC handle
void potSetup(adc_channel_t *channel, uint8_t channel_num)
{
    s_task_handle = xTaskGetCurrentTaskHandle(); // FIX: Set the task handle before initializing ADC

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &g_adc_hdl));

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    digi_config.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = channel[i];
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    digi_config.adc_pattern = adc_pattern;

    ESP_ERROR_CHECK(adc_continuous_config(g_adc_hdl, &digi_config));
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(g_adc_hdl, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(g_adc_hdl));
}

Pot potInit(int minPos, int maxPos, adc_channel_t channel)
{
    return (Pot){
        .minPos = minPos,
        .maxPos = maxPos,
        .channel = channel,
        .pos = 0,
    };
}

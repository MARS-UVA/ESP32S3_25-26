#include "adc.h"
#include "utils.h"
#include <esp_adc/adc_continuous.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <driver/adc.h>
#include "esp_err.h"
#include "driver/gpio.h"

Pot pot = {
    .handle = NULL,
    .actuatorOffset = 0,
    .minPos = 1190, // minimum ADC reading
    .maxPos = 3153,
    .pos = 0,
};

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
void readPot()
{

    esp_err_t ret;
    uint32_t ret_num = 0;      // this variable will hold the number of bytes read from the ADC
    uint8_t result[256] = {0}; // this line makes a new array called 'result' of size 256 bytes
    memset(result, 0xcc, 256); // set all elements at results buffer to value 0xcc

    uint32_t data_samples = 0;

    while (1)
    {

        while (1)
        {
            ret = adc_continuous_read(pot.handle, result, 256, &ret_num, 0);
            if (ret == ESP_OK)
            {
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
                    data_samples = p->type2.data;
                    printf("Value:\t%ld\n", data_samples);
                }
                vTaskDelay(pdMS_TO_TICKS(100)); // Add delay between reads

                // pot.pos = map(pot.minPos, pot.maxPos, data_samples + pot.actuatorOffset);
            }
            else if (ret == ESP_ERR_INVALID_STATE)
            {
                break; // No more data available, wait for next conversion
            }
        }
    }
}

// this function initializes a Potentiometer struct and its ADC handle
void PotInit(adc_unit_t unit, adc_channel_t channel)
{

    adc_continuous_handle_t handle;

    s_task_handle = xTaskGetCurrentTaskHandle(); // FIX: Set the task handle before initializing ADC

    // this is how you intialize  the ADC Continous Driver mode

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_digi_pattern_config_t adc_pattern = {
        .unit = unit,
        .channel = channel,                     // ADC_CHANNEL_1
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH, // Use SOC constant
        .atten = ADC_ATTEN_DB_12,
    };

    digi_config.pattern_num = 1;
    digi_config.adc_pattern = &adc_pattern;

    ESP_ERROR_CHECK(adc_continuous_config(handle, &digi_config));

    pot.handle = handle;

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(pot.handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(pot.handle));
}

// Deinitialize the Potentiometer's ADC handle --> Helpers to avoid memory leaks
void PotDeInit(Pot *pot)
{
    if (pot != NULL && pot->handle != NULL)
    {
        adc_continuous_stop(pot->handle);   // Stop ADC continuous reading
        adc_continuous_deinit(pot->handle); // Delete ADC handle to allow free memory
        pot->handle = NULL;
        ESP_LOGI("FUNC: PotDeInit", "Potentiometer ADC Deinitialized.");
    }
}
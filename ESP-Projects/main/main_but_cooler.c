#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "esp_adc/adc_continuous.h"
#include "driver/gpio.h"
#include "uart.h"
#include "can.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "talonFX.h"
#include "tasks.h"

// Control pins for Kraken and actuator motors
#define KRAKEN1_CONTROL_PIN GPIO_NUM_4
#define KRAKEN2_CONTROL_PIN GPIO_NUM_5
#define ACTUATOR_CONTROL_PIN GPIO_NUM_6
 
// Task handles for various control tasks
TaskHandle_t control_can_handle = NULL;
TaskHandle_t temperature_update_handle = NULL;
TaskHandle_t current_voltage_update_handle = NULL;
TaskHandle_t position_update_handle = NULL;
TaskHandle_t uart_rx_handle = NULL;
TaskHandle_t uart_tx_handle = NULL;
TaskHandle_t enable_handle = NULL;
TaskHandle_t can_enable_handle = NULL;

// ADC configuration
#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL_KRAKEN1 GPIO_NUM_4
#define ADC_CHANNEL_KRAKEN2 GPIO_NUM_5
#define ADC_CHANNEL_ACTUATOR GPIO_NUM_6
#define ADC_CHANNEL_BUFFER_SIZE 512


// adc_continuous_handle_cfg_t kraken1_adc_config = {
//     .max_store_buf_size = (uint32_t)512,
//     .conv_frame_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV,
//     .flags = {
//         .flush_pool = true,
//     },
// };

adc_digi_pattern_config_t adc_pattern[3] = {
    {
        .atten = ADC_ATTEN_DB_12,
        .channel = 3,
        .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12,
    },
    {
        .atten = ADC_ATTEN_DB_12,
        .channel = 4,
        .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12,
    },
    {
        .atten = ADC_ATTEN_DB_12,
        .channel = 5,
        .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12,
    },
};

adc_continuous_handle_t handle = NULL;
adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = 1024,
    .conv_frame_size = 256,
};
adc_continuous_config_t adc_reading_config = {
    .sample_freq_hz = 10000,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    .pattern_num = 3,
    .adc_pattern = adc_pattern,
};


void app_main()
{
    // TalonFX and TalonSRX Initialization
    TalonFX testMotor1 = talonFXInit(25, 4);
    TalonFX testMotor2 = talonFXInit(37, 3);
    TalonFX* motors[2] = {&testMotor1, &testMotor2};
    canSetupTalonFX(motors, 2);

    TalonSRX testactuator = talonSRXInit(4, 2, false);
    TalonSRX* actuators[1] = {&testactuator};
    
    // Initialize ADC pin
    esp_err_t err = ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));
    if(err != ESP_OK) {
        printf("Failed to create ADC handle: %s\n", esp_err_to_name(err));
        return;
    }
    err = adc_continuous_config(handle, &adc_reading_config);
    if(err != ESP_OK) {
        printf("Failed to configure ADC handle: %s\n", esp_err_to_name(err));
        return;
    }
    err = adc_continuous_start(handle);
    if(err != ESP_OK) {
        printf("Failed to start ADC handle: %s\n", esp_err_to_name(err));
        return;
    }
    printf("ADC continuous mode started successfully.\n");
    vTaskDelay(pdMS_TO_TICKS(10));


    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(10));

        uint8_t adc_data[ADC_CHANNEL_BUFFER_SIZE];
        uint32_t num_converted = 0;

        float kraken1_pot_reading = 0;
        float kraken2_pot_reading = 0;
        float actuator_pot_reading = 0;

        err = adc_continuous_read(handle, adc_data, sizeof(adc_data), &num_converted, pdMS_TO_TICKS(100));
        if(err == ESP_OK) {
            for(int i = 0; i < num_converted; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *result = (adc_digi_output_data_t *)&adc_data[i];
                uint16_t raw = result->type2.data;
                switch (result->type2.channel) {
                    case 3:
                        kraken1_pot_reading = raw / 4095.0f;
                        break;

                    case 4:
                        kraken2_pot_reading = raw / 4095.0f;
                        break;

                    case 5:
                        actuator_pot_reading = raw / 4095.0f;
                        break;
                }
            }
            printf("K1: %.2f | K2: %.2f | Actuator: %.2f \n", kraken1_pot_reading, kraken2_pot_reading, actuator_pot_reading);
        }
        else{
            printf("Failed to read ADC data: %s\n", esp_err_to_name(err));
            continue;
        }

        sendEn();
        setFX(&testMotor1, kraken1_pot_reading);
        setFX(&testMotor2, kraken2_pot_reading);
        if(actuator_pot_reading > 0.5){
            testactuator.inverted = false;
            actuator_pot_reading = (actuator_pot_reading - 0.5f) * 2.0f;
        }
        else{
            testactuator.inverted = true;
            actuator_pot_reading = actuator_pot_reading * 2.0f;
        }    
        setSRX(&testactuator, actuator_pot_reading);
    }
}

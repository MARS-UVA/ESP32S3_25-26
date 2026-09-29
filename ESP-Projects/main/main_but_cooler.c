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


adc_continuous_handle_cfg_t kraken1_adc_config = {
    .max_store_buf_size = (uint32_t)512,
    .conv_frame_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV,
    .flags = {
        .flush_pool = true,
    },
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
    


    gpio_set_direction(KRAKEN1_CONTROL_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(KRAKEN2_CONTROL_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(ACTUATOR_CONTROL_PIN, GPIO_MODE_INPUT);



    vTaskDelay(pdMS_TO_TICKS(10));

    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(10));

        float kraken1_pot_reading = gpio_get_level(KRAKEN1_CONTROL_PIN) / 3.3;
        float kraken2_pot_reading = gpio_get_level(KRAKEN2_CONTROL_PIN) / 3.3;
        float actuator_pot_reading = gpio_get_level(ACTUATOR_CONTROL_PIN) / 3.3;

        printf("Kraken1 pot reading: %f\n", kraken1_pot_reading);
        printf("Kraken2 pot reading: %f\n", kraken2_pot_reading);
        printf("Actuator pot reading: %f\n", actuator_pot_reading);

        if(actuator_pot_reading > 0.5){
            testactuator.inverted = false;
        }
        else{
            testactuator.inverted = true;
        }    
        sendEn();
        setFX(&testMotor1, kraken1_pot_reading);
        setFX(&testMotor2, kraken2_pot_reading);
        setSRX(&testactuator, actuator_pot_reading);
    }
    
}

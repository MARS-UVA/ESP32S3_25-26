#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Timer configuration defines
#define TIMER_CONFIG_SPEEDMODE LEDC_LOW_SPEED_MODE
#define TIMER_CONFIG_RESOLUTION LEDC_TIMER_8_BIT
#define TIMER_CONFIG_TIMER_NUM LEDC_TIMER_0
#define TIMER_CONFIG_FREQ_HZ 5000
#define TIMER_CONFIG_CLK_CFG LEDC_AUTO_CLK

// LED configuration defines
#define CHANNEL_CONFIG_GPIO 4
#define CHANNEL_CONFIG_SPEEDMODE LEDC_LOW_SPEED_MODE
#define CHANNEL_CONFIG_CHANNEL LEDC_CHANNEL_0
#define CHANNEL_CONFIG_TIMER_SEL LEDC_TIMER_0
#define CHANNEL_CONFIG_INTR_TYPE LEDC_INTR_DISABLE
#define CHANNEL_CONFIG_DUTY 0
#define CHANNEL_CONFIG_HPOINT 0

// Other defines
#define MAX_DUTY 255

void app_main(void)
{
    /**
     * Refer to C:\esp\v6.1\esp-idf\components\esp_driver_ledc\include\driver\ledc.h
     * Initialize a ledc_timer_config_t struct using the configuration parameters define above
     * Call ledc_timer_config() and pass in your struct to configure the LEDC timer
     * 
     * Then, initialize a ledc_channel_config_t struct using the configuration parameters define above
     * Call ledc_channel_config() and pass in your struct to configure the LEDC channel
     * create a uint32_t variable to keep track of the LED current duty cycle
     */


    for (;;) {   
        /**
         * In the while loop, 
         * Increment the duty cycle until it reaches MAX_DUTY, making sure to set and update the duty cycle of the LEDC channel
         * call vTaskDelay(pdMS_TO_TICKS(10)) to introduce a delay between duty cycle changes
         * Then, decrement the duty cycle until it reaches 0, making sure to set and update the duty cycle of the LEDC channel
         * call vTaskDelay(pdMS_TO_TICKS(10)) to introduce a delay between duty cycle changes
         * 
         */
    }

}

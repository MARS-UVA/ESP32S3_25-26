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
    ledc_timer_config_t ledc_timer = {
        .speed_mode = TIMER_CONFIG_SPEEDMODE,
        .duty_resolution = TIMER_CONFIG_RESOLUTION,
        .timer_num = TIMER_CONFIG_TIMER_NUM,
        .freq_hz = TIMER_CONFIG_FREQ_HZ,
        .clk_cfg = TIMER_CONFIG_CLK_CFG
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));


    ledc_channel_config_t ledc_channel = {
        .gpio_num = CHANNEL_CONFIG_GPIO, 
        .speed_mode = CHANNEL_CONFIG_SPEEDMODE,
        .channel = CHANNEL_CONFIG_CHANNEL,
        .timer_sel = CHANNEL_CONFIG_TIMER_SEL,
        .intr_type = CHANNEL_CONFIG_INTR_TYPE,
        .duty = CHANNEL_CONFIG_DUTY,     
        .hpoint = CHANNEL_CONFIG_HPOINT
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    uint32_t currentDuty = 0;

    for (;;) {   
        printf("Varying brightness\n");
        currentDuty = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        
        while(currentDuty < MAX_DUTY) {
            currentDuty++;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, currentDuty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10 milliseconds
        }
        while(currentDuty > 0) {
            currentDuty--;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, currentDuty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10 milliseconds
        }
    }

}

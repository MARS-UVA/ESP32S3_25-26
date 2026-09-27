#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO 4

void app_main(void)
{   
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    for (;;) {
        gpio_set_level(LED_GPIO, 1); // Set GPIO level to high
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second

        gpio_set_level(LED_GPIO, 0); // Set GPIO level to low
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

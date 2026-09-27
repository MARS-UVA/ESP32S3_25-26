#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO 4

void app_main(void)
{   
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    // Infinite loop to blink the LED
    for (;;) {
        /**
         * To blink the LED, we want to set the GPIO level to high, wait for a
         * second, set the GPIO level to low, and wait for another second.
         *
         * In this loop, you will need to use the 'gpio_set_level()' function to
         * set the GPIO level, and the 'vTaskDelay()' function to wait for a
         * second. Note that 'vTaskDelay()' takes a number of ticks and not
         * seconds, so you will need to convert the number of seconds to ticks
         * using the 'pdMS_TO_TICKS()' macro.
         *
         * You can find the documentation for gpio_set_level() here:
         * https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html
         * 
         * You can find the documentation for vTaskDelay() and pdMS_TO_TICKS() 
         * here:
         * https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/system/freertos.html
        */
        
    }
}

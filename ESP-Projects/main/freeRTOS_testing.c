#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "CL_CAN.h"

#define LED_PIN 18  // GPIO pin number for the LED

TaskHandle_t blinkTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;

void Blink_Task(void *arg)
{
    // Configure LED GPIO as output
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    int level = 0;
    while(1) {
        // Toggle LED state
        level = !level;
        gpio_set_level(LED_PIN, level);  // Set LED level
        ESP_LOGI("Blink_Task", "LED pin %d set to %d", LED_PIN, level);
        vTaskDelay(pdMS_TO_TICKS(500));  // Wait for 500ms
    }
}

void Motor_Task(void *arg)
{
    while(1){
        printf("Motor_Task printing..\n");
        talonPercentOut(256); // 50% of max velocity
        vTaskDelay(5);
    }
}

void app_main(void)
{
    canSetup();

    setPIDValues(0.1f, 0.1f, 0.0f);

    xTaskCreate(Blink_Task, "Blink_Task", 4096, NULL, 10, &blinkTaskHandle);
    xTaskCreatePinnedToCore(Motor_Task, "Motor_Task", 4096, NULL, 10, &motorTaskHandle, 1);
}
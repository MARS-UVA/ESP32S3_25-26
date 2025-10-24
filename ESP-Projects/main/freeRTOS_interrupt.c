#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "CL_CAN.h"

TaskHandle_t blinkTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;

void Blink_Task(void *arg)
{
    while(1){
        
        printf("Blink_Task printing..\n");

        /* Configure the peripheral according to the LED type */
        configure_led();

        while (1) {
            ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
            blink_led();
            /* Toggle the LED state */
            s_led_state = !s_led_state;
            vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        }
    }
}

void Motor_Task(void *arg)
{
    while(1){
        printf("Motor_Task printing..\n");
        setTargetVelocity(512); // 50% of max velocity
        vTaskDelay(5);
    }
}

void app_main(void)
{
    motorSetup();
    setPIDValues();

   xTaskCreate(Blink_Task, "Blink_Task", 4096, NULL, 10, &blinkTaskHandle);
   xTaskCreatePinnedToCore(Motor_Task, "Motor_Task", 4096, NULL, 10, &motorTaskHandle, 1);
 }
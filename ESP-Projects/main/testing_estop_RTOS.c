#include "CAN.h"
#include <driver/gpio.h>

#define BUTTONPIN 5

TaskHandle_t myTaskHandle1 = NULL;
TaskHandle_t myTaskHandle2 = NULL;

void canRunRTOS(void *arg){
    while (1)  {
    talonPercentOut(100);   
    ESP_LOGI("motor running", "motor running");
    }
}

void readButton(){

    while (true){
        ESP_LOGI("LOOP RUN", "LOOP RUN");{
        if (gpio_get_level(BUTTONPIN)) {
            vTaskSuspend(myTaskHandle1);
            ESP_LOGI("cut_loop", "cut_loop");
            }
        else {
            vTaskResume(myTaskHandle1);
        }
        }
    }
}

void app_main(){

    gpio_set_direction(BUTTONPIN, GPIO_MODE_INPUT);
    canSetup();

    xTaskCreatePinnedToCore(canRunRTOS, "canRunRTOS", 4096, NULL, 7, &myTaskHandle1, 1);
    xTaskCreatePinnedToCore(readButton, "readButton", 4096, NULL, 1, &myTaskHandle2, 1);
}


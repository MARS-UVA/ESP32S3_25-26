#include "CAN.h"
#include <driver/gpio.h>

#define BUTTONPIN 5

TaskHandle_t myTaskHandle = NULL;
TaskHandle_t myTaskHandle2 = NULL;

void canRunRTOS(void *arg)
{
talonPercentOut(100);   
}

int readButton(){

return gpio_get_level(BUTTONPIN);

}

void canStopRTOS(void *arg){

        while (true){

            ESP_LOGI("LOOP RUN", "LOOP RUN");{
            if (readButton()) {
              canStop();
              break;
              ESP_LOGI("cut_loop", "cut_loop");
            }
        }
    }
}

void app_main(){

    gpio_set_direction(BUTTONPIN, GPIO_MODE_INPUT);

    xTaskCreate(canRunRTOS, "canRunRTOS", 1024, NULL, 1, &myTaskHandle);
    // xTaskCreatePinnedToCore(canStopRTOS, "canStopRTOS", 1024, NULL, 1, &myTaskHandle2)
}


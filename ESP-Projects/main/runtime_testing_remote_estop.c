#include "CAN.h"
#include <driver/gpio.h>

#define BUTTONPIN 5

// gpio_config_t io_conf = {
//     .pin_bit_mask = (1ULL << BUTTONPIN),
//     .mode = GPIO_MODE_INPUT
//     .pull_up_en = GPIO_PULLUP_DISABLE,
//     .pull_down_en = GPIO_PULLDOWN_DISABLE,
//     .intr_type = GPIO_INTR_DISABLE

// };


int readButton(){

return gpio_get_level(BUTTONPIN);



}

void app_main(){

gpio_set_direction(BUTTONPIN, GPIO_MODE_INPUT);
canSetup();


    while (true) {

        ESP_LOGI("LOOP RUN", "LOOP RUN");
        if (readButton()) {
            canStop();
            ESP_LOGI("cut_loop", "cut_loop");
            break;
        }
        
        talonPercentOut(512);

    }
}

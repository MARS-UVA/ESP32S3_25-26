#include <driver/gpio.h>
#include "./packs/uart/uart.h"
#include "./packs/can/can.h"


//uart goals

//input

//output

//response





// #define BUTTONPIN 5

// TaskHandle_t myTaskHandle1 = NULL;
// TaskHandle_t myTaskHandle2 = NULL;

// void init(void) 
// {
//     const uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_DEFAULT,
//     };

//     // We won't use a buffer for sending data.
//     uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
//     uart_param_config(UART_NUM, &uart_config);
//     uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
// }

// void canRunRTOS(void *arg){
//     while (1)  {
//     talonPercentOut(100);   
//     ESP_LOGI("motor running", "motor running");
//     }
// }

// void readButton(){

//     while (true){
//         ESP_LOGI("LOOP RUN", "LOOP RUN");{
//         if (gpio_get_level(BUTTONPIN)) {
//             vTaskSuspend(myTaskHandle1);
//             ESP_LOGI("cut_loop", "cut_loop");
//             }
//         else {
//             vTaskResume(myTaskHandle1);
//         }
//         }
//     }
// }

void test_callback(SerialPacket packet, void* userdata1, void* userdata2) {

    ESP_LOGI("responding", "THIS IS DATA RECIEVED FROM UART!!!: %x",  packet.back_right_wheel);

}

void app_main(){   

    UART_setup();

    while (true) {

        UART_callback(0x32, test_callback, 0, 0);

    }

}


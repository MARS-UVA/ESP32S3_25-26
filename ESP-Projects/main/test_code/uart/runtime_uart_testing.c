#include <driver/gpio.h>
// #include "./packs/uart/uart.h"
// #include "./packs/can/can.h"
#include "uart.h"
#include "can.h"


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

/*void test_callback(SerialPacket packet, void* userdata1, void* userdata2) {

    ESP_LOGI("responding", "THIS IS DATA RECIEVED FROM UART!!!: %x",  packet.back_right_wheel);

}*/

void app_main(){   
    
    const uart_config_t uart_config = {
        .baud_rate = 2000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = 0,
        .stop_bits = 1,
        .flow_ctrl = 0,
        .source_clk = 4,
    };

    const int uart_buffer_size_rx = (1024 * 2); // setup UART buffered RX IO with event queue
    const int uart_buffer_size_tx = (1024 * 2); // setup UART buffered TX IO with event queue

    // ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, uart_buffer_size_rx, uart_buffer_size_tx, 0, NULL, 0));
    // ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config)); // apply config
    // ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 43, 44, 18, 19));    // [tx, rx] - board -> [43, 44] - esp32s3 | [1, 3] - esp32 devkit v1

    ESP_LOGI("UART_SETUP", "Done");
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, uart_buffer_size_rx, uart_buffer_size_tx, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    //ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, S3_TX_PIN, S3_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); // used to be 43, 44, but those pins are used for CAN so changed to 23, 24
    
    //10 is Tx, 11 is Rx for gpio UART
    uart_set_pin(UART_NUM_1, 10, 11, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t invalid;
    uint8_t header;

    uint8_t front_left_wheel;
    uint8_t back_left_wheel;
    uint8_t front_right_wheel;
    uint8_t back_right_wheel;
    uint8_t front_bucket_drum;
    uint8_t front_actuator;
    uint8_t back_bucket_drum;
    uint8_t back_actuator;

    ControlPacket_OneRobot test_packet = {
        
    }
    
    // control_queue = xQueueCreate(1, sizeof(ControlPacket_OneRobot));

    while (true) {
        UART_write(, 10);
        //UART_callback(0x32, test_callback, 0, 0);

    }

}


#include "driver/uart.h"

const uart_port_t uart_num = UART_NUM_2; 
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
    .rx_flow_ctrl_thresh = 122,


// Setup UART buffered IO with event queue
const int uart_buffer_size = (1024 * 2);
QueueHandle_t uart_queue;


// Read data from UART
const uart_port_t uart_num = UART_NUM_2;
uint8_t data[128];
int length = 0;


};
// Configure UART parameteres

void app_main(){

ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

// Set UART pins(TX: I04, RX: I05, RTS: I018, CTS: I019)
ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 4, 5, 18, 19))

// Install UART driver using an event queue here
ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, \
                                    uart_buffer_size, 10, &uart_queue, 0));
                                
// Read
ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
length = uart_read_bytes(uart_num, data, length, 100);

// Write
char* test_str = "This is a test string.\n";
uart_write_bytes(uart_num, (const char*)test_str, strlen(test_str));

}

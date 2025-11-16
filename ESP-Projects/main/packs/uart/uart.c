#include "./uart.h"




/* --------------------- Functions ------------------ */

void UART_setup()
{
    uart_config_t uart_config = {     
        .baud_rate = 115200, // parameters subject to change
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,

    };
    
    QueueHandle_t uart_queue;
    const int uart_buffer_size = (1024 * 2); // setup UART buffered IO with event queue


    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config)); // apply config
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 43, 44, 18, 19)); // sets UART pins(TX: GPI04, RX: GPI05, RTS: GPI018, CTS: GPI019)
    ESP_ERROR_CHECK(uart_driver_install(
        UART_NUM_0, 
        uart_buffer_size, // RX buffer size
        uart_buffer_size, // TX buffer size
        10,               // queue size
        &uart_queue,      // handle to queue
        0                 // no flags
    )); 

}

SerialPacket UART_read()
{
    SerialPacket packet = {0};
    packet.invalid = 1;

    const uint8_t packetLength = 8; // expected packet length (1 header plus 1 byte for each motor/actuator)
    uint8_t RxBuffer[8] = {0,0,0,0,0,0,0,0}; // zeroinit this so it doesn't have garbage data

    uart_read_bytes(
        UART_NUM_0, 
        RxBuffer, 
        packetLength, 
        50 // this is timeout
    );
    
    if (RxBuffer[0] == 0xFF){
	    packet.invalid = 0;
	    packet.header = RxBuffer[1];
	    packet.top_left_wheel = RxBuffer[2];
	    packet.back_left_wheel = RxBuffer[3];
	    packet.top_right_wheel  = RxBuffer[4];
		packet.back_right_wheel = RxBuffer[5];
		packet.drum  = RxBuffer[6];
		packet.actuator  = RxBuffer[7];
    }

    return packet;
}



// change back to 
void UART_write() // writes a single packet to Jetson on UART
{
    char* test_str = "This is a test string.\n";
    uart_write_bytes(UART_NUM_0, (const char*)test_str, strlen(test_str));

}

void UART_callback(uint8_t reg, void (*callback)(SerialPacket, void*, void*), void* userdata1, void* userdata2) {

    SerialPacket packet = UART_read();

    if (packet.invalid == false && packet.header == reg) {
        callback(packet, userdata1, userdata2);
    }

}




#include "driver/uart.h"
#include "driver/gpio.h"
#include "CAN.h"

/* --------------------- Variables ------------------ */

const uart_port_t uart_num = UART_NUM_2; // sets UART port

uart_config_t uart_config = {     
    .baud_rate = 115200, // parameters subject to change
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
    .rx_flow_ctrl_thresh = 122,

};

const int uart_buffer_size = (1024 * 2); // setup UART buffered IO with event queue
QueueHandle_t uart_queue;
// change


typedef struct serialPacket {
   /* uint8_t opcode;
    uint8_t payload_size;
    uint8_t *payload;
    uint8_t invalid;
    */
  int8_t invalid;
  uint8_t header;
  uint8_t top_left_wheel;
  uint8_t back_left_wheel;
  uint8_t top_right_wheel;
  uint8_t back_right_wheel;
  uint8_t drum;
  uint8_t actuator;
} SerialPacket;


/* --------------------- Functions ------------------ */

void UART_setup()
{
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config)); // apply config
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 4, 5, 18, 19)); // sets UART pins(TX: GPI04, RX: GPI05, RTS: GPI018, CTS: GPI019)
    ESP_ERROR_CHECK(uart_driver_install(
        uart_num, 
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

    const uint8_t packetLength = 7; // expected packet length (1 header plus 1 byte for each motor/actuator)
    uint8_t RxBuffer[packetLength]; // buffer to store received bytes

    int len = uart_read_bytes(
        uart_num, 
        RxBuffer, 
        packetLength, 
        100
    );
    
    if (len == packetLength){

	    packet.invalid = 0;
	    packet.header = RxBuffer[0];
	    packet.top_left_wheel = RxBuffer[1];
	    packet.back_left_wheel = RxBuffer[2];
	    packet.top_right_wheel  = RxBuffer[3];
		packet.back_right_wheel = RxBuffer[4];
		packet.drum  = RxBuffer[5];
		packet.actuator  = RxBuffer[6];
    }
    return packet;
}


// change back to 
void UART_write() // writes a single packet to Jetson on UART
{
    char* test_str = "This is a test string.\n";
    uart_write_bytes(uart_num, (const char*)test_str, strlen(test_str));
}
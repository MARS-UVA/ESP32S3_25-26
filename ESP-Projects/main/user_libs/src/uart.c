#include "uart.h"
#include "tasks.h"

QueueHandle_t control_queue;
/* --------------------- Functions ------------------ */

void UART_setup()
{
    // uart_config_t uart_config = {
    //     .baud_rate = 115200, // parameters subject to change
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     // .rx_flow_ctrl_thresh = 122,
    //     .source_clk = UART_SCLK_DEFAULT,
    // };

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = 3,
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

    uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    control_queue = xQueueCreate(1, sizeof(ControlPacket_OneRobot));
}

void UART_read(ControlPacket_OneRobot *packet)
{
    const uint8_t packetLength = sizeof(ControlPacket_OneRobot) - 1; // expected packet length (1 header plus 1 byte for each motor/actuator)

    packet->header = 0; // Reset packet header
    uart_read_bytes(
        UART_NUM_1,
        (char *)packet + 1,
        packetLength,
        0 // this is timeout
    );

    if (packet->header == 0xFF)
    {
        packet->invalid = 0;
    }
}

// change back to
void UART_write(CurrVoltPacket_OneRobot *packet) // writes a single packet to Jetson on UART
{
    // char* cPacket = (char*)packet;
    const int txBytes = uart_write_bytes(UART_NUM_1, packet, sizeof(CurrVoltPacket_OneRobot));
    // char *test_str = "This is a test string.\n";
    // const int txBytes2 = uart_write_bytes(UART_NUM_1, test_str, strlen(test_str));
}

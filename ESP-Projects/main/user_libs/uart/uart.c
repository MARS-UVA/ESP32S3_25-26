#include "uart.h"

/* --------------------- Functions ------------------ */
QueueHandle_t uart_queue; // queue stores the serial packet values

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
    uart_queue = xQueueCreate(1, sizeof(SerialPacket));
}

void UART_read(SerialPacket *packet)
{
    const uint8_t packetLength = 8;                 // expected packet length (1 header plus 1 byte for each motor/actuator)
    uint8_t RxBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // zeroinit this so it doesn't have garbage data

    uart_read_bytes(
        UART_NUM_1,
        RxBuffer,
        packetLength,
        0 // this is timeout
    );

    if (RxBuffer[0] == 0xFF)
    {
        packet->invalid = 0;
        packet->header = RxBuffer[1];
        packet->top_left_wheel = RxBuffer[2];
        packet->back_left_wheel = RxBuffer[3];
        packet->top_right_wheel = RxBuffer[4];
        packet->back_right_wheel = RxBuffer[5];
        packet->drum = RxBuffer[6];
        packet->actuator = RxBuffer[7];
    }
}

// change back to
void UART_write(OutPacket *packet) // writes a single packet to Jetson on UART
{
    // char* cPacket = (char*)packet;
    const int txBytes = uart_write_bytes(UART_NUM_1, packet, sizeof(OutPacket));
    // char *test_str = "This is a test string.\n";
    // const int txBytes2 = uart_write_bytes(UART_NUM_1, test_str, strlen(test_str));
}

void UART_callback(uint8_t reg, void (*callback)(SerialPacket, void *, void *), void *userdata1, void *userdata2)
{
    // SerialPacket packet = UART_read();
    // if (packet.invalid == false && packet.header == reg)
    //{
    //  callback(packet, userdata1, userdata2);
    //}
}

void UART_rx_task()
{
    SerialPacket pkt = {1, 0, 0, 0, 0, 0, 0, 0};

    while (1)
    {
        UART_read(&pkt);
        if (pkt.invalid == 0)
        {
            xQueueOverwrite(uart_queue, &pkt);
        }
        vTaskDelay(1);
    }
}
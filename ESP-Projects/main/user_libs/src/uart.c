#include "uart.h"
#include "tasks.h"

QueueHandle_t control_queue;
QueueHandle_t temperature_queue;
QueueHandle_t current_voltage_queue;
QueueHandle_t position_queue;
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
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, S3_TX_PIN, S3_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); // used to be 43, 44, but those pins are used for CAN so changed to 23, 24
    control_queue = xQueueCreate(1, sizeof(ControlPacket_OneRobot));
    temperature_queue = xQueueCreate(1, sizeof(TempPacket_OneRobot));
    current_voltage_queue = xQueueCreate(1, sizeof(CurrVoltPacket_OneRobot));
    position_queue = xQueueCreate(1, sizeof(PositionPacket_OneRobot));
}

/*
void UART_read(ControlPacket_OneRobot *packet)
{
    const uint8_t packetLength = sizeof(ControlPacket_OneRobot); // expected packet length (1 header plus 1 byte for each motor/actuator)

    packet->header = 0; // Reset packet header
    uart_read_bytes(
        UART_NUM_1,
        (char *)packet,
        packetLength,
        0 // this is timeout
    );

    // if (packetLength <= 0)
    // {
    //     return;
    // }

    if (packet->invalid == 0xFF)
    {
        packet->invalid = 0;
        packet->front_left_wheel = 127;
        packet->

    }
}
*/

void UART_read(ControlPacket_OneRobot *packet)
{
    const uint8_t packetLength = 10;
    uint8_t RxBuffer[10] = {00, 127, 127, 127, 127, 127, 127, 127, 127, 127}; // zeroinit this so it doesn't have garbage data

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
        packet->front_left_wheel = RxBuffer[2];
        packet->back_left_wheel = RxBuffer[3];
        packet->front_right_wheel = RxBuffer[4];
        packet->back_right_wheel = RxBuffer[5];
        packet->front_bucket_drum = RxBuffer[6];
        packet->front_actuator = RxBuffer[7];
        packet->back_bucket_drum = RxBuffer[8];
        packet->back_actuator = RxBuffer[9];
    }
}

void UART_write(void *packet, size_t size)
{
    uart_write_bytes(UART_NUM_1, packet, size);
}
